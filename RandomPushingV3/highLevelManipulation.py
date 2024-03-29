import json
import time
import numpy as np
import robotic as ry
import open3d as o3d
from typing import Tuple
from arenas import RectangularArena
from scanning import getScannedObject
from cubeEstimator import estimate_cube_pose
from raiUtils import (createWaypointFrame,
                      setupConfig,
                      startupRobot,
                      basicKomo,
                      komoStraightPath)
from utils import extract_position_and_quaternion


class Robot():
    def __init__(self,
                 real_robot: bool=False,
                 max_velocity: float=1.,
                 initial_object_position: np.ndarray=np.array([-.55, -.1, .69]),
                 object_dimensions: np.ndarray=np.array([.12, .04, .04])):
        
        self.C = setupConfig(real_robot, initial_object_position, object_dimensions)

        self.komo = None
        self.arena = None
        self.max_velocity = max_velocity
        self.on_real = real_robot
        self.obj_pos = initial_object_position
        self.obj_dims = object_dimensions

        self.gripperWidth = .075
        self.table_height = .651
        self.gripper_z_depth = .01

        arena_pos = np.array([-.5, -.1, .69])
        arena_pos[2] = self.table_height
        arena_dims = np.array([.25, .30])

        self.action_arena = RectangularArena(arena_pos, width=arena_dims[0], height=arena_dims[1], name="pushArena")
        self.action_arena.display(self.C, color=[.5, .5, .5])

        self.scan_arena = RectangularArena(arena_pos - np.array([.15, 0, 0]), width=arena_dims[0]+.15, height=arena_dims[1]+.2, name="scanArena")
        self.scan_arena.display(self.C, color=[1., 1., 1.])

        # self.C.addFrame("pivot_wall") \
        #     .setShape(ry.ST.ssBox, [.05, arena_dims[0], .3, .0001]) \
        #     .setPosition([initial_object_position[0] - .025 - object_dimensions[0]*.5, arena_pos[1], arena_pos[2]+.15]) \
        #     .setColor([.5, .5, .5]) \
        #     .setContact(1)
        
        self.bot = startupRobot(self.C, real_robot)

        createWaypointFrame(self.C, "predicted_obj", initial_object_position)
        
        initial_object_position[2] -= .02
        self.C.addFrame("objective_pos") \
            .setShape(ry.ST.ssBox, [*object_dimensions, .0001]) \
            .setPosition(initial_object_position) \
            .setColor([0, 1, 0, .5]) \
            .setContact(0)
        
        # A bit redundant to have this, self.obj_pos, self.obj_dims and "predicted_obj" frame. Will have to fix...
        self.C.addFrame("or1").setPosition([0, 0, 0])
        self.C.addFrame("predicted_obj_frame", "or1") \
            .setShape(ry.ST.ssBox, [*object_dimensions, .0001]) \
            .setRelativePosition(initial_object_position) \
            .setColor([1, 0, 1, .5]) \
            .setContact(0)
        
        self.initial_object_position = initial_object_position
        self.initial_object_dimensions = object_dimensions


    def goHome(self):
        self.bot.home(self.C)


    def updateObjectPosition(self):
        """
        Look towards the region where the object is predicted to be and take
        a the point cloud. If there are no point through an error, the object
        has been lost! Otherwise proceed to get the center-point of the point
        cloud and update the predicted object position to said center-point.
        """
        
        ### Look towards the object (for now from a non specified angle)
        self.komo = basicKomo(self.C)
        self.komo.addControlObjective([], 0, .1)
        self.komo.addObjective([1.], ry.FS.scalarProductZZ, ["l_gripper", "table"], ry.OT.ineq, [-1e1], [np.cos(np.deg2rad(100))])
        self.komo.addObjective([1.], ry.FS.scalarProductZZ, ["l_gripper", "table"], ry.OT.ineq, [1e1], [np.cos(np.deg2rad(60))])
        self.komo.addObjective([1.], ry.FS.positionRel, ["predicted_obj", "cameraWrist"], ry.OT.eq, [1e1], [.0, .0, .4])
        self.moveBlocking()

        ### Take the point cloud and update the predicted object position
        mid_point, point_cloud_, dims = getScannedObject(self.bot, self.C, self.scan_arena)
        if not len(mid_point):
            raise Exception("Lost the object!")
        
        self.C.getFrame("predicted_obj").setPosition(mid_point)
        self.obj_pos = mid_point
        self.obj_dims = dims
        # Make up for the table cutoff
        self.obj_dims[2] += .014
        self.obj_pos[2] -= .007

        # Update estimated object frame
        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(point_cloud_)
        pose_mat = estimate_cube_pose(point_cloud, self.initial_object_dimensions, add_noise=True, origin=self.initial_object_position, verbose=0)
        position, quaternion = extract_position_and_quaternion(pose_mat)
        self.C.getFrame("or1").setPosition(position).setQuaternion(quaternion)


    def pushObject(self, push_end_position: np.ndarray=np.array([]), start_distance: float=.1, save_as: str="") -> bool:
        """
        Move through the predicted position of the object in a straight line
        in the direction of a random point in the playing field (or arena).
        Alternatively the point specified in the push_end_position parameter.
        
        This fuction will also update the predicted position of the object,
        setting it equal to the end position of the gripper in the push
        motion.

        TODO: start_distance should be dependent on the object#s dimensions
        """

        self.gripperClose() # Our gripper should be closed for a better push

        ### First define the start and end waypoints TODO: replace this chunk with define_start_and_end_waypoints
        # If there is no objective specified generate one from the arena
        if not len(push_end_position):
            push_end_position = self.action_arena.randomPointInside(self.table_height)

        push_end_position[2] = self.obj_pos[2]
        
        direction = push_end_position - self.obj_pos
        direction /= np.linalg.norm(direction)
        start_pos = self.obj_pos - direction * start_distance

        # Temporary #
        start_pos[2] = .69
        push_end_position[2] = .69
        #############

        createWaypointFrame(self.C, "safe_start", start_pos + np.array([0, 0, .1]))
        createWaypointFrame(self.C, "push_start", start_pos)
        createWaypointFrame(self.C, "push_end", push_end_position)

        ### Define the motion problem and solve it if possible
        self.komo: ry.KOMO = basicKomo(self.C, phases=3, enableCollisions=self.on_real)

        # Get the gripper into it's correct rotation
        self.komo.addObjective([1, 3], ry.FS.vectorZ, ["l_gripper"], ry.OT.eq, [1e1], [0., 0., 1.])
        self.komo.addObjective([1, 3], ry.FS.vectorY, ["l_gripper"], ry.OT.eq, [1e1], -direction)

        # Define the straight motion through push start and end
        self.komo = komoStraightPath(self.C, self.komo, ["safe_start", "push_start"], phases=[1, 2])
        # We already told komo to go to the start_push waypoint in the previous komoStraightPath so we set gotoPoints to false
        self.komo = komoStraightPath(self.C, self.komo, ["push_start", "push_end"], phases=[2, 3], gotoPoints=False)
        self.komo.addObjective([3], ry.FS.positionDiff, ["l_gripper", "push_end"], ry.OT.eq, [1e1])

        success, external_taus, join_states = self.moveBlockingAndCheckForce()

        # Should put data saving in a sepparate function
        if len(save_as):
            push_attempt = {
                "success": success,
                "objectPosition": self.obj_pos.tolist()
            }
            if success:
                push_attempt["tauExternal"] = external_taus
                push_attempt["jointStates"] = join_states
            else:
                push_attempt["currentState"] = self.C.getJointState().tolist()
                push_attempt["objective"] = push_end_position.tolist()
            
            json.dump(push_attempt, open(save_as, "w"))

        ### If the push motion was successful move back a bit
        if success:
            self.C.getFrame("predicted_obj").setPosition(push_end_position)
            self.obj_pos = push_end_position
            self.moveBack()

        return success
    

    def define_start_and_end_waypoints(self):
        """
        TODO:
        """
        # If there is no objective specified generate one from the arena
        if not len(push_end_position):
            push_end_position = self.action_arena.randomPointInside(self.table_height)

        push_end_position[2] = self.obj_pos[2]
        
        direction = push_end_position - self.obj_pos
        direction /= np.linalg.norm(direction)
        start_pos = self.obj_pos - direction * start_distance

        # Temporary #
        start_pos[2] = .69
        push_end_position[2] = .69
        #############

        createWaypointFrame(self.C, "safe_start", start_pos + np.array([0, 0, .1]))
        createWaypointFrame(self.C, "push_start", start_pos)
        createWaypointFrame(self.C, "push_end", push_end_position)
    

    def pullObject(self, pull_end_position: np.ndarray=np.array([])) -> bool:
        """
        This function will only work on the real robot as the simulation
        can not calculate if the maxForceAllowed is exceeded.
        """

        self.gripperClose() # Our gripper should be closed for a better pull

        if not len(pull_end_position):
            pull_end_position = self.action_arena.randomPointInside(self.table_height)

        createWaypointFrame(self.C, "pull_init_start", self.obj_pos + np.array([0, 0, .1]))
        createWaypointFrame(self.C, "pull_init_end", self.obj_pos + np.array([0, 0, self.obj_dims[2]*.5])-.012)

        ### Put the gripper on top of the object
        self.komo: ry.KOMO = basicKomo(self.C, phases=3, enableCollisions=self.on_real)
        self.komo.addObjective([1, 3], ry.FS.scalarProductZZ, ["l_gripper", "table"], ry.OT.ineq, [-1e1], [.99])
        self.komo = komoStraightPath(self.C, self.komo, ["pull_init_start", "pull_init_end"], [2, 3])

        success, _, _ = self.moveBlockingAndCheckForce(maxForceAllowed=7, speed=.5, verbose=1)

        ### Perform pull motion towards the endpoint
        self.bot.sync(self.C, 0)
        z_pos = self.C.getFrame("l_gripper").getPosition()[2]
        createWaypointFrame(self.C, "pull_end_pos", pull_end_position)

        self.komo: ry.KOMO = basicKomo(self.C, phases=1, enableCollisions=self.on_real, slices=35)
        self.komo.addObjective([], ry.FS.scalarProductZZ, ["l_gripper", "table"], ry.OT.ineq, [-1e1], [.99])
        self.komo.addObjective([], ry.FS.position, ["l_gripper"], ry.OT.eq, [0, 0, 1e1], [0, 0, z_pos])
        self.komo.addObjective([1], ry.FS.position, ["l_gripper"], ry.OT.eq, [1e1, 1e1, 0], pull_end_position)

        success = self.moveBlocking()

        if success:
            pull_end_position[2] = self.obj_pos[2]
            self.C.getFrame("predicted_obj").setPosition(pull_end_position)
            self.obj_pos = pull_end_position
        
        return success
    

    def pullRotateObject(self, angle_cos: float) -> bool:
        """
        This function will only work on the real robot as the simulation
        can not calculate if the maxForceAllowed is exceeded.
        """

        self.gripperClose() # Our gripper should be closed for a better pull

        createWaypointFrame(self.C, "pull_init_start", self.obj_pos + np.array([0, 0, .1]))
        createWaypointFrame(self.C, "pull_init_end", self.obj_pos + np.array([0, 0, self.obj_dims[2]*.5])-.012)

        ### Put the gripper on top of the object
        self.komo: ry.KOMO = basicKomo(self.C, phases=3, enableCollisions=self.on_real)
        self.komo.addObjective([1, 3], ry.FS.vectorZ, ["l_gripper"], ry.OT.eq, [1e1], [0, 0, 1])
        self.komo = komoStraightPath(self.C, self.komo, ["pull_init_start", "pull_init_end"], [2, 3])
        self.komo.addObjective([2, 3], ry.FS.scalarProductXY, ["table", "l_gripper"], ry.OT.eq, [1e1], [0])

        success, _, _ = self.moveBlockingAndCheckForce(maxForceAllowed=7, speed=.5, verbose=1)

        ### Put the gripper on top of the object
        self.bot.sync(self.C, 0)
        current_pos = self.C.getFrame("l_gripper").getPosition()
        current_pos[2] -= .01

        self.komo: ry.KOMO = basicKomo(self.C, phases=1, enableCollisions=self.on_real)
        self.komo.addObjective([], ry.FS.position, ["l_gripper"], ry.OT.eq, [1e1], current_pos)
        self.komo.addObjective([], ry.FS.vectorZ, ["l_gripper"], ry.OT.eq, [1e1], [0, 0, 1])
        self.komo.addObjective([1], ry.FS.scalarProductXY, ["table", "l_gripper"], ry.OT.eq, [1e1], [angle_cos])

        success = self.moveBlocking()
        
        return success
    

    def tableSideGrasp(self) -> bool:

        self.gripperOpen()

        self.komo: ry.KOMO = basicKomo(self.C, phases=2, enableCollisions=False)
        end_pos = self.obj_pos + np.array([0, self.obj_dims[1]*.5+.05, -.04 + .04])
        self.komo.addObjective([1], ry.FS.position, ["l_gripper"], ry.OT.eq, [1e1], end_pos + np.array([0, 0, .3]))
        self.komo.addObjective([2], ry.FS.position, ["l_gripper"], ry.OT.eq, [1e1], end_pos)
        self.komo.addObjective([1, 2], ry.FS.scalarProductYZ, ["table", "l_gripper"], ry.OT.ineq, [-1e1], [.96])
        self.komo.addObjective([1, 2], ry.FS.scalarProductXZ, ["l_gripper", "table"], ry.OT.eq, [1e1], [1])

        success = self.moveBlocking(verbose=1)

        if success:
            self.moveBack(-.07, dir="z")
            self.gripperClose()
        
        return success
    

    def approachPoint(self, point: np.ndarray, enableCollisions: bool=True) -> bool:

        self.komo = basicKomo(self.C, enableCollisions=enableCollisions)
        self.komo.addObjective([1], ry.FS.position, ["l_gripper"], ry.OT.eq, [1e1], point)

        return self.moveBlocking()
    

    def pivot(self) -> bool:
        """
        In this function we assume that the object is positioned against a wall
        and is also perpendicular to it.
        """
    
        self.gripperClose()

        # Create helper waypoints and coordinates
        axis_point = self.obj_pos + np.array([-self.obj_dims[0]*.5, 0, self.obj_dims[2]*.5 - .04])
        
        initial_pos = self.obj_pos + np.array([self.obj_dims[0]*.5 + .1, 0, self.obj_dims[2]*.5 - .04])
        contact_point = initial_pos - np.array([.1, 0, 0])
        end_pos = self.obj_pos + np.array([
            self.obj_dims[2]*.5 - self.obj_dims[0]*.5, 0, self.obj_dims[0] - self.obj_dims[2]*.5-.025])

        self.approachPoint(initial_pos + np.array([0, 0, .15]))

        createWaypointFrame(self.C, "inital_pivot_pos", initial_pos, color=[1, 0, 0])
        createWaypointFrame(self.C, "touch_obj_pivot", contact_point, color=[0, 1, 0])
        createWaypointFrame(self.C, "pivot_axis_point", axis_point, color=[0, 0, 1])

        mid_point0 = contact_point + np.array([0, 0, .05])
        dist_to_keep = np.linalg.norm(axis_point - mid_point0)
        angle = np.deg2rad(32)
        rot = np.array([[np.cos(angle), 0, np.sin(angle)],
                        [0., 1., 0.],
                        [-np.sin(angle), 0, np.cos(angle)]])
        mid_point1 = rot.T @ (mid_point0 - axis_point) + axis_point
    
        createWaypointFrame(self.C, "pivot_mid_point0", mid_point0, color=[1, 1, 0])
        createWaypointFrame(self.C, "pivot_mid_point1", mid_point1, color=[1, 1, 0])
        createWaypointFrame(self.C, "pivot_end_point", end_pos, color=[1, 1, 0])

        # Pivot motion
        self.komo = basicKomo(self.C, phases=5, slices=20, enableCollisions=True)

        self.komo.addObjective([1, 5], ry.FS.vectorZ, ["l_gripper"], ry.OT.eq, [1e1], [0., 0., 1.])
        self.komo.addObjective([1, 5], ry.FS.vectorY, ["l_gripper"], ry.OT.eq, [1e1], [1., 0., 0.])

        self.komo = komoStraightPath(self.C, self.komo, ["inital_pivot_pos", "touch_obj_pivot"], [1, 2])

        self.komo = komoStraightPath(self.C, self.komo, ["touch_obj_pivot", "pivot_mid_point0"], [2, 3], gotoPoints=False)
        self.komo.addObjective([3], ry.FS.position, ["l_gripper"], ry.OT.eq, [1e1], mid_point0)
        
        self.komo.addObjective([3, 4], ry.FS.position, ["l_gripper"], ry.OT.eq, [0, 1, 0], initial_pos)
        self.komo.addObjective([3, 4], ry.FS.negDistance, ["l_gripper", "pivot_axis_point"], ry.OT.eq, [1e1], [-dist_to_keep])
        self.komo.addObjective([4], ry.FS.position, ["l_gripper"], ry.OT.eq, [1e1, 0, 0], mid_point1)

        self.komo = komoStraightPath(self.C, self.komo, ["pivot_mid_point1", "pivot_end_point"], [4, 5], gotoPoints=False)
        self.komo.addObjective([5], ry.FS.position, ["l_gripper"], ry.OT.eq, [1e1], end_pos)

        # TODO: update pos and dim values of the object
        return self.moveBlocking(verbose=3, speed=.5)


    def placeGraspMotion(self, end_position: np.ndarray, x_orientation: str) -> bool:

        createWaypointFrame(self.C, "grasp_start_pos", end_position + np.array([0, 0, .1]))
        createWaypointFrame(self.C, "grasp_mid_pos", end_position + np.array([0, 0, .05])) # This avoids getting too close to the object
        createWaypointFrame(self.C, "grasp_end_pos", end_position)

        self.komo: ry.KOMO = basicKomo(self.C, phases=3, enableCollisions=self.on_real)
        
        self.komo.addObjective([1, 3], ry.FS.vectorZ, ["l_gripper"], ry.OT.eq, [1e1], [0., 0., 1.])
        self.komo = komoStraightPath(self.C, self.komo, ["grasp_start_pos", "grasp_mid_pos"])
        self.komo.addObjective([], ry.FS.position, ["l_gripper"], ry.OT.ineq, [0., 0., -100.], end_position[2])
        self.komo.addObjective([3], ry.FS.positionDiff, ["l_gripper", "grasp_end_pos"], ry.OT.eq, [1e1])

        # If no orientation of the gripper is specified we try both options
        if x_orientation == "x":
            self.komo.addObjective([1, 3], ry.FS.vectorX, ["l_gripper"], ry.OT.eq, [1e1], [1., 0., 0.])
        elif x_orientation == "y":
            self.komo.addObjective([1, 3], ry.FS.vectorX, ["l_gripper"], ry.OT.eq, [1e1], [0., 1., 0.])
        else:
            raise Exception(f"Value '{x_orientation}' is not a valid grasp direction!")

        success = self.moveBlocking()

        return success


    def graspObject(self, x_orientation: str="") -> bool:
        """
        We can get a rough estimate of the objects dimensions through the point cloud.
        With this information we try to find a feasible grasp with the gripper z vector
        perpendicular to the table/floor.
        TODO: Make this work for more than just x and y angles.
        """

        self.gripperOpen()
            
        # If the arm is exerting too much force the orientation of the gripper is probably invalid and we must stop

        gripper_end_position = self.obj_pos
        gripper_end_position[2] = self.obj_dims[2] + self.table_height - self.gripper_z_depth

        # If no orientation of the gripper is specified we try both options
        x_orientation = ["x", "y"] if not len(x_orientation) else [x_orientation]
        for orien in x_orientation:

            if orien == "x":
                if self.obj_dims[0] > self.gripperWidth:
                    continue
                success = self.placeGraspMotion(gripper_end_position, "x")
            
            elif orien == "y":
                if self.obj_dims[1] > self.gripperWidth:
                    continue
                success = self.placeGraspMotion(gripper_end_position, "y")
            
            else:
                raise Exception(f"Value '{orien}' is not a valid grasp direction!")

            self.gripperClose()

            if success:
                self.moveBack(dir="z")

            return success
        
        print("No feasible grasp angle.")
        return False
    

    def placeObject(self, end_position: np.ndarray=np.array([]), x_orientation: str="x") -> bool:

        # If no position is specified we place the object at a random spot in the arena.
        if not len(end_position):
            end_position = self.action_arena.randomPointInside()

        gripper_end_position = np.array([end_position[0], end_position[1], 0.])
        gripper_end_position[2] = self.obj_dims[2] + self.table_height - self.gripper_z_depth

        success = self.placeGraspMotion(gripper_end_position, x_orientation)
        self.gripperOpen()

        if success:
            self.C.getFrame("predicted_obj").setPosition(end_position)
            self.obj_pos = end_position
            self.moveBack(dir="z")

        return success


    def moveBack(self, how_much: float=.1, dir: str="y"):
        """
        Move in the direction of the y vector of the gripper by "how_much" in a straight line
        """

        # Create waypoint frames
        initial_gripper_pos = self.C.getFrame("l_gripper").getPosition()
        initial_gripper_rot = self.C.getFrame("l_gripper").getQuaternion()
        self.C.addFrame("retreat_start_pos") \
            .setShape(ry.ST.sphere, size=[.01, .01]) \
            .setQuaternion(initial_gripper_rot) \
            .setPosition(initial_gripper_pos)

        end_frame = self.C.addFrame("retreat_end_pos", "retreat_start_pos") \
            .setShape(ry.ST.sphere, size=[.01, .01]) \
            .setColor([1, 1, 0])
        
        # Set movement direction
        if dir == "y":
            end_frame_rel_pos = np.array([.0, how_much, .0])
        elif dir == "z":
            end_frame_rel_pos = np.array([.0, .0, how_much])
        else:
            raise Exception(f"Value '{dir}' is not a valid retreat direction!")
        
        end_frame.setRelativePosition(end_frame_rel_pos)
        
        # Define komo
        self.komo = basicKomo(self.C, 1, enableCollisions=False)
        self.komo = komoStraightPath(self.C, self.komo, ["retreat_start_pos", "retreat_end_pos"], phases=[0, 1])

        # Move
        self.moveBlocking()


    """
    The following two functions (moveBlocking and moveBlockingAndCheckForce)
    are used to interface with BotOp. It is worth considering if we want them
    to stay in this class or if (together with self.bot) they should be
    handled by some other module in the code so as to leave "Robot" as just a
    motion calculator without any actual motion execution.
    """

    def gripperClose(self):
        self.bot.gripperMove(ry._left, width=.0, speed=1)
        while not self.bot.gripperDone(ry._left):
            self.bot.sync(self.C, .1)

    def gripperOpen(self):
        self.bot.gripperMove(ry._left, width=.075, speed=1)
        while not self.bot.gripperDone(ry._left):
            self.bot.sync(self.C)

    def moveBlocking(self, verbose: int=0, speed: float=1.) -> bool:
    
        ret = ry.NLP_Solver() \
            .setProblem(self.komo.nlp()) \
            .setOptions(stopTolerance=1e-2, verbose=0) \
            .solve()
        
        if verbose: print(ret)
        if verbose > 1 and ret.feasible:
            self.komo.view(True)

        movement_possible = bool(ret.feasible)
        if movement_possible:
            
            self.bot.moveAutoTimed(self.komo.getPath(), self.max_velocity) # moveAutoTimed prints out two numbers, idk why
            while self.bot.getTimeToEnd() > 0:
                self.bot.sync(self.C, .1)

        return movement_possible

        
    def moveBlockingAndCheckForce(self,
                                  maxForceAllowed: float=np.nan,
                                  speed: float=1.,
                                  verbose: int=0, compliance: bool=False) -> Tuple[bool, list, list]:
        
        """
        Unlike moveBlocking, this function also measures the forces applied
        on the robot's gripper.
        """

        ret = ry.NLP_Solver() \
            .setProblem(self.komo.nlp()) \
            .setOptions(stopTolerance=1e-2, verbose=0) \
            .solve()
        
        if verbose: print(ret)
        if verbose > 1 and ret.feasible: self.komo.view(True)

        # Prepare lists to store data
        max_force = -np.inf
        prev_t = -1
        joint_states = []
        external_taus = []

        movement_possible = bool(ret.feasible)
        if movement_possible:
            
            self.bot.moveAutoTimed(self.komo.getPath(), speed)

            tic_time = time.monotonic()
            while self.bot.getTimeToEnd() > 0:

                tic_time += .01
                now_time = time.monotonic()
                if tic_time > now_time:
                    time.sleep(tic_time - now_time)
                
                # Measure force on the direction of the Y vector of the gripper
                y, J = self.C.eval(ry.FS.position, ['l_gripper'], [[0, 1, 0]])
                if compliance:
                    self.bot.setCompliance(J, .5)
                J = np.linalg.pinv(J.T)
                tauExternal = self.bot.get_tauExternal()
                F = np.abs(J @ tauExternal)
                
                # Store measuremets and positions
                joint_states.append(self.bot.get_q().tolist())
                external_taus.append(tauExternal.tolist())

                # if prev_t == self.bot.get_t():
                #     print("Broke doe to stalling!")
                #     break

                # prev_t = self.bot.get_t()

                # Check if max force is exceeded
                if verbose:
                    print("Current force on l_gripper: ", F[0])
                    print("t of bot: ", prev_t)
                max_force = F[0] if F[0] > max_force else max_force

                self.bot.sync(self.C, .0)
                if max_force > maxForceAllowed:
                    self.bot.stop(self.C)
                    print("Max force exceeded! Robot stopped")
                    break
            if verbose: print("Max force enacted: ", max_force)
            self.bot.sync(self.C)

        return movement_possible, external_taus, joint_states

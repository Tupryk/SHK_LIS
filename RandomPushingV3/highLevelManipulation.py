import json
import time
import numpy as np
import robotic as ry
from typing import Tuple
from arenas import RectangularArena
from scanning import getScannedObject
from raiUtils import (createWaypointFrame,
                      setupConfig,
                      startupRobot,
                      basicKomo,
                      komoStraightPath)


class Robot():
    def __init__(self,
                 real_robot: bool=False,
                 max_velocity: float=1.,
                 initial_object_position: np.ndarray=np.array([-.5, -.1, .69]),
                 object_dimensions: np.ndarray=np.array([.05, .05, .05])):
        
        self.C = setupConfig(real_robot, initial_object_position)
        self.bot = startupRobot(self.C, real_robot)
        self.komo = None
        self.arena = None
        self.max_velocity = max_velocity
        self.on_real = real_robot
        self.obj_pos = initial_object_position
        self.obj_dims = object_dimensions

        self.gripperWidth = .075
        self.table_height = .651
        self.gripper_z_depth = .01

        createWaypointFrame(self.C, "predicted_obj", initial_object_position)

        arena_pos = initial_object_position.copy()
        arena_pos[2] = self.table_height
        arena_dims = np.array([.25, .30])

        self.push_arena = RectangularArena(arena_pos, width=arena_dims[0], height=arena_dims[1], name="pushArena")
        self.push_arena.display(self.C, color=[.5, .5, .5])

        self.scan_arena = RectangularArena(arena_pos, width=arena_dims[0]+.2, height=arena_dims[1]+.2, name="scanArena")
        self.scan_arena.display(self.C, color=[1., 1., 1.])


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
        self.komo.addObjective([1.], ry.FS.positionRel, ["predicted_obj", "cameraWrist"], ry.OT.eq, [1e1], [.0, .0, .4])
        self.moveBlocking()

        ### Take the point cloud and update the predicted object position
        mid_point, _, dims = getScannedObject(self.bot, self.C, self.scan_arena)
        if not len(mid_point):
            raise Exception("Lost the object!")
        
        self.C.getFrame("predicted_obj").setPosition(mid_point)
        self.obj_pos = mid_point
        self.obj_dims = dims


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

        ### First define the start and end waypoints
        obj_pos = self.C.getFrame("predicted_obj").getPosition()

        # If there is no objective specified generate one from the arena
        if not len(push_end_position):
            push_end_position = self.push_arena.randomPointInside()

        push_end_position[2] = obj_pos[2]
        
        direction = push_end_position - obj_pos
        direction /= np.linalg.norm(direction)
        start_pos = obj_pos - direction * start_distance

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
                "objectPosition": obj_pos.tolist()
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
            self.moveBack()

        return success
    

    def graspObject(self, x_orientation: str="", place: bool=False) -> bool:
        """
        We can get a rough estimate of the objects dimensions through the point cloud.
        With this information we try to find a feasible grasp with the gripper z vector
        perpendicular to the table/floor.
        """

        if not place:
            self.gripperOpen()
            
        # If the arm is exerting too much force the orientation of the gripper is probably invalid and we must stop

        gripper_end_position = self.obj_pos
        gripper_end_position[2] = self.obj_dims[2] + self.table_height - self.gripper_z_depth

        createWaypointFrame(self.C, "grasp_start_pos", gripper_end_position + np.array([0, 0, .1]))
        createWaypointFrame(self.C, "grasp_end_pos", gripper_end_position)

        self.komo: ry.KOMO = basicKomo(self.C, phases=2, enableCollisions=self.on_real)

        self.komo.addObjective([1, 2], ry.FS.vectorZ, ["l_gripper"], ry.OT.eq, [1e1], [0., 0., 1.])
        self.komo = komoStraightPath(self.C, self.komo, ["grasp_start_pos", "grasp_end_pos"])

        # If no orientation of the gripper is specified we try both options
        x_orientation = ["x", "y"] if not len(x_orientation) else [x_orientation]
        for orien in x_orientation:

            if orien == "x":
                if self.obj_dims[0] > self.gripperWidth:
                    continue
                self.komo.addObjective([1, 2], ry.FS.vectorX, ["l_gripper"], ry.OT.eq, [1e1], [1., 0., 0.])
            elif orien == "y":
                if self.obj_dims[1] > self.gripperWidth:
                    continue
                self.komo.addObjective([1, 2], ry.FS.vectorX, ["l_gripper"], ry.OT.eq, [1e1], [0., 1., 0.])
            else:
                raise Exception(f"Value '{orien}' is not a valid grasp direction!")

            success, _, _ = self.moveBlockingAndCheckForce()

            if place:
                self.gripperOpen()
            else:
                self.gripperClose()

            return success
        
        print("No feasible grasp angle.")
        return False


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
        self.bot.gripperClose(ry._left, speed=.2)
        while not self.bot.gripperDone(ry._left):
            self.bot.sync(self.C)


    def gripperOpen(self):
        self.bot.gripperMove(ry._left, width=.079, speed=.1)
        while not self.bot.gripperDone(ry._left):
            self.bot.sync(self.C)

    def moveBlocking(self, verbose: int=0) -> bool:
    
        ret = ry.NLP_Solver() \
            .setProblem(self.komo.nlp()) \
            .setOptions(stopTolerance=1e-2, verbose=0) \
            .solve()
        
        if verbose: print(ret)
        if verbose > 1 and ret.feasible: self.komo.view(True)

        movement_possible = bool(ret.feasible)
        if movement_possible:
            
            self.bot.moveAutoTimed(self.komo.getPath(), self.max_velocity) # moveAutoTimed prints out two numbers, idk why
            while self.bot.getTimeToEnd() > 0:
                self.bot.sync(self.C, .1)

        return movement_possible

        
    def moveBlockingAndCheckForce(self,
                                  maxForceAllowed: float=np.nan,
                                  verbose: int=0) -> Tuple[bool, list, list]:
        
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
        joint_states = []
        external_taus = []

        movement_possible = bool(ret.feasible)
        if movement_possible:
            
            self.bot.moveAutoTimed(self.komo.getPath(), self.max_velocity)

            tic_time = time.monotonic()
            while self.bot.getTimeToEnd() > 0:

                tic_time += .1
                now_time = time.monotonic()
                if tic_time > now_time:
                    time.sleep(tic_time - now_time)
                
                # Measure force on the direction of the Y vector of the gripper
                y, J = self.C.eval(ry.FS.position, ['l_gripper'], [[0, 1, 0]])
                J = np.linalg.pinv(J.T)
                tauExternal = self.bot.get_tauExternal()
                F = np.abs(J @ tauExternal)
                
                # Store measuremets and positions
                joint_states.append(self.bot.get_q().tolist())
                external_taus.append(tauExternal.tolist())

                # Check if max force is exceeded
                max_force = F if F > max_force else max_force
                if max_force > maxForceAllowed:
                    print("Max force exceeded!")
                    break
            
            if verbose: print("Max force enacted: ", max_force)
            self.bot.sync(self.C)

        return movement_possible, external_taus, joint_states

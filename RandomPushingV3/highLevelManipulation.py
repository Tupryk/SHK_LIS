import numpy as np
import robotic as ry
from typing import Tuple
import matplotlib.pyplot as plt
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
                 initial_object_position: np.ndarray=np.array([-.5, -.1, .69])):
        
        self.C = setupConfig(real_robot, initial_object_position)
        self.bot = startupRobot(self.C, real_robot)
        self.komo = None
        self.arena = None
        self.max_velocity = max_velocity

        """
        Push attempts stores the following data after each push:
            {
                success: bool
                forces: [float]
            }
        """
        self.push_attempts = []

        createWaypointFrame(self.C, "predicted_obj", initial_object_position)

        TABLE_CENTER = np.array([-.23, -.16, .651])
        TABLE_DIMS = np.array([.89, .55])
        ROBOT_POS = np.array([-.03, -.24, .651])
        ROBOT_RING = .29

        self.push_arena = RectangularArena(middleP=TABLE_CENTER, width=TABLE_DIMS[0]*.8, height=TABLE_DIMS[1]*.8, middlePCirc=ROBOT_POS, innerR=ROBOT_RING, name="pushArena")
        self.push_arena.plotArena(self.C, color=[.5, .5, .5])

        self.scan_arena = RectangularArena(middleP=TABLE_CENTER, width=TABLE_DIMS[0]+.2, height=TABLE_DIMS[1]+.2, middlePCirc=ROBOT_POS, innerR=ROBOT_RING*.5, name="scanArena")
        self.scan_arena.plotArena(self.C, color=[1., 1., 1.])


    def updateObjectPosition(self):
        """
        Look towards the region where the object is predicted to be and take
        a the point cloud. If there are no point through an error, the object
        has been lost! Otherwise proceed to get the center-point of the point
        cloud and update the predicted object position to said center-point.
        """
        
        ### Look towards the object (for now from a non specified angle)
        self.komo = basicKomo(self.C)
        self.komo.addObjective([1.], ry.FS.positionRel, ["predicted_obj", "cameraWrist"], ry.OT.eq, [1e1], [.0, .0, .4])
        self.moveBlocking()

        ### Take the point cloud and update the predicted object position
        mid_point, _ = getScannedObject(self.bot, self.C, self.scan_arena)
        if not len(mid_point):
            raise Exception("Lost the object!")
        
        self.C.getFrame("predicted_obj").setPosition(mid_point)


    def pushObject(self, objective: np.ndarray=np.array([]), start_distance: float=.2):
        """
        Move through the predicted position of the object in a straight line
        in the direction of a random point in the playing field (or arena).
        Alternatively the point specified in the objective parameter.
        
        This fuction will also update the predicted position of the object,
        setting it equal to the end position of the gripper in the push
        motion.
        """

        ### First define the start and end waypoints
        obj_pos = self.C.getFrame("predicted_obj").getPosition()

        # If there is no objective specified generate one from the arena
        if not len(objective):
            objective = self.push_arena.randomPointInArena()
            while self.push_arena.point_in_inner_circ(objective, self.push_arena.middlePCirc, self.push_arena.innerR): # This repetition could be made better
                objective = self.push_arena.randomPointInArena()

        objective[2] = obj_pos[2]
        
        direction = objective - obj_pos
        direction /= np.linalg.norm(direction)
        start_pos = obj_pos - direction * start_distance

        # Temporary #
        start_pos[2] = .69
        objective[2] = .69
        #############

        createWaypointFrame(self.C, "safe_start", start_pos + np.array([0, 0, .1]))
        createWaypointFrame(self.C, "push_start", start_pos)
        createWaypointFrame(self.C, "push_end", objective)

        ### Define the motion problem and solve it if possible
        self.komo: ry.KOMO = basicKomo(self.C, phases=3, enableCollisions=False)

        # Get the gripper into it's correct rotation
        self.komo.addObjective([1, 3], ry.FS.vectorZ, ["l_gripper"], ry.OT.eq, [1e1], [0., 0., 1.])
        self.komo.addObjective([1, 3], ry.FS.vectorY, ["l_gripper"], ry.OT.eq, [1e1], -direction)

        # Define the straight motion through push start and end
        self.komo = komoStraightPath(self.C, self.komo, ["safe_start", "push_start"], phases=[1, 2])
        # We already told komo to go to the start_push waypoint in the previous komoStraightPath so we set gotoPoints to false
        self.komo = komoStraightPath(self.C, self.komo, ["push_start", "push_end"], phases=[2, 3], gotoPoints=False)
        self.komo.addObjective([3], ry.FS.positionDiff, ["l_gripper", "push_end"], ry.OT.eq, [1e1])

        success = self.moveBlockingAndCheckForce()

        ### If the push motion was successful move back a bit
        if success:
            self.C.getFrame("predicted_obj").setPosition(objective)
            self.moveBack()


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
        print(self.moveBlocking())

    def displayResults(self, index_of_push_attempt_to_plot: int=-1):
        """
        While pushing some data gets stored like the forces ejected on
        the gripper and if the push was executed correctly.
        """
        success_count = len([pa["success"] for pa in self.push_attempts if pa["success"]])
        total_attempts = len(self.push_attempts)
        perc = success_count / total_attempts * 100
        print(f"Succeded in {success_count} of {total_attempts} push attempts ({perc:.2f}%).")

        if self.push_attempts[index_of_push_attempt_to_plot]["success"]:
            forces = [f for f in self.push_attempts["forces"]]
            plt.plot(forces)
            plt.show()


    """
    The following two functions (moveBlocking and moveBlockingAndCheckForce)
    are used to interface with BotOp. It is worth considering if we want them
    to stay in this class or if (together with self.bot) they should be
    handled by some other module in the code so as to leave "Robot" as just a
    motion calculator without any actual motion execution.
    """
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
                                  verbose: int=0) -> Tuple[bool, float]:
        
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

        max_force = -np.inf
        forces = []
        movement_possible = bool(ret.feasible)
        if movement_possible:
            
            self.bot.moveAutoTimed(self.komo.getPath(), self.max_velocity)

            while self.bot.getTimeToEnd() > 0:
                
                self.bot.sync(self.C, .1)
                
                # Measure force on the direction of the Y vector of the gripper
                y, J = self.C.eval(ry.FS.position, ['l_gripper'], [[0, 1, 0]])
                F = np.abs(J @ self.bot.get_tauExternal())
                
                # Store measured force
                forces.append(F)
                max_force = F if F > max_force else max_force
                if max_force > maxForceAllowed:
                    print("Max force exceeded!")
                    break
            
            if verbose: print("Max force enacted: ", max_force)
        
        self.push_attempts.append({"success": movement_possible, "forces": forces})
        return movement_possible, max_force

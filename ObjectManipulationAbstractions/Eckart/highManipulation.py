import numpy as np
import robotic as ry
from typing import Union
import matplotlib.pyplot as plt


# Side note: Maybe it could be interesting if there is a mode where all of the komos get calculated beforehand and executed in the end.

class RobotMan():

    def __init__(self, config_file: str="scenarios/pandaSingle.g", block_placement: str="", real_robot: bool=False):
        
        self.C = ry.Config()
        self.C.addFile(ry.raiPath(config_file))
        
        if block_placement == "bridge":
            self.createNewBlock("block1", [.35, .19, .67], color=[1, 0, 0], dims=[.1, .05, .05])
            self.createNewBlock("block2", [.3, .04, .67], color=[0, 1, 0], dims=[.1, .05, .05])
            self.createNewBlock("block3", [.35, -.11, .67], color=[0, 0, 1], dims=[.1, .05, .05])
        elif not block_placement:
            self.createNewBlock("block", [-.3, .04, .67])
            self.createNewBlock("block1", [.35, .19, .67], color=[1, 0, 0])
            self.createNewBlock("block2", [.3, .04, .67], color=[0, 1, 0])
            self.createNewBlock("block3", [.35, -.11, .67], color=[0, 0, 1])
            self.createNewBlock("long_block", [.0, .1, .67], color=[1, 1, 0], dims=[.05, .1, .05])
            self.createNewBlock("long_block_objective", [.0, .3, .67], color=[1, .5, .5], dims=[.05, .05, .05])
        else:
            raise Exception(f'The block placement {block_placement} does not exist!')
        
        self.C.view()
        
        self.bot = ry.BotOp(self.C, real_robot)

        self.komo = None

        self.times = 0

        self.blockGraspOrientations = {
            "zx": [ry.FS.scalarProductXY, ry.FS.scalarProductXZ, ry.FS.scalarProductYZ],
            "zy": [ry.FS.scalarProductYY, ry.FS.scalarProductXZ, ry.FS.scalarProductYZ],
            "yx": [ry.FS.scalarProductXY, ry.FS.scalarProductXZ, ry.FS.scalarProductZZ],
            "yz": [ry.FS.scalarProductXX, ry.FS.scalarProductXZ, ry.FS.scalarProductZZ],
            "xy": [ry.FS.scalarProductYY, ry.FS.scalarProductYZ, ry.FS.scalarProductZZ],
            "xz": [ry.FS.scalarProductYX, ry.FS.scalarProductYZ, ry.FS.scalarProductZZ]
        }

    
    def goHome(self):
        self.bot.home(self.C)


    def createNewBlock(self, name: str, position: Union[np.ndarray, list], color: [float]=[1., .5, .0], dims: [float]=[.05, .05, .05]):
        self.C.addFrame(name) \
            .setPosition(position) \
            .setShape(ry.ST.ssBox, size=[*dims, .002]) \
            .setColor(color) \
            .setContact(True) \
            .setMass(1e-2)


    def initKomo(self, phases: int=1, slicesPerPhase: int=20, enableCollisions: bool=True) -> ry.KOMO:

        q_now = self.C.getJointState()

        self.komo = ry.KOMO()
        self.komo.setConfig(self.C, enableCollisions)
        self.komo.setTiming(phases, slicesPerPhase, 1., 2)

        self.komo.addControlObjective([], 1, 1)
        self.komo.addControlObjective([], 2, 1)

        if enableCollisions:
            self.komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq)

        self.komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq)
        self.komo.addObjective([], ry.FS.qItself, [], ry.OT.sos, [.1], q_now)

        self.komo.addObjective([phases], ry.FS.qItself, [], ry.OT.eq, [10.], [], 1)


    def moveToPointFreely(self, point: Union[np.ndarray, list]):
        self.initKomo(phases=1)
        self.komo.addControlObjective([], 0, 1e1)
        self.komo.addObjective([1], ry.FS.position, ["l_gripper"], ry.OT.eq, [1e1], point)
        self.solveAndExecuteProblem()


    def moveToPointZLock(self, point: Union[np.ndarray, list]):
        self.initKomo(phases=1)
        self.komo.addObjective([1], ry.FS.position, ["l_gripper"], ry.OT.eq, [1e1], point)
        self.komo.addObjective([1], ry.FS.vectorZ, ["l_gripper"], ry.OT.eq, [1e1], [0, 0, 1])
        self.solveAndExecuteProblem()


    def moveToPointWithRotation(self):
        pass


    def pathMustBeStraight(self, frames: [str]=[], points: [np.ndarray]=[], phases: [int]=[1, 2], gotoPoints=False):

        if len(frames) == 2:
            delta = self.C.getFrame(frames[1]).getPosition()-self.C.getFrame(frames[0]).getPosition()
            delta /= np.linalg.norm(delta)
            mat = np.eye(3) - np.outer(delta, delta)
            self.komo.addObjective(phases, ry.FS.positionDiff, ['l_gripper', frames[0]], ry.OT.eq, mat)

            if gotoPoints:
                self.komo.addObjective([phases[0]], ry.FS.positionDiff, ['l_gripper', frames[0]], ry.OT.eq, [1e1])
                self.komo.addObjective([phases[1]], ry.FS.positionDiff, ['l_gripper', frames[1]], ry.OT.eq, [1e1])

        elif len(points) == 2:
            self.createWaypointFrame("straight_point", points[0], color=[1., .5, .0])
            delta = points[1]-points[0]
            delta /= np.linalg.norm(delta)
            mat = np.eye(3) - np.outer(delta, delta)
            self.komo.addObjective(phases, ry.FS.positionDiff, ['l_gripper', "straight_point"], ry.OT.eq, mat)

            if gotoPoints:
                self.komo.addObjective([phases[0]], ry.FS.position, ['l_gripper'], ry.OT.eq, [1e1], points[0])
                self.komo.addObjective([phases[1]], ry.FS.position, ['l_gripper'], ry.OT.eq, [1e1], points[1])

        else:
            raise Exception('Invalid input lengths:')
            

    def grabBlock(self, objFrame: str="block", grasp_direction: str=''):
        """
        """

        initialGrab_pos = self.C.getFrame(objFrame).getPosition() + np.array([0, 0, .1]) # This should maybe take the width of the object into account
        self.createWaypointFrame("grab_initial", initialGrab_pos)

        graspOptions = ["x", "y"] if not grasp_direction else [grasp_direction]
        for i in graspOptions:
            self.initKomo(phases=2, enableCollisions=False)

            self.komo.addObjective([1], ry.FS.positionDiff, ["l_gripper", "grab_initial"], ry.OT.eq, [1e1])
            self.komo.addObjective([2], ry.FS.positionDiff, ["l_gripper", objFrame], ry.OT.eq, [1e1])
            self.komo.addObjective([1, 2], ry.FS.vectorZ, ["l_gripper"], ry.OT.eq, [1e1], [0, 0, 1])

            self.pathMustBeStraight(["grab_initial", objFrame])
            
            if i == 'x':
                self.komo.addObjective([1, 2], ry.FS.scalarProductXY, [objFrame, "l_gripper"], ry.OT.eq, [1e0])
            elif i == 'y':
                self.komo.addObjective([1, 2], ry.FS.scalarProductXX, [objFrame, "l_gripper"], ry.OT.eq, [1e0])
            else:
                raise Exception(f'Grasp direction "{i}" not defined:')
            
            success = self.solveAndExecuteProblem()
            if success:
                self.gripperClose()
                return

        raise Exception("Grasp motion was not possible :(")
        

    def placeBlock(self, where: Union[np.ndarray, str], gripperX_aligned: str=""):
        """
        Supposes the robot is currently holding a block
        """
        options = ["x", "y"] if not gripperX_aligned else [gripperX_aligned]
        for i in options:

            self.initKomo(phases=1, enableCollisions=False)

            if type(where) == str:
                self.komo.addObjective([1], ry.FS.positionRel, ["l_gripper", where], ry.OT.eq, [1e1], [.0, .0, .075])
            else:
                self.komo.addObjective([1], ry.FS.position, ["l_gripper"], ry.OT.eq, [1e1], where)

            self.komo.addObjective([1], ry.FS.vectorZ, ["l_gripper"], ry.OT.eq, [1e1], [0, 0, 1])

            if i == "x":
                self.komo.addObjective([1], ry.FS.vectorX, ["l_gripper"], ry.OT.eq, [1e1], [1, 0, 0])
            elif i == "y":
                self.komo.addObjective([1], ry.FS.vectorX, ["l_gripper"], ry.OT.eq, [1e1], [0, 1, 0])
            else:
                raise Exception(f'Grasp direction "{i}" not defined:')
                
            success = self.solveAndExecuteProblem()
            if success:
                self.gripperOpen()
                return

        raise Exception("Place motion was not possible :(")
    

    def placeBlockRotated(self, where: str, rotation_type: str=""):
        """
        Supposes the robot is currently holding a block
        """
        options = ["+x", "-x", "+y", "-y"] if not rotation_type else [rotation_type]
        for i in options:

            self.initKomo(phases=1)

            self.komo.addObjective([1], ry.FS.positionRel, ["l_gripper", where], ry.OT.eq, [1e1], [.0, .0, .1])

            if i == "+x":
                self.komo.addObjective([1], ry.FS.vectorY, ["l_gripper"], ry.OT.eq, [1e1], [0, 0, 1])
                self.komo.addObjective([1], ry.FS.vectorX, ["l_gripper"], ry.OT.eq, [1e1], [0, 1, 0])
            elif i == "-x":
                self.komo.addObjective([1], ry.FS.vectorY, ["l_gripper"], ry.OT.eq, [1e1], [0, 0, -1])
                self.komo.addObjective([1], ry.FS.vectorX, ["l_gripper"], ry.OT.eq, [1e1], [0, 1, 0])
            elif i == "+y":
                # TODO
                raise Exception("Not implemented")
            elif i == "-y":
                # TODO
                raise Exception("Not implemented")
                
            success = self.solveAndExecuteProblem()
            if success:
                self.gripperOpen()
                return

        raise Exception("Place motion was not possible :(")

    
    def pushBlock(self, objFrame: str="block", push_direction: str="", push_length: float=.1, obj_radious: float=.025):

        graspOptions = ["-x", "+x", "-y", "+y"] if not push_direction else [push_direction]
        for i in graspOptions:
            obj_pos = self.C.getFrame(objFrame).getPosition()

            if i == "-x":
                start_dir = np.array([1, 0, 0])
            elif i == "+x":
                start_dir = np.array([-1, 0, 0])
            elif i == "-y":
                start_dir = np.array([0, 1, 0])
            elif i == "+y":
                start_dir = np.array([0, -1, 0])
            else:
                raise Exception(f'Push direction "{i}" not defined:')
            
            offset_start = obj_radious*4

            start_pos = obj_pos + start_dir*offset_start
            end_pos = obj_pos - start_dir*push_length

            self.createWaypointFrame("start", start_pos)
            self.createWaypointFrame("end", end_pos)

            delta = end_pos-start_pos
            delta /= np.linalg.norm(delta)

            self.initKomo(2, enableCollisions=False)
            
            self.komo.addObjective([1, 2], ry.FS.vectorX, ['l_gripper'], ry.OT.eq, delta.reshape(1,3))
            self.komo.addObjective([1, 2], ry.FS.vectorZ, ['l_gripper'], ry.OT.eq, [1], [0,0,1])

            self.komo.addObjective([1, 2], ry.FS.distance, ["l_gripper", "start"], ry.OT.sos, [1e1])
            self.komo.addObjective([1, 2], ry.FS.distance, ["l_gripper", "end"], ry.OT.sos, [1e1])

            mat = np.eye(3) - np.outer(delta, delta)
            self.komo.addObjective([1, 2], ry.FS.positionDiff, ['l_gripper', "start"], ry.OT.eq, mat)

            self.komo.addObjective([1], ry.FS.positionDiff, ['l_gripper', "start"], ry.OT.eq, [1e1])
            self.komo.addObjective([2], ry.FS.positionDiff, ['l_gripper', "end"], ry.OT.eq, [1e1])

            success = self.solveAndExecuteProblem()
            if success: return

        raise Exception("Push motion was not possible :(")


    def createWaypointFrame(self, name: str, position: np.ndarray, color: [float]=[0., 1., 0.]) -> ry.Frame:
        way = self.C.getFrame(name)
        if not way:
            way = self.C.addFrame(name) \
            .setShape(ry.ST.sphere, size=[.01, .002])
        way.setPosition(position).setColor(color)
        return way


    def moveBack(self, how_much: float=.1, dir: str="y"):
        """
        Move in the direction of the y vector of the gripper by "how_much" in a straight line
        """

        # Create waypoint frames
        initial_gripper_pos = self.C.getFrame("l_gripper").getPosition()
        self.createWaypointFrame("retreat_start_pos", initial_gripper_pos)

        end_frame = self.C.addFrame("retreat_end_pos", "l_gripper") \
            .setShape(ry.ST.sphere, size=[.01, .002])
        
        # Set movement direction
        if dir == "y":
            end_frame_rel_pos = [.0, how_much, .0]
        elif dir == "z":
            end_frame_rel_pos = [.0, .0, how_much]
        else:
            raise Exception(f"Value '{dir}' is not a valid retreat direction!")
        
        end_frame.setRelativePosition(end_frame_rel_pos)
        
        # Define komo
        self.initKomo(phases=1)
        self.pathMustBeStraight(frames=["retreat_start_pos", "retreat_end_pos"], phases=[0, 1])

        # Move
        success = self.solveAndExecuteProblem()
        if not success:
            raise Exception("Moving back was not possibles :(")


    def viewMovementGraph(self):
        q = self.komo.getPath()
        plt.plot(q)
        plt.show()


    def gripperClose(self):
        self.bot.gripperClose(ry._left, speed=.2)
        while not self.bot.gripperDone(ry._left):
            self.bot.sync(self.C)


    def gripperOpen(self):
        self.bot.gripperMove(ry._left, width=.08, speed=.1)
        while not self.bot.gripperDone(ry._left):
            self.bot.sync(self.C)


    def solveAndExecuteProblem(self, verbose: int=0) -> bool:

        ret = ry.NLP_Solver(self.komo.nlp(), verbose=0).solve()
    
        if not ret.feasible:
            if verbose:
                print("The motion problem defined is not feasible!")
                self.komo.view_play(True)
            return False

        if verbose:
            self.viewMovementGraph()
            self.komo.getReport(True)
            self.komo.view_play(True)

        q = self.komo.getPath()
        self.bot.moveAutoTimed(q)
        while self.bot.getTimeToEnd() > 0:
            self.bot.sync(self.C)

        return True
    
    ### EXPERIMENTAL STUFF ###
    def grabBlockSide(self, objFrame: str="block", grasp_direction: str="") -> ry.KOMO():
        """
        grasp a box with a centered top grasp (axes fully aligned)
        """
        options = self.blockGraspOrientations.items() if not grasp_direction else [grasp_direction]
        for i in options:
            self.initKomo(phases=1)

            try:
                align = self.blockGraspOrientations[i]
            except:
                raise Exception(f'Grasp direction "{i}" not defined:')

            # position: centered
            self.komo.addObjective([1], ry.FS.positionDiff, ["l_gripper", objFrame], ry.OT.eq, [1e1])

            # orientation: grasp axis orthogonal to target plane X-specific
            self.komo.addObjective([1], align[0], [objFrame, "l_gripper"], ry.OT.eq, [1e0])
            self.komo.addObjective([1], align[1], [objFrame, "l_gripper"], ry.OT.eq, [1e0])
            self.komo.addObjective([1], align[2], [objFrame, "l_gripper"], ry.OT.eq, [1e0])

            success = self.solveAndExecuteProblem()
            if success: return

        print("Grasp motion was not possible :(")

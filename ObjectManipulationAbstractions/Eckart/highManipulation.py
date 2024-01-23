import numpy as np
import robotic as ry
from typing import Union
import matplotlib.pyplot as plt


class RobotMan():

    def __init__(self, total_time: int, config_file: str="scenarios/pandaSingle.g", real_robot: bool=False):
        
        self.C = ry.Config()
        self.C.addFile(ry.raiPath(config_file))
        
        self.createNewBlock("block", [-.3, .04, .67])
        self.createNewBlock("block1", [.3, -.06, .67], color=[1, 0, 0])
        self.createNewBlock("block2", [.3, .04, .67], color=[0, 1, 0])
        self.createNewBlock("block3", [.3, .14, .67], color=[0, 0, 1])
        
        self.C.view()
        
        self.bot = ry.BotOp(self.C, real_robot)

        self.initKomo(total_time)

        self.times = 0

        self.blockGraspOrientations = {
            "zx": [ry.FS.scalarProductXY, ry.FS.scalarProductXZ, ry.FS.scalarProductYZ],
            "zy": [ry.FS.scalarProductYY, ry.FS.scalarProductXZ, ry.FS.scalarProductYZ],
            "yx": [ry.FS.scalarProductXY, ry.FS.scalarProductXZ, ry.FS.scalarProductZZ],
            "yz": [ry.FS.scalarProductXX, ry.FS.scalarProductXZ, ry.FS.scalarProductZZ],
            "xy": [ry.FS.scalarProductYY, ry.FS.scalarProductYZ, ry.FS.scalarProductZZ],
            "xz": [ry.FS.scalarProductYX, ry.FS.scalarProductYZ, ry.FS.scalarProductZZ]
        }


    def createNewBlock(self, name: str, position: Union[np.ndarray, list], color: [float]=[1., .5, .0]):
        self.C.addFrame(name) \
            .setPosition(position) \
            .setShape(ry.ST.ssBox, size=[.05, .05, .05, .002]) \
            .setColor(color) \
            .setContact(True) \
            .setMass(1e-2)


    def initKomo(self, phases: int, slicesPerPhase: int=20, enableCollisions: bool=True) -> ry.KOMO:

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


    def moveToPointFreely(self, point: np.ndarray):
        self.initKomo(phases=1)

        self.komo.addObjective([1], ry.FS.position, ["l_gripper"], ry.OT.eq, [1e1], point)


    def moveToPointWithRotation(self):
        pass


    def grabBlock(self, objFrame: str="block", grasp_direction: str='x'):
        """
        """
        for i in ["x", "y"]:
            self.initKomo(phases=1)

            initialGrab_pos = self.C.getFrame(objFrame).getPosition() + np.array([0, 0, .1])
            self.createWaypointFrame("grab_initial", initialGrab_pos)

            self.komo.addObjective([1], ry.FS.positionDiff, ["l_gripper", "grab_initial"], ry.OT.eq, [1e1])
            self.komo.addObjective([2], ry.FS.positionDiff, ["l_gripper", objFrame], ry.OT.eq, [1e1])
            self.komo.addObjective([1, 2], ry.FS.vectorZ, ["l_gripper"], ry.OT.eq, [1e1], [0, 0, 1])
            
            if grasp_direction == 'x':
                self.komo.addObjective([1, 2], ry.FS.scalarProductXY, [objFrame, "l_gripper"], ry.OT.eq, [1e0])
            elif grasp_direction == 'y':
                self.komo.addObjective([1, 2], ry.FS.scalarProductXX, [objFrame, "l_gripper"], ry.OT.eq, [1e0])
            else:
                raise Exception(f'PickDirection "{grasp_direction}" not defined:')
            
            success = self.solveAndExecuteProblem()
            if success: return

        print("Movement was not possible :(")
        

    def placeBlock(self, where: np.ndarray, zRotation: float=np.nan):
        """
        Supposes the robot is currently holding a block
        """
        self.initKomo(phases=1)
        self.komo.addObjective([1], ry.FS.position, ["l_gripper"], ry.OT.eq, [1e1], where)
        self.komo.addObjective([1], ry.FS.vectorZ, ["l_gripper"], ry.OT.eq, [1e1], [0, 0, 1])


    def grabBlockSide(self, objFrame: str="block", grasp_direction: str='xz') -> ry.KOMO():
        """
        grasp a box with a centered top grasp (axes fully aligned)
        """
        self.initKomo(phases=1)

        try:
            align = self.blockGraspOrientations[grasp_direction]
        except:
            raise Exception(f'PickDirection "{grasp_direction}" not defined:')

        # position: centered
        self.komo.addObjective([1], ry.FS.positionDiff, ["l_gripper", objFrame], ry.OT.eq, [1e1])

        # orientation: grasp axis orthogonal to target plane X-specific
        self.komo.addObjective([1], align[0], [objFrame, "l_gripper"], ry.OT.eq, [1e0])
        self.komo.addObjective([1], align[1], [objFrame, "l_gripper"], ry.OT.eq, [1e0])
        self.komo.addObjective([1], align[2], [objFrame, "l_gripper"], ry.OT.eq, [1e0])

    
    def pushBlock(self, objFrame: str="block", pushDirection: str="-x", obj_radious: float=.025):

        obj_pos = self.C.getFrame(objFrame).getPosition()

        if pushDirection == "-x":
            offset_start = np.array([obj_radious*4, 0, 0])
        elif pushDirection == "+x":
            offset_start = np.array([-obj_radious*4, 0, 0])
        elif pushDirection == "-y":
            offset_start = np.array([0, obj_radious*4, 0])
        elif pushDirection == "+y":
            offset_start = np.array([0, -obj_radious*4, 0])
        else:
            raise Exception(f'PushDirection "{pushDirection}" not defined:')
        
        start_pos = obj_pos + offset_start
        end_pos = obj_pos - offset_start

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


    def createWaypointFrame(self, name: str, position: np.ndarray, color: [float]=[0., 1., 0.]) -> ry.Frame:
        way = self.C.getFrame(name)
        if not way:
            way = self.C.addFrame(name) \
                        .setShape(ry.ST.sphere, size=[.01, .002])

        way.setPosition(position).setColor(color)
        return way


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
                print("The motion problem defined is not feasible :(")
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

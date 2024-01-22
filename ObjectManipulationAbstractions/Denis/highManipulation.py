import robotic as ry
import numpy as np
import matplotlib.pyplot as plt


def syncAndExit(C, bot, sync_time=.1, homing = True):
    key = bot.sync(C, .1)
    if chr(key) == "q":
        if(homing==True):
            bot.home(C)
        exit()


class RobotMan():

    def __init__(self, config_file="scenarios/pandaSingle.g", real_robot=False):
        
        self.C = ry.Config()
        self.C.addFile(ry.raiPath(config_file))
        self.C.addFrame('block') \
            .setPosition([-.3, .04, .67]) \
            .setShape(ry.ST.ssBox, size=[.04, .12, .04, .002]) \
            .setColor([1, .5, 0]) \
            .setContact(True) \
            .setMass(1e-2)
        
        self.C.view()
        
        self.bot = ry.BotOp(self.C, real_robot)

        self.orientations = {
            "zx": [ry.FS.scalarProductXY, ry.FS.scalarProductXZ, ry.FS.scalarProductYZ],
            "zy": [ry.FS.scalarProductYY, ry.FS.scalarProductXZ, ry.FS.scalarProductYZ],
            "yx": [ry.FS.scalarProductXY, ry.FS.scalarProductXZ, ry.FS.scalarProductZZ],
            "yz": [ry.FS.scalarProductXX, ry.FS.scalarProductXZ, ry.FS.scalarProductZZ],
            "xy": [ry.FS.scalarProductYY, ry.FS.scalarProductYZ, ry.FS.scalarProductZZ],
            "xz": [ry.FS.scalarProductYX, ry.FS.scalarProductYZ, ry.FS.scalarProductZZ]
        }


    def setupInverseKinematics(self,
                               phases: int,
                               enableCollisions: bool=True,
                               slicesPerPhase: int=20) -> ry.KOMO:
        """
        """
        q_now = self.C.getJointState()

        komo = ry.KOMO()
        komo.setConfig(self.C, enableCollisions)

        komo.setTiming(phases, slicesPerPhase, 1., 2)

        komo.addControlObjective([], 1, 1)
        komo.addControlObjective([], 2, 1)

        komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq)
        if enableCollisions:
            komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq)
        komo.addObjective([], ry.FS.qItself, [], ry.OT.sos, [.1], q_now)

        return komo


    def graspBox(self):
        """
        """
        self.setupInverseKinematics(self.C)
        pass


    def graspTopBox(self, objFrame: str, grasp_direction: str='xz') -> ry.KOMO():
        """
        grasp a box with a centered top grasp (axes fully aligned)
        """
        komo = self.setupInverseKinematics(phases=1)

        try:
            align = self.orientations[grasp_direction]
        except:
            raise Exception('pickDirection not defined:', grasp_direction)

        # position: centered
        komo.addObjective([1], ry.FS.positionDiff, ["l_gripper", objFrame], ry.OT.eq, [1e1])

        # orientation: grasp axis orthogonal to target plane X-specific
        komo.addObjective([1], align[0], [objFrame, "l_gripper"], ry.OT.eq, [1e0])
        komo.addObjective([1], align[1], [objFrame, "l_gripper"], ry.OT.eq, [1e0])
        komo.addObjective([1], align[2], [objFrame, "l_gripper"], ry.OT.eq, [1e0])

        return komo


    def grabBlock(self, C, bot, block):
        qHome = C.getJointState()

        komo = ry.KOMO(C, 1, 1, 0, False)
        komo.addObjective(
            times=[], 
            feature=ry.FS.jointState, 
            frames=[],
            type=ry.OT.sos, 
            scale=[1e-1], 
            target=qHome
        )
        komo.addObjective([], ry.FS.positionDiff, ['l_gripper', block], ry.OT.eq, [1e1])

        ret = ry.NLP_Solver(komo.nlp(), verbose=4) .solve()
        print(ret)

        q = komo.getPath()
        del komo #also closes komo view

        C.setJointState(q[0])

        komo = self.setupInverseKinematics(C, 1)

        komo.addObjective([], ry.FS.positionDiff, ['l_gripper', block], ry.OT.eq, [1e1])
        komo.addObjective([], ry.FS.scalarProductXY, ['l_gripper', block], ry.OT.eq, [1e1], [0])
        komo.addObjective([], ry.FS.scalarProductXZ, ['l_gripper', block], ry.OT.eq, [1e1], [0])
        komo.addObjective([], ry.FS.distance, ['l_palm', block], ry.OT.ineq, [1e1])

        return komo

    def endeffectorToPoint(self):
        """
        """
        pass
    

    def solveKomo(self, komo):
        ret = ry.NLP_Solver(komo.nlp(), verbose=0 ) .solve()
        print(ret)
        if ret.feasible:
            print('--- FEASIBLE ---')
        else:
            print('---  !!!!!!!!!!! NOT FEASIBLE !!!!!!!!!!! ---')

        q = komo.getPath()

        return q
    
    def executeMotion(self, komo: ry.KOMO):
        q = self.solveKomo(komo)
        self.bot.move(q, [3.5])
        while self.bot.getTimeToEnd() > 0:
            self.bot.sync(self.C)

        q = komo.getPath()
        plt.plot(q)
        plt.show()


class Debug():
    """
    """
    def __init__(self, C):
        self.C = C


    def plotPath(komo: ry.KOMO()) -> None:
        q = komo.getPath()
        plt.plot(q)
        plt.show()

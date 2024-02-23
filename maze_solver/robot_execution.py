import time
import numpy as np
import robotic as ry
from typing import Tuple


class Robot():
    def __init__(self, C: ry.Config, on_real: bool=False):
        self.bot = ry.BotOp(C, on_real)
        self.bot.home(C)

        self.bot.gripperMove(ry._left, .078, .4)
        while not self.bot.gripperDone(ry._left):
            self.bot.sync(C, .1)


    def execute_path_blocking(self,
                              C: ry.Config,
                              path: np.ndarray,
                              time_to_solve: float=3.):
        
        self.bot.move(path, [time_to_solve]) # moveAutoTimed prints out two numbers, idk why
        while self.bot.getTimeToEnd() > 0:
            self.bot.sync(C, .1)


    def execute_path_and_measure_forces(self,
                                        C: ry.Config,
                                        path: np.ndarray,
                                        max_velocity: float=1.,
                                        max_force_allowed: float=np.nan) -> Tuple[list, list]:
        max_force = -np.inf
        joint_states = []
        external_taus = []
        
        self.bot.moveAutoTimed(path, max_velocity)

        tic_time = time.monotonic()
        while self.bot.getTimeToEnd() > 0:

            tic_time += .1
            now_time = time.monotonic()
            if tic_time > now_time:
                time.sleep(tic_time - now_time)
            
            self.bot.sync(C, .005)

            # Measure force on the direction of the Y vector of the gripper
            y, J = C.eval(ry.FS.position, ['l_gripper'], [[0, 1, 0]])
            J = np.linalg.pinv(J.T)
            tauExternal = self.bot.get_tauExternal()
            F = np.abs(J @ tauExternal)
            
            # Store measuremets and positions
            joint_states.append(self.bot.get_q().tolist())
            external_taus.append(tauExternal.tolist())

            # Check if max force is exceeded
            max_force = F if F > max_force else max_force
            if max_force > max_force_allowed:
                print("Max force exceeded!")
                break
        
        return external_taus, joint_states
    
    def grasp(self, C: ry.Config):
        self.bot.gripperClose(ry._left)
        while not self.bot.gripperDone(ry._left):
            self.bot.sync(C)

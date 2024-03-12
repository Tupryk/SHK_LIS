import time
import numpy as np
import open3d as o3d
import robotic as ry
from typing import Tuple
from utils import estimate_cube_pose, extract_position_and_quaternion


class Robot():
    def __init__(self, C: ry.Config, on_real: bool=False):
        self.bot = ry.BotOp(C, on_real)
        self.bot.home(C)

        self.bot.gripperMove(ry._left, .078, .4)
        while not self.bot.gripperDone(ry._left):
            self.bot.sync(C, .1)


    def goHome(self, C: ry.Config):
        self.bot.home(C)


    def execute_path_blocking(self,
                              C: ry.Config,
                              path: np.ndarray,
                              time_to_solve: float=3.):
        
        self.bot.move(path, [time_to_solve])
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
            
            self.bot.sync(C, .0)

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
            self.bot.sync(C, .1)

    def release(self, C: ry.Config):
        self.bot.gripperMove(ry._left, .078, .4)
        while not self.bot.gripperDone(ry._left):
            self.bot.sync(C, .1)

    def get_sigle_box_pos_qurn(self, C, box_dims: np.ndarray, midpoint: np.ndarray, radius: float, z_cutoff: float=.67) -> Tuple[np.ndarray, np.ndarray]:
        _, _, points = self.bot.getImageDepthPcl('cameraWrist', False)

        new_p = []
        for lines in points:
            for p in lines:
                if np.linalg.norm(p) != 0:
                    new_p.append(p.tolist())
        points = np.array(new_p)
        
        cameraFrame = C.getFrame("cameraWrist")

        R, t = cameraFrame.getRotationMatrix(), cameraFrame.getPosition()

        points = points @ R.T
        points = points + np.tile(t.T, (points.shape[0], 1))

        objectpoints=[]
        for p in points:
            if p[2] > (z_cutoff) and np.linalg.norm(p-midpoint) <= radius:
                objectpoints.append(p)
        objectpoints = np.array(objectpoints)
        if len(objectpoints) == 0:
            print("Lost the object!")
            exit()

        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(objectpoints)
        pc, _ = pc.remove_radius_outlier(nb_points=20, radius=.02)

        # Update estimated object frame
        pose_mat = estimate_cube_pose(pc, box_dims[:3], add_noise=True, origin=np.array([0, 0, 0]), verbose=0)
        position, quaternion = extract_position_and_quaternion(pose_mat)
        return position, quaternion

import numpy as np
from visual import mstorePCR
from arena import *
import open3d as o3d

INITIAL_OBJ_POS = [-.5, 0, .69]
DEBUG = False
OBJ_HEIGHT = .08

ON_REAL = True

robot_pos = np.array([-.54, -.17, .651])

if __name__ == "__main__":


    data = []
    obj_pos = INITIAL_OBJ_POS


    print("Splitting and saving completed.")

    pcd_files = [o3d.io.read_point_cloud(f'data/point_cloud_{i}.pcd') for i in range(8)]

    final_points = mstorePCR(pcd_files, True, verbose=1)

    
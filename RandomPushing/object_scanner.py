import robotic as ry
import numpy as np
from config import setup_config, startup_robot
from visual import getObject, lookAtObj, mstorePCR, point2pointPCR_normal, scanObject, point2pointPCR
from arena import *
import json
import open3d as o3d

INITIAL_OBJ_POS = [-.5, 0, .69]
DEBUG = False
OBJ_HEIGHT = .08

ON_REAL = True

robot_pos = np.array([-.54, -.17, .651])

if __name__ == "__main__":

    #-- load parameters, typically automatically from 'rai.cfg'
    ry.params_print()
    verbose = 0

    #-- define a configuration
    C = setup_config(INITIAL_OBJ_POS, ON_REAL)
    if DEBUG:
        key = C.view(verbose>0, 'happy with the config?')
        print('key pressed: ', chr(key)) #use this for basic interaction, e.g. aborting the program
        if chr(key)=='q':
            exit()

    bot = startup_robot(C, ON_REAL)

    data = []
    obj_pos = INITIAL_OBJ_POS

    # Generate Arena
    arena = CircularArena(middleP=robot_pos, innerR=None, outerR=.35)
    arena.plotArena(C)

    # Point towards set initial object position
    lookAtObj(bot, C, np.array(obj_pos))

    # Capture midpoint from point cloud
    obj_pos  = getObject(bot, C, arena) 

    scanned_views = scanObject(bot, C, np.array(obj_pos), arena, save_as="./data/scanned_views.json", view_count=8)

    for i, point_cloud in enumerate(scanned_views):

        # Save each point cloud as a separate JSON file without additional brackets
        output_filename = f'data/point_cloud_{i}.json'
        with open(output_filename, 'w') as output_file:
            json.dump(point_cloud, output_file)

        print(f'Saved {output_filename}')

        # Create a PointCloud object
        point_cloud_obj = o3d.geometry.PointCloud()

        # Set the points in the PointCloud
        point_cloud_obj.points = o3d.utility.Vector3dVector(point_cloud)

        # Save the PointCloud as a PCD file in the "data" subdirectory
        output_pcd_file = f'data/point_cloud_{i}.pcd'
        o3d.io.write_point_cloud(output_pcd_file, point_cloud_obj)

    print("Splitting and saving completed.")

    pcd_files = [o3d.io.read_point_cloud(f'data/point_cloud_{i}.pcd') for i in range(len(scanned_views))]

    final_points = mstorePCR(pcd_files, False, verbose=1)

    pclFrame = C.getFrame("pcl")
    #pclFrame.setPointCloud(np.array(final_points))
    #C.view_recopyMeshes()
    
    bot.hold()
    bot.home(C)

    C.view(True)
    del bot
    del C
    exit()
    
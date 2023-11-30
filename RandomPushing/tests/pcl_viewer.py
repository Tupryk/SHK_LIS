from robotic import ry
import open3d as o3d
import numpy as np

def pcl_viewer(pclfile=None, point=None):
    if(pclfile):
        point_cloud = o3d.io.read_point_cloud(pclfile)
        points = np.asarray(point_cloud.points)
    else:
        pass 
    
    C = ry.Config()
    C.addFile(ry.raiPath('../rai-robotModels/scenarios/pandaSingle.g'))

    bot = ry.BotOp(C, False)
    bot.home(C)

    pclFrame = C.getFrame("pcl")
    pclFrame.setPointCloud(np.array(final_points))
    C.view_recopyMeshes()

if __name__ == "__main__":
    homing(.9)

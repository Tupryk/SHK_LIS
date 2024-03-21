import robotic as ry
import numpy as np
import matplotlib.pyplot as plt
import cv2 
import open3d as o3d
from sklearn.neighbors import NearestNeighbors

ON_REAL = False

def compute_normals(point_cloud, radius):
    """
    Compute normals for each point in the point cloud.
    
    Args:
        point_cloud (numpy.ndarray): Nx3 array representing the point cloud.
        radius (float): Radius for nearest neighbors search.
        
    Returns:
        numpy.ndarray: Nx3 array representing the normals for each point.
    """
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(point_cloud)
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=30))
    normals = np.asarray(pcd.normals)
    return normals

def find_interesting_viewpoints(point_cloud, normals, angle_threshold, num_neighbors=40):
    """
    Find interesting viewpoints based on disalignment of normals.
    
    Args:
        point_cloud (numpy.ndarray): Nx3 array representing the point cloud.
        normals (numpy.ndarray): Nx3 array representing the normals for each point.
        angle_threshold (float): Threshold for angle difference between neighboring normals (in radians).
        num_neighbors (int): Number of nearest neighbors to consider.
        
    Returns:
        list: List of indices of interesting viewpoints.
    """
    # Build a k-d tree on the point cloud
    tree = NearestNeighbors(n_neighbors=num_neighbors+1).fit(point_cloud)
    
    num_points = point_cloud.shape[0]
    interesting_indices = []
    for i in range(num_points):
        # Query the k-d tree to find nearest neighbors (excluding the point itself)
        _, neighbor_indices = tree.kneighbors([point_cloud[i]], n_neighbors=num_neighbors+1)
        neighbor_indices = neighbor_indices[0][1:]  # Exclude the point itself
        
        # Compute dot products of normals with neighbor normals
        dot_products = np.sum(normals[i] * normals[neighbor_indices], axis=1)
        
        # Compute angles between normals
        cos_angles = dot_products / (np.linalg.norm(normals[i]) * np.linalg.norm(normals[neighbor_indices], axis=1))
        angles = np.arccos(np.clip(cos_angles, -1, 1))
        
        # Check if any angle exceeds the threshold
        if np.any(angles > angle_threshold):
            interesting_indices.append(i)
    return interesting_indices

def visualize_point_cloud_with_interesting_points(point_cloud, interesting_points):
    """
    Visualize the point cloud with interesting viewpoints using Open3D.
    
    Args:
        point_cloud (numpy.ndarray): Nx3 array representing the point cloud.
        interesting_points (list): List of interesting viewpoints.
    """
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(point_cloud)
    
    # Colorize the interesting points to distinguish them
    colors = np.zeros((len(point_cloud), 3))
    colors[:len(interesting_points)] = [1, 0, 0]  # Color interesting points red
    
    pcd.colors = o3d.utility.Vector3dVector(colors)
    
    o3d.visualization.draw_geometries([pcd])

def main():

    C = ry.Config()
    C.addFile(ry.raiPath('scenarios/pandaSingle_.g'))       # pandaSingle with added giraffe in my scenarios

    C.addFrame('predicted_obj') \
        .setShape(ry.ST.marker, size=[.1]) \
        .setPosition([0, .25, .7]) \
        .setColor([1, 0, 0])

    bot = ry.BotOp(C, ON_REAL)
    bot.home(C)

    bot.gripperMove(ry._left, .01, .4)
    while not bot.gripperDone(ry._left):
        bot.sync(C, .1)
    
    C.view(True)



    q_now = C.getJointState()
    q_home = bot.get_qHome()

    komo = ry.KOMO()
    komo.setConfig(C, True)
    komo.setTiming(1., 1, 1., 0)
    komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq)
    komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq)

    komo.addObjective([1.], ry.FS.positionRel, ["predicted_obj", "cameraWrist"], ry.OT.eq, [1.], [.0, .0, .2])
    komo.addObjective([1.], ry.FS.position, ["l_gripper"], ry.OT.ineq, np.array([[.0, .0, -100.]]), [0, 0, 1])
    komo.addObjective([], ry.FS.qItself, [], ry.OT.sos, [.1], q_home)
    komo.addObjective([], ry.FS.qItself, [], ry.OT.sos, [.1], q_now)

    ret = ry.NLP_Solver() \
        .setProblem(komo.nlp()) \
        .setOptions(stopTolerance=1e-2, verbose=0) \
        .solve()
    
    print(ret)
        
    bot.moveTo(komo.getPath()[0], 1., False)
    while bot.getTimeToEnd() > 0:
        key=bot.sync(C, .1)
        if chr(key) == "q":
            print("Terminated (visual)")
            bot.home(C)
            del bot
            del C
            exit()


    rgb, depth, points = bot.getImageDepthPcl('cameraWrist', False)
    fig = plt.figure(figsize=(10,5))
    axs = fig.subplots(1, 2)
    axs[0].imshow(rgb)
    axs[1].matshow(depth)
    plt.show()

    cameraFrame = C.getFrame("cameraWrist")
    R, t = cameraFrame.getRotationMatrix(), cameraFrame.getPosition()

    points = points.reshape(points.shape[0]*points.shape[1],points.shape[2])
    points = points @ R.T
    points = points + np.tile(t.T, (points.shape[0], 1))
    filtered_points = points[points[:,2] >= .67]
    

    #filtered_points = filtered_points.reshape(filtered_points.shape[0]*filtered_points.shape[1], filtered_points.shape[2])
    pcd= o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(filtered_points)
    pcd.normals = o3d.utility.Vector3dVector(compute_normals(filtered_points, radius=0.1))
    o3d.visualization.draw_geometries([pcd])
    angle_threshold = np.radians(60)  # For example, 30 degrees
    
    # Find interesting viewpoints
    interesting_viewpoints = find_interesting_viewpoints(np.asarray(pcd.points), np.asarray(pcd.normals), angle_threshold)
    print("Number of interesting viewpoints:", len(interesting_viewpoints))
    
    # Visualize the point cloud with interesting viewpoints
    visualize_point_cloud_with_interesting_points(np.asarray(pcd.points), interesting_viewpoints)
    # --- calculate color gradient
    # # Separate channels
    # red_channel = rgb[:,:,0]
    # green_channel = rgb[:,:,1]
    # blue_channel = rgb[:,:,2]

    # # Apply Sobel filter to each channel (adjust kernel size if needed)
    # sobel_red = cv2.Sobel(red_channel, cv2.CV_64F, 1, 0, ksize=3)  # Apply in x-direction
    # sobel_green = cv2.Sobel(green_channel, cv2.CV_64F, 1, 0, ksize=3)
    # sobel_blue = cv2.Sobel(blue_channel, cv2.CV_64F, 1, 0, ksize=3)

    # # Combine channels (optional, see alternative approach below)
    # magnitude_red, _ = cv2.cartToPolar(sobel_red, np.zeros_like(sobel_red))
    # magnitude_green, _ = cv2.cartToPolar(sobel_green, np.zeros_like(sobel_green))
    # magnitude_blue, _ = cv2.cartToPolar(sobel_blue, np.zeros_like(sobel_blue))
    # combined_gradient = (magnitude_red + magnitude_green + magnitude_blue) / 3  # Simple average

    # combined_gradient_norm = (combined_gradient - np.min(combined_gradient)) / (np.max(combined_gradient) - np.min(combined_gradient))
    # plt.imshow(combined_gradient_norm, cmap="gray")  # Use grayscale colorspace
    # plt.title("Combined Color Gradient")
    # plt.colorbar()  # Add a colorbar to visualize intensity levels
    # plt.show()

if __name__ == "__main__":
    main()
import numpy as np
import robotic as ry
from typing import Tuple, List
from arenas import Arena
"""
This code needs cleaning
"""

def getFilteredPointCloud(bot: ry.BotOp,
                          C: ry.Config,
                          arena: Arena,
                          z_cutoff: float=.68) -> Tuple[List[float], List[float]]:
    bot.sync(C, .0)
    rgb, depth, points = bot.getImageDepthPcl('cameraWrist', False)

    new_rgb = []
    for lines in rgb:
        for c in lines: 
            new_rgb.append(c.tolist())
    rgb = new_rgb

    new_p = []
    for lines in points:
        for p in lines: 
            new_p.append(p.tolist())
    points = new_p

    new_rgb = []
    new_p = []
    for i, p in enumerate(points):
        if sum(p) != 0:
            new_rgb.append(rgb[i])
            new_p.append(p)
    points = np.array(new_p)
    
    cameraFrame = C.getFrame("cameraWrist")

    R, t = cameraFrame.getRotationMatrix(), cameraFrame.getPosition()

    points = points @ R.T
    points = points + np.tile(t.T, (points.shape[0], 1))

    objectpoints=[]
    colors = []
    for i, p in enumerate(points):
        if p[2] > (z_cutoff) and arena.point_in_arena(np.array(p)):
            objectpoints.append(p)
            colors.append(rgb[i])

    return objectpoints, colors


def getPointsMinMaxCoors(points: [[float]]) -> Tuple[np.ndarray, np.ndarray]:

    min_coor = np.array([
        min([p[0] for p in points]),
        min([p[1] for p in points]),
        min([p[2] for p in points])
    ])

    max_coor = np.array([
        max([p[0] for p in points]),
        max([p[1] for p in points]),
        max([p[2] for p in points])
    ])

    return min_coor, max_coor


def getScannedObject(bot: ry.BotOp,
                     C: ry.Config,
                     arena: Arena,
                     visuals: bool=True) -> Tuple[np.ndarray, np.ndarray]:

    points, _ = getFilteredPointCloud(bot, C, arena)
    points = np.array(points)

    if not len(points):
        print("ERROR: Object not found!")
        return np.array([]), np.array([])

    min_coor, max_coor = getPointsMinMaxCoors(points)

    midpoint = (max_coor+min_coor)/2
    
    if visuals:
        pclFrame = C.getFrame("pcl")
        if not pclFrame:
            pclFrame = C.addFrame('pcl')
            pclFrame.setPointCloud(np.array(points))
            pclFrame.setColor([0.,1.,0.]) #only to see it when overlaying with truth
            C.view_recopyMeshes()

            C.addFrame('mid_point') \
                .setPosition(midpoint) \
                .setShape(ry.ST.marker, size=[.2]) \
                .setColor([1, 1, 0])
        else:
            pclFrame.setPointCloud(np.array(points))
            C.view_recopyMeshes()
            C.getFrame('mid_point') \
                .setPosition(midpoint)
    
    return midpoint, points
import random
import numpy as np

from RobotEnviroment.init import *
from RobotEnviroment.arenas import RectangularArena
from PointClouds.InteractionPointFinder import getPushPoints
from PointClouds.Registration import *
from PathPlanning.scanning import *
from PathPlanning.motion import *
from utils.debug import *


ON_REAL = False
ROBOT_VELOCITY = 1. # rad/s
INITIAL_OBJ_POS = np.array([-.50, -.1, .69])

C = setup_config(INITIAL_OBJ_POS, ON_REAL)
bot = startup_robot(C, ON_REAL)

TABLE_CENTER = np.array([-.23, -.16, .651])
TABLE_DIMS = np.array([.89, .55])
ROBOT_POS = np.array([-.03, -.24, .651])
ROBOT_RING = .29

pushArena = RectangularArena(middleP=TABLE_CENTER, width=TABLE_DIMS[0], height=TABLE_DIMS[1], middlePCirc=ROBOT_POS, innerR=ROBOT_RING, name="pushArena")
pushArena.plotArena(C, color=[.5, .5, .5])

scanArena = RectangularArena(middleP=TABLE_CENTER, width=TABLE_DIMS[0]+.2, height=TABLE_DIMS[1]+.2, middlePCirc=ROBOT_POS, innerR=ROBOT_RING*.5, name="scanArena")
scanArena.plotArena(C, color=[1., 1., 1.])

NUMBER_OF_PUSH_TRIALS = 100
predObjPos = INITIAL_OBJ_POS

pointClouds = [] # Stores the world position of the point cloud and the point cloud {"world_position": [x, y, z], "pc": np.array([])}
minNumScans = 2 # Minimum nuber of point cloud scans until we start merging them
fullPC = np.array([])
maxForces = []

NUMBER_OF_PUSH_TRIALS = 10
predObjPos = INITIAL_OBJ_POS

pointClouds = [] # Stores the world position of the point cloud and the point cloud {"world_position": [x, y, z], "pc": np.array([])}
minNumScans = 2 # Minimum nuber of point cloud scans until we start merging them
fullPC = np.array([])
maxForces = []

for i in range(NUMBER_OF_PUSH_TRIALS):

    print("Starting Trial Number ", i+1)

    # Scan object at it's predicted position and get it's possible push points
    print("Scanning object...")

    lookSuccess = False
    while not lookSuccess:
        lookSuccess = lookAtObjVectorField(predObjPos, bot, C, velocity=ROBOT_VELOCITY, verbose=0)
        
    predObjPos, pointCloud = getScannedObject(bot, C, scanArena)
    if not len(predObjPos):
        print ("Lost the Object!")
        break
    push_points  = getPushPoints(pointCloud, verbose=0)

    # Store the object's point cloud
    print("Storing point cloud")
    for j, _ in enumerate(pointCloud):
        pointCloud[j] -= predObjPos
    pointClouds.append(pointCloud)

    if len(pointClouds) >= minNumScans:
        fullPC = joinOffsetPCS(pointClouds.copy(), verbose=0)

    # Try to push object
    for _ in range(5): # This counts the number of attempts to push an object from the current view angle. If it reaches 5 we try a new angle
        if not len(push_points):
            print("No viable push points found!")
            break

        pushP = random.choice(push_points)
        # push_points.remove(pushP)

        print("Trying to push obj...")
        waypoints = pushMotionWaypoints(pushP[0], pushP[1], predObjPos, config=C)
        if not pushArena.point_in_arena(waypoints[-1]): # Should automatically generate waypoints that are not outside the push arena but for now we do this
            print("Generated waypoints outside of arena!")
            continue

        success, maxForce = doPushThroughWaypoints(bot, C, waypoints, velocity=ROBOT_VELOCITY, verbose=0)
        if success:
            print("Success! :)")
            predObjPos = waypoints[-1] # This should be offset by the width of the object!
            maxForces.append(maxForce)
            break
        print("Failed! :(")
    
bot.home(C)
C.view(True)

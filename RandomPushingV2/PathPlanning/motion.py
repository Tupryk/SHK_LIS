import random
import numpy as np
import robotic as ry
from utils.geometry import pathLength
from RobotEnviroment.robotMovement import moveLocking


def giveRandomAllowedAngle(allowedSegments: [[float]]) -> float:

    # This could be made better I think
    total_len = 0
    for seg in allowedSegments:
        total_len += seg[1]-seg[0]
    nonAdjustedAngle = np.random.random()*total_len

    nonAdjustedAngle += allowedSegments[0][0]
    for i, seg in enumerate(allowedSegments[:-1]):
        if nonAdjustedAngle >= seg[0] and nonAdjustedAngle <= seg[1]:
            return nonAdjustedAngle
        nonAdjustedAngle += allowedSegments[i+1][0]-seg[1]
    
    return nonAdjustedAngle


def pushMotionWaypoints(point: np.ndarray,
                        normal: np.ndarray,
                        robot_pos: np.ndarray,
                        start_dist: float=.15,
                        end_dist_range: [float]=[.1, .15],
                        initial_elevation: float=.15,
                        config: ry.Config=None) -> [np.ndarray]:

    robot2point = point-robot_pos
    dir = -1. if np.inner(robot2point, normal) < 0 else 1.
    start_pos = normal * start_dist * dir
    end_dist = np.random.rand() * (end_dist_range[1]-end_dist_range[0]) + end_dist_range[0]
    end_pos = normal * end_dist * dir
    initial = point.copy()+start_pos.copy()
    initial[2] += initial_elevation
    waypoints = [initial, point+start_pos, point, point-end_pos]
    
    if config:
        for i, w in enumerate(waypoints):
            way = config.getFrame(f"way{i}")

            if not way:
                way = config.addFrame(f"way{i}") \
                .setShape(ry.ST.marker, size=[.1])
                if i == 0:
                    way.setColor([1, 0, 0])
                elif i == len(waypoints)-1:
                    way.setColor([0, 0, 1])
                else:
                    way.setColor([0, 1, 0])

            way.setPosition(w)

    return waypoints


def moveToInitialPushPoint(bot: ry.BotOp,
                           C: ry.Config,
                           initialPoint: np.ndarray,
                           direction: np.ndarray,
                           speed: float=.2,
                           verbose: int=0) -> bool:
    q_now = C.getJointState()

    komo = ry.KOMO()
    komo.setConfig(C, True)

    komo.setTiming(1, 2, 1., 2)

    komo.addControlObjective([], 0, 1e-2)
    komo.addControlObjective([], 2, 1e1)

    komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq)
    komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq)
    komo.addObjective([], ry.FS.qItself, [], ry.OT.sos, [.1], q_now)

    komo.addObjective([1.], ry.FS.position, ['l_gripper'], ry.OT.eq, [1e1], initialPoint)
    komo.addObjective([1.], ry.FS.vectorZ, ['l_gripper'], ry.OT.eq, [1e1], -direction)
    komo.addObjective([1.], ry.FS.scalarProductXZ, ['l_gripper', 'table'], ry.OT.eq, [1e1], [0.])
    komo.addObjective([1.], ry.FS.scalarProductYZ, ['l_gripper', 'table'], ry.OT.ineq, [-1e1], [0.])

    dist_to_travel = np.linalg.norm(initialPoint-C.getFrame("l_gripper").getPosition())
    timeToTravel = dist_to_travel/speed
    return moveLocking(bot, C, komo, timeToTravel, verbose=verbose)


def moveThroughPushPath(bot: ry.BotOp,
                        C: ry.Config,
                        waypoints: [np.ndarray],
                        direction: np.ndarray,
                        speed: float=.2,
                        verbose: int=0) -> bool:

    komo = ry.KOMO()
    komo.setConfig(C, True)

    # We assume that the gripper is already at the starting waypoint and with the correct rotation
    komo.setTiming(len(waypoints)-1, 5, 1., 2)

    komo.addControlObjective([], 0, 1e-2)
    komo.addControlObjective([], 2, 1e1)

    komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq)
    komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq)

    komo.addObjective([], ry.FS.vectorZ, ['l_gripper'], ry.OT.eq, [1e1], -direction)
    komo.addObjective([], ry.FS.scalarProductXZ, ['l_gripper', 'table'], ry.OT.eq, [1e1], [0.])
    komo.addObjective([], ry.FS.scalarProductYZ, ['l_gripper', 'table'], ry.OT.ineq, [-1e1], [0.])

    for i, way in enumerate(waypoints[1:]):
        komo.addObjective([i+1.], ry.FS.position, ['l_gripper'], ry.OT.eq, [1e1], way)
    
    dist_to_travel = pathLength(waypoints)
    timeToTravel = dist_to_travel/speed
    return moveLocking(bot, C, komo, timeToTravel, verbose=verbose)


def moveBackAfterPush(bot: ry.BotOp,
                      C: ry.Config,
                      waypoints: [np.ndarray],
                      direction: np.ndarray,
                      speed: float=.2,
                      verbose: int=0) -> bool:
    
    komo = ry.KOMO()
    komo.setConfig(C, True)

    komo.setTiming(len(waypoints)-1, 2, 1., 2)

    komo.addControlObjective([], 0, 1e-2)
    komo.addControlObjective([], 2, 1e1)

    komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq)
    komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq)

    # We assume that the gripper is already at the starting waypoint and with the correct rotation
    komo.addObjective([], ry.FS.vectorZ, ['l_gripper'], ry.OT.eq, [1e1], -direction)
    komo.addObjective([], ry.FS.scalarProductXZ, ['l_gripper', 'table'], ry.OT.eq, [1e1], [0.])
    komo.addObjective([], ry.FS.scalarProductYZ, ['l_gripper', 'table'], ry.OT.ineq, [-1e1], [0.])

    for i in range(len(waypoints)-1):
        index = len(waypoints)-2 - i
        komo.addObjective([i+1], ry.FS.position, ['l_gripper'], ry.OT.eq, [1e1], waypoints[index]) 

    dist_to_travel = pathLength(waypoints)
    timeToTravel = dist_to_travel/speed
    return moveLocking(bot, C, komo, timeToTravel, verbose=verbose)


def doPushThroughWaypoints(C: ry.Config,
                           bot: ry.BotOp,
                           ways: [np.ndarray],
                           verbose: int=0,
                           speed: float=.2) -> bool:

    move_dir = ways[-1]-ways[1]
    pathLen = np.linalg.norm(move_dir) # Could use this to calculate the time the robot has to move for uniform velocities.
    move_dir /= pathLen

    ### GO TO INITIAL PUSH POINT ###
    success = moveToInitialPushPoint(bot, C,
                                     ways[0], move_dir,
                                     speed=speed, verbose=verbose)
    if not success:
        return False
    
    ### EXECUTE PUSH ###
    success = moveThroughPushPath(bot, C,
                                  ways, move_dir,
                                  speed=speed, verbose=verbose)
    if not success:
        return False
    
    ### MOVE BACK ###
    # This is done so that the object is not disturbed after the push action
    success = moveBackAfterPush(bot, C,
                                  ways, move_dir,
                                  speed=speed, verbose=verbose)
    if not success:
        return False
    
    return True

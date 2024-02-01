import numpy as np
import robotic as ry
from typing import Tuple
from utils.raiUtils import komoStraightPath, createWaypointFrame
from RobotEnviroment.robotMovement import moveBlocking, moveBlockingAndCheckForce

STANDARD_VELOCITY = .2


def standardKomo(C: ry.Config, phases: int, slicesPerPhase: int=20, enableCollisions: bool=True) -> ry.KOMO:

    q_now = C.getJointState()

    komo = ry.KOMO()
    komo.setConfig(C, enableCollisions)

    komo.setTiming(phases, slicesPerPhase, 1., 2)

    komo.addControlObjective([], 1, 1e-1)
    komo.addControlObjective([], 2, 1e1)

    komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq)
    if enableCollisions:
        komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq)
    komo.addObjective([], ry.FS.qItself, [], ry.OT.sos, [.1], q_now)

    komo.addObjective([phases], ry.FS.qItself, [], ry.OT.eq, [10.], [], 1)

    return komo


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
                        end_dist_range: [float]=[.01, .05],
                        initial_elevation: float=.15,
                        config: ry.Config=None,
                        minHeight: float=.7) -> [np.ndarray]:

    robot2point = point-robot_pos
    dir = -1. if np.inner(robot2point, normal) < 0 else 1.
    start_pos = normal * start_dist * dir
    end_dist = np.random.rand() * (end_dist_range[1]-end_dist_range[0]) + end_dist_range[0]
    end_pos = normal * end_dist * dir
    initial = point.copy()+start_pos.copy()
    initial[2] += initial_elevation
    waypoints = [initial, point+start_pos, point-end_pos]
    if waypoints[-1][2] < minHeight:
        waypoints[-1][2] = minHeight
    
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


def doPushThroughWaypoints(bot: ry.BotOp,
                           C: ry.Config,
                           waypoints: [np.ndarray],
                           velocity: float=STANDARD_VELOCITY,
                           verbose: int=0) -> Tuple[bool, float]:

    createWaypointFrame(C, "start_push", waypoints[1])
    createWaypointFrame(C, "end_push", waypoints[2])

    # Define the komo problem
    komo = standardKomo(C, 5, enableCollisions=False)

    y_vector_dir = waypoints[2] - waypoints[1]
    y_vector_dir /= np.linalg.norm(y_vector_dir)
    komo.addObjective([1., 4], ry.FS.vectorY, ['l_gripper'], ry.OT.eq, [1e1], y_vector_dir)
    komo.addObjective([1., 4], ry.FS.vectorZ, ['l_gripper'], ry.OT.eq, [1e1], [0, 0, 1])
    komo.addObjective([1., 4], ry.FS.scalarProductXZ, ['l_gripper', 'table'], ry.OT.eq, [1e1], [0.])
    komo.addObjective([1., 4], ry.FS.scalarProductYZ, ['l_gripper', 'table'], ry.OT.ineq, [-1e1], [0.])

    komo.addObjective([2], ry.FS.position, ['l_gripper'], ry.OT.eq, [1e1], waypoints[0])

    komo = komoStraightPath(C, komo, ["start_push", "end_push"], phases=[3, 4])

    # Execute motion if possible
    success, maxForce = moveBlockingAndCheckForce(bot, C, komo, velocity, verbose=verbose)
    return success, maxForce


def moveBack(bot: ry.BotOp, C: ry.Config, how_much: float=.1, dir: str="y", velocity: float=STANDARD_VELOCITY, verbose: int=0) -> bool:
    """
    Move in the direction of the y vector of the gripper by "how_much" in a straight line
    """

    # Create waypoint frames
    initial_gripper_pos = C.getFrame("l_gripper").getPosition()
    initial_gripper_rot = C.getFrame("l_gripper").getQuaternion()
    start_frame = C.addFrame("retreat_start_pos") \
        .setShape(ry.ST.sphere, size=[.01, .01]) \
        .setQuaternion(initial_gripper_rot) \
        .setPosition(initial_gripper_pos)

    end_frame = C.addFrame("retreat_end_pos", "retreat_start_pos") \
        .setShape(ry.ST.sphere, size=[.01, .01]) \
        .setColor([1, 1, 0])
    
    # Set movement direction
    if dir == "y":
        end_frame_rel_pos = np.array([.0, -how_much, .0])
    elif dir == "z":
        end_frame_rel_pos = np.array([.0, .0, how_much])
    else:
        raise Exception(f"Value '{dir}' is not a valid retreat direction!")
    
    end_frame.setRelativePosition(end_frame_rel_pos)
    
    # Define komo
    komo = standardKomo(C, 1, enableCollisions=False)
    komo = komoStraightPath(C, komo, ["retreat_start_pos", "retreat_end_pos"], phases=[0, 1])

    # Move
    success = moveBlocking(bot, C, komo, velocity, verbose=verbose)
    return success


def pokePoint(bot: ry.BotOp,
              C: ry.Config,
              point: np.ndarray,
              velocity: float=.1,
              maxPush: float=.02,
              verbose: int=0) -> Tuple[bool, float]:

    waypoints = [
        point + np.array([.0, .0, .1]),
        point + np.array([.0, .0, maxPush]),
        point - np.array([.0, .0, maxPush]),
    ]

    # Go to starting position
    komo = standardKomo(C, 1)

    komo.addObjective([1.], ry.FS.vectorZ, ['l_gripper'], ry.OT.eq, [1e1], [0., 0., 1.])
    komo.addObjective([1.], ry.FS.position, ['l_gripper'], ry.OT.eq, [1e1], waypoints[0])

    success = moveBlocking(bot, C, komo, velocity, verbose=verbose)
    if not success:
        print("Failed getting to initial poking position!")
        return False, .0

    # Poke
    komo = standardKomo(C, 2)

    komo.addObjective([], ry.FS.vectorZ, ['l_gripper'], ry.OT.eq, [1e1], [0., 0., 1.])
    komo.addObjective([1.], ry.FS.position, ['l_gripper'], ry.OT.eq, [1e1], waypoints[1])
    komo.addObjective([2.], ry.FS.position, ['l_gripper'], ry.OT.eq, [1e1], waypoints[2])

    success, resultingForce = moveBlockingAndCheckForce(bot, C, komo, velocity, maxForceAllowed=1, verbose=verbose)
    if not success:
        print("Failed poking object!")
        return False, .0

    # Go back up
    komo = standardKomo(C, 2)

    komo.addObjective([], ry.FS.vectorZ, ['l_gripper'], ry.OT.eq, [1e1], [0., 0., 1.])
    komo.addObjective([1.], ry.FS.position, ['l_gripper'], ry.OT.eq, [1e1], waypoints[1])
    komo.addObjective([2.], ry.FS.position, ['l_gripper'], ry.OT.eq, [1e1], waypoints[0])

    success = moveBlocking(bot, C, komo, velocity, verbose=verbose)
    if not success:
        print("Failed moving back from poking!")
        return False, .0

    return True, resultingForce

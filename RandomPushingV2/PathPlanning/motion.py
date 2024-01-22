import numpy as np
import robotic as ry
from typing import Tuple
from RobotEnviroment.robotMovement import moveBlocking, moveBlockingAndCheckForce

STANDARD_VELOCITY = .2


def standardKomo(C: ry.Config, phases: int, slicesPerPhase: int=20) -> ry.KOMO:

    q_now = C.getJointState()

    komo = ry.KOMO()
    komo.setConfig(C, True)

    komo.setTiming(phases, slicesPerPhase, 1., 2)

    komo.addControlObjective([], 0, 1e-2)
    komo.addControlObjective([], 1, 1e-1)
    komo.addControlObjective([], 2, 1e1)

    komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq)
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
                        end_dist_range: [float]=[.1, .15],
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
    waypoints = [initial, point+start_pos, point, point-end_pos]
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


def specialPush(bot: ry.BotOp,
                C: ry.Config,
                direction: np.ndarray,
                velocity: float=STANDARD_VELOCITY,
                verbose: int=0) -> Tuple[bool, float]:
    
    """
    Creates a motion problem using "waypoint engineering" approach: define waypoints and motion relative to these
    - We assume the direction vector is normalised.
    """
    komo = standardKomo(C, 2)

    mat = np.eye(3) - np.outer(direction, direction)

    komo.addObjective([0, 1], ry.FS.negDistance, ['l_gripper', 'pushWayPoint'], ry.OT.ineq, [1], [-.1])
    komo.addObjective([1], ry.FS.positionDiff, ['l_gripper', 'startWayPoint'], ry.OT.eq, [1e1])
    komo.addObjective([1, 2], ry.FS.positionDiff, ['l_gripper', 'startWayPoint'], ry.OT.eq, mat)

    komo.addObjective([2], ry.FS.positionDiff, ['l_gripper', 'endWayPoint'], ry.OT.eq, [1e1])

    komo.addObjective([2], ry.FS.qItself, [], ry.OT.eq, [1e1], [], 1) # No motion derivative of q vector ergo the velocity = 0

    komo.addObjective([1, 2], ry.FS.vectorX, ['l_gripper'], ry.OT.eq, direction.reshape(1, 3))
    komo.addObjective([1, 2], ry.FS.vectorZ, ['l_gripper'], ry.OT.eq, [1], -direction)

    # komo.addObjective([1, 2], ry.FS.scalarProductYZ, ['l_gripper', 'table'], ry.OT.ineq, [-1e1], [0.]) # Don't squish camera onto table

    success, maxForce = moveBlockingAndCheckForce(bot, C, komo, velocity, verbose=verbose)

    return success, maxForce


def doPushThroughWaypoints(bot: ry.BotOp,
                           C: ry.Config,
                           waypoints: [np.ndarray],
                           velocity: float=.2,
                           verbose: int=0) -> Tuple[bool, float]:
    
    # Calculate gripper direction
    gripper_dir = waypoints[-1]-waypoints[1]
    gripper_dir /= np.linalg.norm(gripper_dir)
    # Gripper should be at a 45 degree angle with respect to the table.
    gripper_dir[2] = -np.sqrt(2)
    gripper_dir /= np.linalg.norm(gripper_dir)
    gripper_dir *= -1

    # Define the komo problem
    total_move_positions = len(waypoints)*2. - 1.
    komo = standardKomo(C, total_move_positions)

    komo.addObjective([1., total_move_positions], ry.FS.vectorZ, ['l_gripper'], ry.OT.eq, [1e1], gripper_dir)
    komo.addObjective([1., total_move_positions], ry.FS.scalarProductXZ, ['l_gripper', 'table'], ry.OT.eq, [1e1], [0.])
    komo.addObjective([1., total_move_positions], ry.FS.scalarProductYZ, ['l_gripper', 'table'], ry.OT.ineq, [-1e1], [0.])

    for i, way in enumerate(waypoints):
        komo.addObjective([i], ry.FS.position, ['l_gripper'], ry.OT.eq, [1e1], way)

    waypoints.reverse()
    for i, way in enumerate(waypoints[1:]):
        komo.addObjective([i+len(waypoints)], ry.FS.position, ['l_gripper'], ry.OT.eq, [1e1], way)

    # Execute motion if possible
    success, maxForce = moveBlockingAndCheckForce(bot, C, komo, velocity, verbose=verbose)
    return success, maxForce


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


def straightPush(bot: ry.BotOp,
                C: ry.Config,
                start: np.ndarray,
                obj_pos: np.ndarray,
                end: np.ndarray,
                velocity: float=.2,
                verbose: int=0) -> bool:
    
    # Calculate gripper direction
    gripper_dir = end-start
    gripper_dir /= np.linalg.norm(gripper_dir)
    gripper_dir *= -1.


    # Define the komo problem
    komo = ry.KOMO()
    komo.setConfig(C, True)
    komo.setTiming(2., 20, 1., 1)

    komo.addControlObjective([], 1, 1e1)
    # komo.addControlObjective([], 2, 1e1)

    komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq)
    komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq)

    komo.addObjective([1., 2.], ry.FS.vectorY, ['l_gripper'], ry.OT.eq, [1e1], gripper_dir)
    komo.addObjective([1., 2.], ry.FS.vectorZ, ['l_gripper'], ry.OT.eq, [1e1], [0., 0., 1.])

    komo.addObjective([1., 2.], ry.FS.distance, ["l_gripper", "start"], ry.OT.sos, [1e1])
    komo.addObjective([1., 2.], ry.FS.distance, ["l_gripper", "end"], ry.OT.sos, [1e1])

    # komo.addObjective([1., 2.], ry.FS.distance, ["l_gripper", "predicted_obj"], ry.OT.sos, [1e2])
    delta = end-start
    delta /= np.linalg.norm(delta, 2)


    komo.addObjective([1,2], ry.FS.positionDiff, ['l_gripper', "start"], ry.OT.eq, (np.eye(3)-np.outer(delta,delta)))

    # komo.addObjective([1.5], ry.FS.position, ['l_gripper'], ry.OT.eq, [1e1], obj_pos)

    komo.addObjective([1.], ry.FS.positionDiff, ['l_gripper', "start"], ry.OT.eq, [1e1])
    komo.addObjective([2.], ry.FS.positionDiff, ['l_gripper', "end"], ry.OT.eq, [1e1])

    komo.addObjective([2], ry.FS.qItself, [], ry.OT.eq, [1e1], [], 1)

    # Execute motion if possible
    success = moveBlocking(bot, C, komo, velocity, verbose=verbose)
    return success

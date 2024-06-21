"""
Infinite bridge building program
"""

import numpy as np
import robotic as ry
from bridgebuild_env import create_n_boxes

import manipulation as manip
import robot_execution as robex


# Setup the configuration
C = ry.Config()
C.addFile(ry.raiPath("scenarios/pandaSingle.g"))
C = create_n_boxes(C=C, N=3, position="fixed")
C.delFrame("panda_collCameraWrist") # Remove so collision with wrist camera is ignored
C.view()

# Start the robot
bot = robex.Robot(C, on_real=False)
bot.goHome(C)

# We define three different pick and place motions, each one specific to a block
def base_block(block: str, position: list) -> manip.ManipulationModelling:
    """
    This is the first block in the bridge building process.
    It is set in a random position in the x-y plane and maintains
    it's initial vertical orientation.
    """

    graspDirection = np.random.choice(["x", "y"])
    M = manip.ManipulationModelling(C, "Base block", helpers=["l_gripper"])
    M.setup_pick_and_place_waypoints("l_gripper", block) # Initialize pick and place problem

    M.grasp_box(1, "l_gripper", block, "l_palm", graspDirection) # Grasp motion
    M.place_box(2, block, "table", "l_palm", "z") # Place motion

    M.target_relative_xy_position(2.0, block, "table", position) # Place at the specified goal position

    return M

def support_block(block: str, base_block: str, angle: float) -> manip.ManipulationModelling:
    """
    This is the second block in the bridge building process.
    It's position is defined as being at a specific distance to
    the initial block. Within this constraint it can be at any
    position around the initial block.
    """

    graspDirection = np.random.choice(["x", "y"])
    M = manip.ManipulationModelling(C, "Support block", helpers=["l_gripper"])
    M.setup_pick_and_place_waypoints("l_gripper", block) # Initialize pick and place problem

    M.grasp_box(1, "l_gripper", block, "l_palm", graspDirection) # Grasp motion
    M.place_box(2, block, "table", "l_palm", "z") # Place motion
    
    distance_to_base = .11
    offset = C.getFrame(base_block).getPosition()[:2]
    position = np.array([np.cos(angle), np.sin(angle)]) * distance_to_base + offset
    M.target_relative_xy_position(2, block, "table", position.tolist())

    return M

def roof_block(block: str, base_block: str, support_block: str) -> manip.ManipulationModelling:
    """
    This is the third and final block. It's position is perpendicular
    to the other blocks with respect to the z axis. It's rotation
    in the x or y axis will be determined by the direction of the vector
    that defines the relative position between the two support blocks.
    """

    graspDirection = np.random.choice(["x", "y"])
    placeDirection = "y" if graspDirection == "x" else "x" # keep motions simple
    M = manip.ManipulationModelling(C, "Roof block", helpers=["l_gripper"])
    M.setup_pick_and_place_waypoints("l_gripper", block, homing_scale=1e-1) # Initialize pick and place problem

    M.grasp_box(1, "l_gripper", block, "l_palm", graspDirection) # Grasp motion
    # M.place_box(2, block, "table", "l_palm", placeDirection) # Place motion

    base_block_pos = C.getFrame(base_block).getPosition()
    support_block_pos = C.getFrame(support_block).getPosition()
    center_point = (support_block_pos + base_block_pos) * .5
    M.target_relative_xy_position(2, block, "table", center_point.tolist()) # Position

    dir_vec = support_block_pos - base_block_pos
    dir_vec /= np.linalg.norm(dir_vec)
    M.komo.addObjective([2], ry.FS.vectorZ, [block], ry.OT.eq, [1e1], dir_vec)

    M.komo.addObjective([2], ry.FS.negDistance, [block, support_block], ry.OT.ineq, [-1e1], [-.02])
    M.komo.addObjective([2], ry.FS.negDistance, [block, base_block], ry.OT.ineq, [-1e1], [-.02])

    if placeDirection == "x":
        M.komo.addObjective([2], ry.FS.vectorX, [block], ry.OT.eq, [1e1], [0., 0., 1.])

    elif placeDirection == "y":
        M.komo.addObjective([2], ry.FS.vectorY, [block], ry.OT.eq, [1e1], [0., 0., 1.])
    
    return M

def fill_pick_place_gaps_and_run(manipulation_model: manip.ManipulationModelling, box: str, boxes: list):

    # Picking
    M2 = manipulation_model.sub_motion(0)
    M2.retract([0.0, 0.2], "l_gripper")
    M2.approach([0.8, 1.0], "l_gripper")
    for b in boxes:
        M2.keep_distance([], b, "l_palm")

    path2 = M2.solve()
    if not M2.feasible:
        return False

    # Placing
    M3 = manipulation_model.sub_motion(1)
    M3.retract([0.0, 0.2], "l_gripper")
    M3.approach([0.8, 1.0], "l_gripper")
    M3.keep_distances([.2, .8], boxes.copy(), margin=.01)
    for b in boxes:
        M3.keep_distance([], b, "l_palm")

    path3 = M3.solve()
    if not M3.ret.feasible:
        return False

    # Run motion
    # M2.play(C)
    # C.attach("l_gripper", box)
    # M3.play(C)
    # C.attach("table", box)

    bot.execute_path_blocking(C, path=path2, time_to_solve=2)
    C.attach("l_gripper", box)
    bot.grasp(C)

    bot.execute_path_blocking(C, path=path3, time_to_solve=2)
    C.attach("table", box)
    bot.release(C)

    return True


max_attempts_per_instruction = 10
box_order = [f"box{i}" for i in range(1, 4)]
while True:
    # Build the bridge
    n_success = 0
    attempts_per_instruction = 0 # Is there a smarter way of doing this?
    np.random.shuffle(box_order)

    while n_success < 3 and attempts_per_instruction < max_attempts_per_instruction:

        # Solve the basic inverse kinematics
        if n_success == 0:
            position = [np.random.uniform(-.2, .2), np.random.uniform(.2, .4)]
            M = base_block(box_order[n_success], position)
        elif n_success == 1:
            angle = np.random.uniform(.0, 2.*np.pi)
            M = support_block(box_order[n_success], box_order[n_success-1], angle)
        elif n_success == 2:
            M = roof_block(box_order[n_success], box_order[n_success-1], box_order[n_success-2])

        M.solve()
        if not M.feasible:
            print("Problem at kinematics")
            attempts_per_instruction += 1
            continue

        # Solve the full path
        if not fill_pick_place_gaps_and_run(M, box_order[n_success], box_order):
            print("Problem at full motion")
            attempts_per_instruction += 1
            continue

        n_success += 1
        attempts_per_instruction = 0

    # Deconstruct the bridge
    n_success = 0
    attempts_per_instruction = 0

    while n_success < 1 and attempts_per_instruction < max_attempts_per_instruction:

        # Solve the basic inverse kinematics
        position = [np.random.uniform(-.2, .2), np.random.uniform(.2, .4)]
        M = base_block(box_order[2-n_success], position)
        M.keep_distances([2], box_order.copy(), margin=.05)

        M.solve()
        if not M.feasible:
            print("Problem at kinematics")
            attempts_per_instruction += 1
            continue

        # Solve the full path
        if not fill_pick_place_gaps_and_run(M, box_order[2-n_success], box_order):
            print("Problem at full motion")
            attempts_per_instruction += 1
            continue

        n_success += 1
        attempts_per_instruction = 0

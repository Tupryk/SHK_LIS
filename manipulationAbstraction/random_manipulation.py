import robotic as ry
import manipulation as manip
import numpy as np
import random
import robot_execution as robex

C = ry.Config()
C.addFile(ry.raiPath('../rai-robotModels/scenarios/pandaSingle.g'))

midpoint = np.array([-0.105, 0.4, 0.745])
# C.addFrame("obstacle") \
#     .setPosition(midpoint) \
#     .setShape(ry.ST.ssBox, size=[0.09, 0.45, 0.23, 0.001]) \
#     .setColor([0, 0, 1]) \
#     .setContact(1) \
#     .setMass(.1)

for i in range(3):
    box_name = 'box{}'.format(i + 1)

    color = [0, 0, 0]
    color[i] = 1
    position_val1 = 0.1 * (i - 5)
    C.addFrame(box_name) \
        .setPosition([position_val1, 0.05, 0.705]) \
        .setShape(ry.ST.ssBox, size=[0.04, 0.04, 0.12, 0.001]) \
        .setColor(color) \
        .setContact(1) \
        .setMass(.1)

#C.delFrame("panda_collCameraWrist")

# for convenience, a few definitions:
gripper = "l_gripper"
palm = "l_palm"
box = "box1"
table = "table"

C.view()

def pick_place(graspDirection, placeDirection, placePosition, object_, info) -> bool:
    # Define the joint states at grasp and place
    M = manip.ManipulationModelling(C, info, helpers=[gripper])
    M.setup_pick_and_place_waypoints(gripper, object_, homing_scale=1e-1)
    M.grasp_box(1., gripper, object_, palm, graspDirection)
    M.place_box(2., object_, table, palm, placeDirection)
    M.target_relative_xy_position(2., object_, table, placePosition)
    M.solve()
    if not M.feasible:
        return False

    # Define the in between joint states when going for the grasp
    M1 = M.sub_motion(0)
    M1.no_collision([.3,.7], palm, object_, margin=.05)
    M1.retract([.0, .2], gripper)
    M1.approach([.8, 1.], gripper)
    path1 = M1.solve()
    if not M1.feasible:
        return False

    # Define the in between joint states when placing the object
    M2 = M.sub_motion(1)
    M2.no_collision([], table, object_)
    M2.no_collision([.2, .8], table, object_, .04)
    path2 = M2.solve()
    if not M2.feasible:
        return False

    # M1.play(C)
    # C.attach(gripper, box)
    # M2.play(C)
    # C.attach(table, box)

    robot = robex.Robot(C, on_real=True)
    robot.execute_path_blocking(C, path1)
    robot.grasp(C)
    C.attach(gripper, object_)
    robot.execute_path_blocking(C, path2)
    C.attach(table, object_)
    robot.release(C)

    return True


def build_house():
    paths = []
    for i, box in enumerate(["box1", "box2", "box3"]):
        M = manip.ManipulationModelling(C, info, helpers=[gripper])
        M.setup_pick_and_place_waypoints(gripper, object_, homing_scale=1e-1)
        M.grasp_box(1., gripper, object_, palm, graspDirection)
        M.place_box(2., object_, table, palm, placeDirection)
        M.target_relative_xy_position(2., object_, table, placePosition)
        M.solve()
        if not M.feasible:
            return False

        # Define the in between joint states when going for the grasp
        M1 = M.sub_motion(0)
        M1.no_collision([.3,.7], palm, object_, margin=.05)
        M1.retract([.0, .2], gripper)
        M1.approach([.8, 1.], gripper)
        path1 = M1.solve()
        if not M1.feasible:
            return False

        # Define the in between joint states when placing the object
        M2 = M.sub_motion(1)
        M2.no_collision([], table, object_)
        M2.no_collision([.2, .8], table, object_, .04)
        path2 = M2.solve()
        if not M2.feasible:
            return False


def pull(object_, info) -> bool:
    return False


def push(pushPosition, info) -> bool:
    M = manip.ManipulationModelling(C, info, ['l_gripper'])
    M.setup_pick_and_place_waypoints(gripper, box, 1e-1)
    M.straight_push([1.,2.], box, gripper, table)
    M.komo.addObjective([2.], ry.FS.position, [box], ry.OT.eq, 1e1*np.array([[1,0,0],[0,1,0]]), pushPosition)
    M.place_box(2, object_, table, palm)
    M.solve()
    if not M.feasible:
        return False

    M1 = M.sub_motion(0)
    M1.retractPush([.0, .15], gripper, .03)
    M1.approachPush([.85, 1.], gripper, .03)
    M1.no_collision([.15,.85], box, "l_finger1", .02)
    M1.no_collision([.15,.85], box, "l_finger2", .02)
    M1.no_collision([.15,.85], box, 'l_palm', .02)
    M1.no_collision([], table, "l_finger1", .0)
    M1.no_collision([], table, "l_finger2", .0)
    path1 = M1.solve()
    if not M1.feasible:
        return False

    M2 = M.sub_motion(1)
    M2.komo.addObjective([], ry.FS.positionRel, [gripper, '_push_start'], ry.OT.eq, 1e1*np.array([[1,0,0],[0,0,1]]))
    path2 = M2.solve()
    if not M2.feasible:
         return False

    M1.play(C, 1.)
    C.attach(gripper, box)
    M2.play(C, 1.)
    C.attach(table, box)

    # robot = robex.Robot(C, on_real=True)
    # robot.grasp(C)
    # robot.execute_path_blocking(C, path1)
    # C.attach(gripper, box)
    # robot.execute_path_blocking(C, path2)
    # C.attach(table, box)

    return True

robot = robex.Robot(C, on_real=True)
robot.goHome(C)
del robot

attempt_count = 100
data = []
for l in range(attempt_count):

    #action = np.random.choice(["grasp", "push"])
    action = np.random.choice(["push"])
    if action == "grasp":
        graspDirection = random.choice(['x', 'y', 'z'])
        placeDirection = random.choice(['x', 'y', 'z', 'xNeg', 'yNeg', 'zNeg'])
        placePosition = midpoint + np.random.uniform(-.1, .1, size=3)
        object_ = np.random.choice([f"box{i+1}" for i in range(3)])
        info = f'placement {l}: grasp {graspDirection} place {placeDirection}'
        success = pick_place(graspDirection, placeDirection, placePosition, object_, info)

    elif action == "push":
        pushPosition = midpoint + np.random.uniform(-.1, .1, size=3)
        info = f'push {l}'
        success = push(pushPosition, info)

    else:
        raise Exception(f'Action "{action}" is not defined!')
    
    data.append({"action": action, "success": success})
print(data)

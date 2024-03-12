import robotic as ry
import manipulation as manip
import numpy as np
import random
import robot_execution as robex

C = ry.Config()
C.addFile(ry.raiPath('../rai-robotModels/scenarios/pandaSingle.g'))
BOX_COUNT = 1

midpoint = np.array([-0.105, 0.4, 0.745])

for i in range(BOX_COUNT):
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
ON_REAL = True

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
    M2.no_collision([], table, "panda_collCameraWrist")
    M2.no_collision([.2, .8], table, object_, .04)
    M2.no_collision([], palm, object_)
    path2 = M2.solve()
    if not M2.feasible:
        return False

    if ON_REAL:
        robot = robex.Robot(C, on_real=True)
        robot.execute_path_blocking(C, path1)
        robot.grasp(C)
        C.attach(gripper, object_)
        robot.execute_path_blocking(C, path2)
        C.attach(table, object_)
        robot.release(C)
    else:
        M1.play(C)
        C.attach(gripper, box)
        M2.play(C)
        C.attach(table, box)

    return True


def build_house():
    pass


def updateObjectPosition(object_, info) -> bool:
        M = manip.ManipulationModelling(C, info, ['l_gripper'])
        M.setup_inverse_kinematics()
        M.komo.addObjective([1.], ry.FS.scalarProductZZ, ["l_gripper", "table"], ry.OT.ineq, [-1e1], [np.cos(np.deg2rad(100))])
        M.komo.addObjective([1.], ry.FS.scalarProductZZ, ["l_gripper", "table"], ry.OT.ineq, [1e1], [np.cos(np.deg2rad(60))])
        M.komo.addObjective([1.], ry.FS.positionRel, [object_, "cameraWrist"], ry.OT.eq, [1e1], [.0, .0, .4])
        path = M.solve()
        if not M.feasible:
            return False
        
        robot = robex.Robot(C, on_real=ON_REAL)
        robot.execute_path_blocking(C, path)
        obj_dims = C.getFrame(object_).getSize()
        position, quaternion = robot.get_sigle_box_pos_qurn(C, obj_dims, midpoint, radius=.2)
        C.getFrame(object_).setPosition(position).setQuaternion(quaternion)
        return True


def pull(object_, info) -> bool:
    M = manip.ManipulationModelling(C, info, ['l_gripper'])
    M.setup_pick_and_place_waypoints(gripper, object_, 1e-1)
    M.pull([1.,2.], object_, gripper, table)
    placePosition = midpoint + np.random.uniform(-.1, .1, size=3)
    M.komo.addObjective([2.], ry.FS.position, [object_], ry.OT.eq, 1e1*np.array([[1,0,0],[0,1,0]]), placePosition)
    M.solve()
    if not M.feasible:
        return False

    M1 = M.sub_motion(0, accumulated_collisions=False)
    M1.retractPush([.0, .15], gripper, .03)
    M1.approachPush([.85, 1.], gripper, .03)
    path1 = M1.solve()
    if not M1.feasible:
        return False

    M2 = M.sub_motion(1, accumulated_collisions=False)
    path2 = M2.solve()
    if not M2.feasible:
         return False
    
    if ON_REAL:
        robot = robex.Robot(C, on_real=True)
        robot.release(C)
        robot.execute_path_blocking(C, path1)
        C.attach(gripper, box)
        robot.execute_path_blocking(C, path2)
        C.attach(table, box)
    else:
        M1.play(C, 1.)
        C.attach(gripper, object_)
        M2.play(C, 1.)
        C.attach(table, object_)

    return True


def push(object_, pushPosition, info) -> bool:
    M = manip.ManipulationModelling(C, info, ['l_gripper'])
    M.setup_pick_and_place_waypoints(gripper, object_, 1e-1)
    M.straight_push([1.,2.], object_, gripper, table)
    M.komo.addObjective([2.], ry.FS.position, [object_], ry.OT.eq, 1e1*np.array([[1,0,0],[0,1,0]]), pushPosition)
    M.solve()
    if not M.feasible:
        return False

    M1 = M.sub_motion(0)
    M1.retractPush([.0, .15], gripper, .03)
    M1.approachPush([.85, 1.], gripper, .03)
    M1.no_collision([.15,.85], object_, "l_finger1", .02)
    M1.no_collision([.15,.85], object_, "l_finger2", .02)
    M1.no_collision([.15,.85], object_, 'l_palm', .02)
    M1.no_collision([], table, "l_finger1", .0)
    M1.no_collision([], table, "l_finger2", .0)
    M1.no_collision([], table, "panda_collCameraWrist")
    path1 = M1.solve()
    if not M1.feasible:
        return False

    M2 = M.sub_motion(1)
    M2.komo.addObjective([], ry.FS.positionRel, [gripper, '_push_start'], ry.OT.eq, 1e1*np.array([[1,0,0],[0,0,1]]))
    path2 = M2.solve()
    if not M2.feasible:
         return False

    if ON_REAL:
        robot = robex.Robot(C, on_real=True)
        robot.grasp(C)
        robot.execute_path_blocking(C, path1)
        C.attach(gripper, object_)
        robot.execute_path_blocking(C, path2)
        C.attach(table, object_)
    else:
        M1.play(C, 1.)
        C.attach(gripper, object_)
        M2.play(C, 1.)
        C.attach(table, object_)

    return True

if ON_REAL:
    robot = robex.Robot(C, on_real=True)
    robot.goHome(C)
    del robot

attempt_count = 100
data = []
successes = 0
just_updated = False
success = False
for l in range(attempt_count):

    #action = np.random.choice(["pick_place", "push", "pull", "view"])
    action = np.random.choice(["pick_place", "push", "view"])
    object_ = np.random.choice([f"box{i+1}" for i in range(BOX_COUNT)])
    objective_pos = midpoint + np.random.uniform(-.1, .1, size=3)
    info = f"action number {l} ({action}): "

    if action == "pick_place":
        graspDirection = random.choice(['x', 'y', 'z'])
        placeDirection = random.choice(['x', 'y', 'z', 'xNeg', 'yNeg', 'zNeg'])
        info += f'grasp {graspDirection} place {placeDirection}'
        success = pick_place(graspDirection, placeDirection, objective_pos, object_, info)

    elif action == "push":
        success = push(object_, objective_pos, info)

    elif action == "pull":
        success = pull(object_, objective_pos, info)

    elif action == "view":
        if successes >= 1 and not just_updated:
            success = updateObjectPosition(object_, info)
            just_updated = True

    else:
        raise Exception(f'Action "{action}" is not defined!')
    if success:
        if action != "view":
            just_updated = False
        successes += 1
    data.append({"action": info, "success": success})
print(data)

import os
import robotic as ry
import manipulation as manip
import numpy as np
import robot_execution as robex
from matplotlib.image import imsave
from scipy.spatial.transform import Rotation as R
import math
from utils import draw_arena, sample_arena

# elliptical arena parameters
A = .4
B = .35
OFFSET = [0, .2]

def return_brige_build_path(C, bridgePosition, gripper="l_gripper", palm="l_palm", table="table", visualize=False):
    bridge_paths = []

    qStart = C.getJointState()

    for i in range(3):
        pass 

    box = "box1"
    possible = False
    for graspDirection in ["x", "y", "z"]:
        for placeDirection in ["z", "zNeg"]:
            M = manip.ManipulationModelling(C, helpers=[gripper])
            M.setup_pick_and_place_waypoints(gripper, box, homing_scale=1e-1)
            M.grasp_box(1., gripper, box, palm, graspDirection)
            M.place_box(2., box, table, palm, placeDirection)
            M.target_relative_xy_position(2., box, table, bridgePosition)
            M.solve()
            if not M.feasible:
                continue

            # Define the in between joint states when going for the grasp
            M1 = M.sub_motion(0)
            M1.keep_distance([.3,.7], palm, box, margin=.05)
            M1.retract([.0, .2], gripper)
            M1.approach([.8, 1.], gripper)
            path1 = M1.solve()
            if not M1.feasible:
                continue

            # Define the in between joint states when placing the object
            M2 = M.sub_motion(1)
            M2.keep_distance([], table, box)
            M2.keep_distance([.2, .8], table, box, .04)
            path2 = M2.solve()
            if not M2.feasible:
                continue
            possible = True
            break
        if possible:
            break

    if not possible:
        return False, None
    
    bridge_paths.append(path1)
    bridge_paths.append(path2)

    frameStart = M.komo.getFrameState(0)
    X = M.komo.getFrameState(1)
    C.setFrameState(X)
    C.setJointState(path2[-1])

    box = "box2"
    possible = False
    for graspDirection in ["x", "y", "z"]:
        for placeDirection in ["z", "zNeg"]:
            M = manip.ManipulationModelling(C, helpers=[gripper])
            M.setup_pick_and_place_waypoints(gripper, box, homing_scale=1e-1)
            M.grasp_box(1., gripper, box, palm, graspDirection)
            M.place_box(2., box, table, palm, placeDirection)
            M.komo.addObjective([2.], ry.FS.negDistance, ["box2", "box1"], ry.OT.eq, [1e1], [-.05])
            M.solve()
            if not M.feasible:
                continue

            # Define the in between joint states when going for the grasp
            M3 = M.sub_motion(0)
            M3.keep_distance([.3,.7], palm, box, margin=.05)
            M3.retract([.0, .2], gripper)
            M3.approach([.8, 1.], gripper)
            path1 = M3.solve()
            if not M3.feasible:
                continue

            # Define the in between joint states when placing the object
            M4 = M.sub_motion(1)
            M4.keep_distance([], table, box)
            M4.keep_distance([], "box1", palm)
            M4.keep_distance([], "box2", palm)
            M4.keep_distance([.2, .8], table, box, .04)
            path2 = M4.solve()
            if not M4.feasible:
                continue
            possible = True
            break
        if possible:
            break

    if not possible:
        return False, None
    
    bridge_paths.append(path1)
    bridge_paths.append(path2)
    
    X = M.komo.getFrameState(1)
    C.setFrameState(X)
    C.setJointState(path2[-1])

    box = "box3"
    possible = False
    for graspDirection in ["x", "y"]:
        M = manip.ManipulationModelling(C, helpers=[gripper])
        M.setup_pick_and_place_waypoints(gripper, box, homing_scale=1e-1)
        M.grasp_box(1., gripper, box, palm, grasp_direction="y")
        
        box3_dir = C.getFrame("box2").getPosition() - C.getFrame("box1").getPosition()
        box3_dir /= np.linalg.norm(box3_dir)

        M.komo.addObjective([2.], ry.FS.vectorZ, [box], ry.OT.eq, [1e1], box3_dir)
        M.komo.addObjective([2.], ry.FS.vectorX, [box], ry.OT.eq, [1e1], [0, 0, -1])
        end_pos = bridgePosition + box3_dir *.05
        end_pos[2] = .69 + .11
        M.komo.addObjective([2.], ry.FS.position, [box], ry.OT.eq, [1e1], end_pos)
        M.solve()
        if not M.feasible:
            continue

        # Define the in between joint states when going for the grasp
        M5 = M.sub_motion(0)
        M5.keep_distance([.3, .7], palm, box, margin=.05)
        M5.retract([.0, .2], gripper)
        M5.approach([.8, 1.], gripper)
        path1 = M5.solve()
        if not M5.feasible:
            continue

        # Define the in between joint states when placing the object
        M6 = M.sub_motion(1)
        M6.keep_distance([], table, box)
        M6.keep_distance([], "box1", palm)
        M6.keep_distance([], "box2", palm)
        M6.keep_distance([.0, .9], "box1", box)
        M6.keep_distance([.0, .9], "box2", box)
        M6.keep_distance([.2, .8], table, box, .04)
        M6.approach([.8, 1.], gripper)
        path2 = M6.solve()
        if not M6.feasible:
            continue
        possible = True
        break

    if not possible:
        return False, None
    
    bridge_paths.append(path1)
    bridge_paths.append(path2)
    
    C.setFrameState(frameStart)
    C.setJointState(qStart)

    # make copy of C
    C2 = ry.Config()
    C2.addFile("scenarios/pandaSingleWithTopCamera.g")
    home = C2.getJointState()

    for i in range(3):
        box_name = 'box{}'.format(i + 1)

        color = [0, 0, 0]
        color[i] = 1
        position_val1 = 0.1 * (i - 5)
        C2.addFrame(box_name) \
            .setPosition([position_val1, 0.05, 0.705]) \
            .setShape(ry.ST.ssBox, size=[0.04, 0.04, 0.12, 0.001]) \
            .setColor(color) \
            .setContact(1) \
            .setMass(.1)

    C2.setFrameState(frameStart)
    C2.setJointState(home)
    C2.view()
    robot = robex.Robot(C2, on_real=False)

    img, _ = robot.bot.getImageAndDepth("topCamera")
    if(visualize):

        C2.setJointState(qStart)
        C2.view()       

        robot = robex.Robot(C2, on_real=False)
        for i in range(3):
            box = f"box{i+1}"
            robot.execute_path_blocking(C2, bridge_paths[i*2])
            robot.grasp(C2)
            C2.attach(gripper, box)
            
            robot.execute_path_blocking(C2, bridge_paths[i*2+1])
            C2.attach(table, box)
            robot.release(C2)

    for i in range(1,3):
        rotation_matrix = C2.getFrame(f"box{i}").getRotationMatrix()
        rotation = R.from_matrix(rotation_matrix)

        euler_angles = rotation.as_euler('xyz', degrees=True)

        # kind of ugly hack: if pitch or roll > .5 degrees, i.e. block is not standing on table, return false
        if abs(euler_angles[0])>.1 or abs(euler_angles[1])>.1:
            return False, None
        
    if not math.isclose(C2.getFrame(f"box3").getPosition()[2] - C2.getFrame(f"box1").getPosition()[2], 0.08, abs_tol=0.001):
        return False, None

    return bridge_paths, img


def move_blocks(C, gripper="l_gripper", palm="l_palm", table="table", visualize=True):
    bridge_paths = []

    qStart = C.getJointState()
    frameStart = 0

    for i in range(1,4):

        box = f"box{i}"
        possible = False
        for graspDirection in ["x", "y", "z"]:
            for placeDirection in ["z", "zNeg"]:
                M = manip.ManipulationModelling(C, helpers=[gripper])
                M.setup_pick_and_place_waypoints(gripper, box, homing_scale=1e-1)
                M.grasp_box(1., gripper, box, palm, graspDirection)
                M.place_box(2., box, table, palm, placeDirection)
                M.target_relative_xy_position(2., box, table, sample_arena(A, B, OFFSET))
                
                M.komo.addObjective([2.], ry.FS.angularVel, [gripper], ry.OT.sos, [1e1])
                

                M.keep_distances([2.], [f"box{i}" for i in range(1,4)], .1)

                M.solve()
                
                if not M.feasible:
                    continue

                # Define the in between joint states when going for the grasp
                M1 = M.sub_motion(0)
                M1.keep_distance([.3,.7], palm, box, margin=.08)
                M1.retract([.0, .2], gripper)
                M1.approach([.8, 1.], gripper)
                path1 = M1.solve()
                if not M1.feasible:
                    continue

                # Define the in between joint states when placing the object
                M2 = M.sub_motion(1)
                M2.keep_distance([], table, box)
                M2.keep_distance([.2, .8], table, box, .1)
                path2 = M2.solve()
                if not M2.feasible:
                    continue
                possible = True
                break
            if possible:
                break

        if not possible:
            return False
        
        bridge_paths.append(path1)
        bridge_paths.append(path2)

        if(i==1):
            frameStart = M.komo.getFrameState(0)
        X = M.komo.getFrameState(1)
        C.setFrameState(X)
        C.setJointState(path2[-1])

    C.setFrameState(frameStart)
    C.setJointState(qStart)


    if(visualize):
        C.view()

        draw_arena(C, A, B, OFFSET)

        robot = robex.Robot(C, on_real=False)
        robot.goHome(C)
        for i in range(3):
            box = f"box{i+1}"
            robot.execute_path_blocking(C, bridge_paths[i*2])
            robot.grasp(C)
            C.attach(gripper, box)
            
            robot.execute_path_blocking(C, bridge_paths[i*2+1])
            C.attach(table, box)
            robot.release(C)

    for i in range(1,4):
        rotation_matrix = C.getFrame(f"box{i}").getRotationMatrix()
        rotation = R.from_matrix(rotation_matrix)

        euler_angles = rotation.as_euler('xyz', degrees=True)

        # kind of ugly hack: if pitch or roll > .5 degrees, i.e. block is not on table, return false
        if abs(euler_angles[0])>.1 or abs(euler_angles[1])>.1:
            return False, None


    return True

def random_config(C, a, b, offset):
    for i in range(1,4):

        x_y_pos = sample_arena(a, b, offset)[:2]
        x_y_pos.append(.705)
        C.getFrame(f"box{i}").setPosition(x_y_pos)

    return True



def main():
    # for convenience, a few definitions:
    gripper = "l_gripper"
    palm = "l_palm"
    table = "table"


    # number of restart -> number of restart configurations with starting same block pos, attempt count -> attempts during that restart configuration
    number_of_restart = 1000
    for i in range(number_of_restart):
        C = ry.Config()
        C.addFile("scenarios/pandaSingleWithTopCamera.g")

        midpoint = [-0.105, 0.2, 0.745]

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



        # extract number of files in block pos folder to account for starting index
        file_count = len([f for f in os.listdir("block_pos") if os.path.isfile(os.path.join("block_pos", f))])

        attempt_count = 1

        count=-1
        for _ in range(attempt_count):
            # pick bridge position
            bridgePosition = [midpoint[0] + (np.random.random()*.2 -.1), midpoint[1] + (np.random.random()*.2 -.1)]
            # move blocks randomly inside elliptical arena
            moved_success = move_blocks(C, gripper, palm, table, True)
            #moved_success = random_config(C, A, B, OFFSET)
            # return path of bridge building
            path = None
            if moved_success:
                path, img = return_brige_build_path(C, bridgePosition, gripper, palm, table, True)

            if path:
                # Ensure the directories exist
                os.makedirs("paths", exist_ok=True)
                os.makedirs("block_pos", exist_ok=True)
                os.makedirs("init_scene_img", exist_ok=True)

                count += 1
                index = count + file_count

                imsave(f"init_scene_img/image{index}.png", img)

                np.save(f"npy_paths/path{index}", path)

                with open(f'paths/path{index}.txt', 'w') as file:
                    for array in path:
                        file.write(' '.join(map(str, array)) + '\n')

                box_pos = []
                for i in range(3):
                    box_pos.append(C.getFrame(f"box{i+1}").getPosition())

                with open(f'block_pos/block_pos{index}.txt', 'w') as file:
                    for pos in box_pos:
                        file.write(' '.join(map(str, pos)) + '\n')
        del C

        
            
if __name__ == "__main__":
    main()
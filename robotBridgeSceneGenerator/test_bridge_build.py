import robotic as ry
import numpy as np
import robot_execution as robex

#TODO fix initial fast movement

# for convenience, a few definitions:
gripper = "l_gripper"
palm = "l_palm"
box = "box1"
table = "table"

def read_floats_from_file(file_path):
    float_lists = []
    
    try:
        with open(file_path, 'r') as file:
            for line in file:
                float_list = [float(value) for value in line.strip().split()]
                float_lists.append(float_list)
                
        return float_lists
    
    except Exception as e:
        print(f"An error occurred: {e}")



if __name__ == "__main__":
    C = ry.Config()
    C.addFile(ry.raiPath('../rai-robotModels/scenarios/pandaSingle.g'))
    path= np.load(f"paths/path0.npy")

    print(path[1])

    file_path = 'block_pos/block_pos0.txt' 
    float_entries = read_floats_from_file(file_path)
    print(float_entries)


    for i in range(3):
        box_name = 'box{}'.format(i + 1)

        color = [0, 0, 0]
        color[i] = 1
        position_val1 = -.1 * (i - 5) - .1
        C.addFrame(box_name) \
            .setPosition([float_entries[i][0], float_entries[i][1], 0.705]) \
            .setShape(ry.ST.ssBox, size=[0.04, 0.04, 0.12, 0.001]) \
            .setColor(color) \
            .setContact(1) \
            .setMass(.1)

        C.view(True)

    robot = robex.Robot(C, on_real=False)
    robot.goHome(C)
    for i in range(3):
        box = f"box{i+1}"
        robot.execute_path_blocking(C, path[i*2])
        robot.grasp(C)
        C.attach(gripper, box)
        
        robot.execute_path_blocking(C, path[i*2+1])
        C.attach(table, box)
        robot.release(C)

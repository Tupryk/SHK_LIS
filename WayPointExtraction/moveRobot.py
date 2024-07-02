import robotic as ry
import numpy as np
import time 

def extract_waypoints_linearly_interpolated(C, qs, number_waypoints):
    n_qReal = []

    for i in range(number_waypoints):
        n_qReal.append(qs[i*int(len(qs)/number_waypoints)])

    for i, q in enumerate(n_qReal):
        C.setJointState(q[1:])
        C.addFrame(f'way{i}'). setShape(ry.ST.marker, [.1]) .setPosition(C.getFrame("l_gripper").getPosition())


def main():  
    C = ry.Config()
    C.addFile(ry.raiPath('../rai-robotModels/scenarios/pandaSingle.g'))
    C.view(True)

    qReal = np.load("logs/qReal.npy")
    extract_waypoints_linearly_interpolated(C, qReal, 10)
    C.view(True)


if __name__ == "__main__":
    main()
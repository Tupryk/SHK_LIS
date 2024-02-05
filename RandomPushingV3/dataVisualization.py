import json
import numpy as np
import robotic as ry
import matplotlib.pyplot as plt
from raiUtils import setupConfig


def visualizeForces(attempt_paths: [str]):

    for attempt_path in attempt_paths:
        # Setup config
        attempt_data = json.load(open(attempt_path))
        if not attempt_data["success"]: continue # We can only plot data from successful pushes!
        C: ry.Config = setupConfig()

        # Calculate forces
        forces = []
        for i, tauEx in enumerate(attempt_data["tauExternal"]):

            C.setJointState(attempt_data["jointStates"][i])
            y, J = C.eval(ry.FS.position, ['l_gripper'], [[0, 1, 0]])
            J = np.linalg.pinv(J.T)
            F = np.abs(J @ tauEx)
            forces.append(F)

        # Plot data
        plt.plot(forces)
    plt.show()

visualizeForces([f"./data/push_attempt_{i+1}.json" for i in range(174)])

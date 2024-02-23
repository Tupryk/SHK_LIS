import numpy as np
import robotic as ry


def solve_path_motion(C: ry.Config, rrt_path: np.ndarray) -> np.ndarray:
    """
    This function supposes the the robot is already grasping the box.
    """
    z_pos = C.getFrame("l_gripper").getPosition()[2]
    phases = len(rrt_path)-1
    komo = ry.KOMO()
    komo.setConfig(C, False)
    komo.setTiming(phases, 5, 1., 2)

    komo.addControlObjective([], 1, 1e-1)
    komo.addControlObjective([], 2, 1e-1)

    komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq)
    komo.addObjective([1, phases], ry.FS.vectorZ, ["l_gripper"], ry.OT.eq, [1e1], [0, 0, 1])
    komo.addObjective([1, phases], ry.FS.position, ["l_gripper"], ry.OT.eq, [0, 0, 1e1], [0, 0, z_pos])

    for i in range(1, len(rrt_path)):
        komo.addObjective([i], ry.FS.position, ["l_gripper"], ry.OT.eq, [1, 1, 0], [rrt_path[i][0], rrt_path[i][1], 0])

    ret = ry.NLP_Solver(komo.nlp(), verbose=0).solve()
    if not ret.feasible:
        print("RRT path not possible!")
        return np.array([])
    
    return komo.getPath()

import robotic as ry
import manipulation as manip
import numpy as np
import random

C = ry.Config()
C.addFile(ry.raiPath('../rai-robotModels/scenarios/pandaSingle.g'))

midpoint = np.array([-0.105, 0.4, 0.705])
C.addFrame("box") \
    .setPosition(midpoint) \
    .setShape(ry.ST.ssBox, size=[0.04, 0.12, 0.04, 0.001]) \
    .setColor([0, 0, 1]) \
    .setContact(1) \
    .setMass(.1)

#C.delFrame("panda_collCameraWrist")

# for convenience, a few definitions:
gripper = "l_gripper"
palm = "l_palm"
box = "box"
table = "table"

C.view()

def pull(object_, info) -> bool:
    M = manip.ManipulationModelling(C, info, ['l_gripper'])
    M.setup_pick_and_place_waypoints(gripper, object_, 1e-1)
    M.straight_pull([1.,2.], object_, gripper, table)
    placePosition = midpoint + np.random.uniform(-.1, .1, size=3)
    M.komo.addObjective([2.], ry.FS.position, [object_], ry.OT.eq, 1e1*np.array([[1,0,0],[0,1,0]]), placePosition)
    M.solve()
    if not M.feasible:
        return False

    M1 = M.sub_motion(0, accumulated_collisions=False)
    M1.retractPush([.0, .15], gripper, .03)
    M1.approachPush([.85, 1.], gripper, .03)
    # M1.no_collision([.15,.85], object_, "l_finger1", .02)
    # M1.no_collision([.15,.85], object_, "l_finger2", .02)
    # M1.no_collision([.15,.85], object_, 'l_palm', .02)
    # M1.no_collision([], table, "l_finger1", .0)
    # M1.no_collision([], table, "l_finger2", .0)
    path1 = M1.solve()
    if not M1.feasible:
        return False

    M2 = M.sub_motion(1, accumulated_collisions=False)
    #M2.komo.addObjective([], ry.FS.positionRel, [gripper, '_pull_start'], ry.OT.eq, 1e1*np.array([[1,0,0],[0,0,1]]))
    path2 = M2.solve()
    if not M2.feasible:
         return False

    M1.play(C, 1.)
    C.attach(gripper, object_)
    M2.play(C, 1.)
    C.attach(table, object_)

    return True


attempt_count = 100
data = []
for l in range(attempt_count):
    
    action = "pull"
    if action == "pull":
        placePosition = [midpoint[0] + np.random.random()*.4 -.2, midpoint[1] + np.random.random()*.4 -.2]
        object_ = np.random.choice([box])
        success = pull(object_, "")

    else:
        raise Exception(f'Action "{action}" is not defined!')
    
    data.append({"action": action, "success": success})
print(data)

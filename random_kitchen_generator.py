import time
import rowan
import numpy as np
import robotic as ry
from load_random_plys import load_rand_objs, load_single_from_type


if __name__ == "__main__":

    my_seed = np.random.randint(0, 100_000)
    print("Seed:", my_seed)
    np.random.seed(my_seed)

    C = ry.Config()
    C.addFile("/home/denis/RoboCasaScene/rai-robotModels/ranger/ranger.g")

    #C = load_single_from_type(C, pos=[1., 1., .5], root_path="rai_jointed/fixtures/stovetops")
    
    # C = load_single_from_type(C, pos=[1., -1.5, .8], root_path="rai_jointed/fixtures/toasters")
    C = load_single_from_type(C, pos=[1., 1., .5], rot=rowan.from_euler(0, 0, 0), root_path="rai_jointed/fixtures/stoves")
    C = load_single_from_type(C, pos=[2., 1., .9], rot=rowan.from_euler(0, 0, 0), root_path="rai_jointed/fixtures/sinks")
    C.addFrame("counter_sink") \
    .setPosition([2., 1., .36]) \
    .setShape(ry.ST.ssBox, [1.2, .8, .7, .001])


    C = load_single_from_type(C, pos=[3., -1., .5], rot=rowan.from_euler(np.pi, 0, 0), root_path="rai_jointed/fixtures/ovens")
    C = load_single_from_type(C, pos=[2., -1., .5], rot=rowan.from_euler(np.pi, 0, 0), root_path="rai_jointed/fixtures/dishwashers")

   # Counter with random objects
    C = load_rand_objs(C, dims=[1., .6, 0.], pos=[1., -1., .8])
    C.addFrame("counter") \
        .setPosition([1., -1, .35]) \
        .setShape(ry.ST.ssBox, [1.2, .8, .7, .001])

    C.view(True)
    C.animate()
    # C.view(True)
    
    # S = ry.Simulation(C, ry.SimulationEngine.physx, verbose=0)

    # tau=.01
    # for i in range(200):
    #     time.sleep(tau)
    #     S.step([], tau,  ry.ControlMode.none)
    #     C.view()

import robotic as ry 
import numpy as np 

def create_n_boxes(C, N, position='fixed'):

    color_map = [[1,0,.5], # pink
                [0.5,1,0], # lime green
                [.5,1,1], # light turqois
                [0.5,0,1], # violet
                [1,0,0], # red
                [0,1,0], # green
                [0,0,1], # blue
                [1,.5,0], # orange
                [1,1,.5], # lime yellow
                [1,.5,1], # light pink                            
                ]

    for i in range(N):
        box_name = 'box{}'.format(i + 1)

        if position == 'fixed':
            position_val1 = 0.13 * (i - 2.5)
            C.addFrame(box_name) \
                .setPosition([position_val1, 0.05, 0.705]) \
                .setShape(ry.ST.ssBox, size=[0.04, 0.04, 0.12, 0.001]) \
                .setColor(color_map[i % len(color_map)]) \
                .setContact(True) \
                .setMass(1e-2)


        elif position == 'random':
            C.addFrame(box_name) \
                .setPosition([np.random.uniform(-0.5, 0.5), np.random.uniform(-0.5, 0.5), 0.69]) \
                .setShape(ry.ST.ssBox, size=[0.04, 0.04, 0.12, 0.001]) \
                .setColor(color_map[i % len(color_map)]) \
                .setContact(True) \
                .setMass(1e-2)
    
    return C
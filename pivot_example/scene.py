import rowan
import numpy as np
import robotic as ry


def give_scene() -> ry.Config:
    C = ry.Config()
    C.addFile(ry.raiPath('../rai-robotModels/scenarios/pandaSingle.g'))

    C.addFrame("shelf") \
        .setPosition([.45, -.3, .87]) \
        .setQuaternion(rowan.from_euler(-np.pi*.5, 0, 0))

    C.addFrame("wall_bottom", "shelf") \
        .setShape(ry.ST.ssBox, [.38, .36, .02, .001]) \
        .setRelativePosition([.0, .0, -.19]) \
        .setColor([1., 1., 1.]) \
        .setContact(1)

    C.addFrame("wall_top", "shelf") \
        .setShape(ry.ST.ssBox, [.38, .36, .02, .001]) \
        .setRelativePosition([.0, .0, .19]) \
        .setColor([1., 1., 1.]) \
        .setContact(1)

    C.addFrame("wall_right", "shelf") \
        .setShape(ry.ST.ssBox, [.38, .02, .4, .001]) \
        .setRelativePosition([.0, -.19, .0]) \
        .setColor([1., 1., 1.]) \
        .setContact(1)

    C.addFrame("wall_left", "shelf") \
        .setShape(ry.ST.ssBox, [.38, .02, .4, .001]) \
        .setRelativePosition([.0, .19, .0]) \
        .setColor([1., 1., 1.]) \
        .setContact(1)

    C.addFrame("wall_back", "shelf") \
        .setShape(ry.ST.ssBox, [.005, .36, .36, .001]) \
        .setRelativePosition([.18, .0, .0]) \
        .setColor([.8, .75, .7]) \
        .setContact(1)

    C.addFrame("pre_hinge", "shelf") \
        .setRelativePosition([-.2, -.19, .0])

    C.addFrame("hinge", "pre_hinge") \
        .setJoint(ry.JT.hingeZ)

    C.addFrame("door", "hinge") \
        .setShape(ry.ST.ssBox, [.015, .4, .4, .001]) \
        .setRelativePosition([.0, .19, .0]) \
        .setColor([.75, .75, .75]) \
        .setContact(1)

    C.addFrame("handle", "door") \
        .setShape(ry.ST.ssBox, [.055, .05, .105, .001]) \
        .setRelativePosition([-.0285, .135, .01]) \
        .setColor([1., 0., 0.]) \
        .setContact(1)
    
    return C

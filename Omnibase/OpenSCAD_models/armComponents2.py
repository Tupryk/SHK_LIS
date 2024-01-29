import numpy as np
from solid.objects import cube, translate, union, rotate, color, cylinder


def controlBox():
    box = color("Yellow")(
        cube([.445, .3575, .09], center=True))
    
    longTopBar1 = color("Blue")(
        translate([0, .3575*.5 - .01, -.045-.01])(
            cube([.445+.08, .02, .02], center=True)))
    longTopBar2 = color("Blue")(
        translate([0, .3575*.5 - .01, .045+.01])(
            cube([.445+.08, .02, .02], center=True)))

    shortVerticalBar_r = color("Blue")(
        translate([.445*.5 + .01, .3575*.5 - .01])(
            cube([.02, .02, .09], center=True)))
    shortVerticalBar_l = color("Blue")(
        translate([-(.445*.5 + .01), .3575*.5 - .01])(
            cube([.02, .02, .09], center=True)))

    longVerticalTwin_r = color("Blue")(
        translate([.085+.02, -.03, .045+.01])(
            cube([.02, .3575+.02, .02], center=True)))
    longVerticalTwin_l = color("Blue")(
        translate([-.085-.02, -.03, .045+.01])(
            cube([.02, .3575+.02, .02], center=True)))
    

    longVerticalTwin_rBack = color("Blue")(
        translate([.085+.02, -.03, -.045-.01])(
            cube([.02, .3575+.02, .02], center=True)))
    longVerticalTwin_lBack = color("Blue")(
        translate([-.085-.02, -.03, -.045-.01])(
            cube([.02, .3575+.02, .02], center=True)))

    control_box = union()(box, longTopBar1, longTopBar2, shortVerticalBar_r, shortVerticalBar_l,
                          longVerticalTwin_r, longVerticalTwin_l, longVerticalTwin_rBack, longVerticalTwin_lBack)
    return control_box


def powerInverter():
    inverter = color("Green")(
        translate([-.04, 0, 0])(
            cube([.26, .135, .08], center=True)))

    footBar1 = color("Blue")(
        translate([-.02, -.015, -.04-.01])(
            cube([.26+.04+.04, .02, .02], center=True)))
    footBar2 = color("Blue")(
        translate([-.02, .015, -.04-.01])(
            cube([.26+.04+.04, .02, .02], center=True)))

    return rotate([0, 0, 0])(translate([-.1, -.15, .0])(union()(inverter, footBar1, footBar2)))


def robotBase():
    height = .01+.01+.02
    wheel1Pos = np.array([0.27453005, -0.1585, height])
    wheel2Pos = np.array([-0.27453005, -0.1585, height])
    armPos = (wheel1Pos+wheel2Pos)*.5
    base = color("Orange")(translate(armPos)(
        cube([.21, .21, .04], center=True)))
    return base

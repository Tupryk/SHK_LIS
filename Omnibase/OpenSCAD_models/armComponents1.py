from solid.objects import cube, translate, union, rotate, color, cylinder


def controlBox():
    box = color("Yellow")(
        cube([.445, .3575, .09], center=True))
    longTopBar = color("Blue")(
        translate([0, .3575*.5 - .01, .045+.01])(
            cube([.445+.08, .02, .02], center=True)))

    shortVerticalBar_r = color("Blue")(
        translate([.445*.5 + .01, .3575*.5 - .01, -.01])(
            cube([.02, .02, .09+.02], center=True)))
    shortVerticalBar_l = color("Blue")(
        translate([-(.445*.5 + .01), .3575*.5 - .01, -.01])(
            cube([.02, .02, .09+.02], center=True)))

    shortVerticalBarBack_r = color("Blue")(
        translate([.085, -(.3575*.5 + .01), 0])(
            cube([.02, .02, .09], center=True)))
    shortVerticalBarBack_l = color("Blue")(
        translate([-.085, -(.3575*.5 + .01), 0])(
            cube([.02, .02, .09], center=True)))

    longVerticalTwin_r = color("Blue")(
        translate([.085, -.03, .045+.01])(
            cube([.02, .3575+.02, .02], center=True)))
    longVerticalTwin_l = color("Blue")(
        translate([-.085, -.03, .045+.01])(
            cube([.02, .3575+.02, .02], center=True)))

    control_box = union()(box, longTopBar, shortVerticalBar_r, shortVerticalBar_l,
                          shortVerticalBarBack_r, shortVerticalBarBack_l,
                          longVerticalTwin_r, longVerticalTwin_l)
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

    return union()(inverter, footBar1, footBar2)


def robotBase():
    base = color("Orange")(
        cube([.21, .21, .04], center=True))
    return base

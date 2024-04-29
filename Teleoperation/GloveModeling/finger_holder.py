#! /usr/bin/env python3
import sys
import numpy as np
from solid import scad_render_to_file, difference
from solid.objects import cube, translate, union, rotate, color, cylinder, scale


SEGMENTS = 128

def basic_geometry():

    thickness = 1

    shapes = []
    finger_holder_l = cube([4, 4.5, thickness], center=True)
    tmp = translate([-1.7, 0, 0])(cube([.3, 1.2, 10], center=True))
    finger_holder_l = difference()(finger_holder_l, tmp)

    tmp = translate([1.7, 0, 0])(cube([.3, 1.2, 10], center=True))
    finger_holder_l = translate([-3, 0, 0])(rotate([0, 0, 10])(finger_holder_l))
    mini_hole_l = translate([-3, 0, 0])(rotate([0, 0, 10])(tmp))

    finger_holder_r = cube([4.5, 4.5, thickness], center=True)
    tmp = translate([1.7, 0, 0])(cube([.3, 1.2, 10], center=True))
    finger_holder_r = difference()(finger_holder_r, tmp)

    tmp = translate([-1.7, 0, 0])(cube([.3, 1.2, 10], center=True))
    finger_holder_r = translate([3.25, 0, 0])(rotate([0, 0, -10])(finger_holder_r))
    mini_hole_r = translate([3.25, 0, 0])(rotate([0, 0, -10])(tmp))

    connector = translate([0, .3, 0])(cube([3, 4.6, thickness], center=True))
    hole = cube([1.2, .9, 3], center=True)
    connector = difference()(connector, hole)

    finger_holder = union()(finger_holder_l, finger_holder_r, connector)
    finger_holder = difference()(finger_holder, mini_hole_l)
    finger_holder = difference()(finger_holder, mini_hole_r)
    finger_holder = scale([1, 1, 1.5])(finger_holder)

    curver = translate([0, 0, 24.7])(rotate([90, 0, 0])(cylinder(r=25, h=10, center=True)))
    finger_holder = difference()(finger_holder, curver)
    finger_holder = scale([10, 10, 10])(finger_holder)

    shapes.append(finger_holder)
    return union()(*shapes)

if __name__ == '__main__':
    out_dir = sys.argv[1] if len(sys.argv) > 1 else None

    a = basic_geometry()

    # Adding the file_header argument as shown allows you to change
    # the detail of arcs by changing the SEGMENTS variable.  This can
    # be expensive when making lots of small curves, but is otherwise
    # useful.
    file_out = scad_render_to_file(a, out_dir=out_dir, file_header=f'$fn = {SEGMENTS};')
    print(f"{__file__}: SCAD file written to: \n{file_out}")

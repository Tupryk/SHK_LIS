#! /usr/bin/env python3
import sys
import numpy as np
from solid import scad_render_to_file, difference
from solid.objects import cube, translate, union, rotate, color, cylinder, scale


SEGMENTS = 48

def basic_geometry():

    shapes = []
    thumb_base = scale((1, 1.5, 1))(cylinder(h=.5, r=1.5, center=True))
    tmp = translate([-1.1, 0, 0])(cube([.3, 1.2, 10], center=True))
    thumb_base = difference()(thumb_base, tmp)
    tmp = translate([1.1, 0, 0])(cube([.3, 1.2, 10], center=True))
    thumb_base = difference()(thumb_base, tmp)

    angle = rotate((0, 90, 0))(cylinder(h=1, r=10, center=True))
    tmp = rotate((0, 90, 0))(cylinder(h=100, r=9.3, center=True))
    angle = difference()(angle, tmp)
    tmp = translate([0, -3, 0])(cube([10, 15, 100], center=True))
    angle = difference()(angle, tmp)
    tmp = translate([0, 5, -5])(cube([100, 100, 10], center=True))
    angle = difference()(angle, tmp)
    angle = translate([0, -9.65, 0])(angle)

    thumb_base = union()(thumb_base, angle)

    thumb_base = scale([10, 10, 10])(thumb_base)
    shapes.append(thumb_base)
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

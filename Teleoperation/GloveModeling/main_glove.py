#! /usr/bin/env python3
import sys
import numpy as np
from solid import scad_render_to_file, difference
from solid.objects import cube, translate, union, rotate, color, cylinder, scale


SEGMENTS = 48

def basic_geometry():

    shapes = []
    base = cube([9.5, 7.5, 2], center=True)
    cut0 = rotate([0, 0, 3])(translate([-5, 0, 0])(cube([2, 10, 3], center=True)))
    base = difference()(base, cut0)
    cyl = translate([0, 0, -8.5])(rotate([90, 0, 0])(cylinder(r=9, h=10, center=True)))
    cyl2 = translate([0, 0, -8.5])(rotate([90, 0, 0])(cylinder(r=9.7, h=11, center=True)))
    cyl3 = translate([0, 0, -8.5])(rotate([90, 0, 0])(cylinder(r=12, h=10, center=True)))
    cyl3 = difference()(cyl3, cyl2)
    base = difference()(base, cyl3)
    base = difference()(base, cyl)
    cyl_thumb = translate([4.25, -1.5, -5])(rotate([90, 0, 90])(cylinder(r=3.75*.5+3+.7, h=10, center=True)))
    base = difference()(base, cyl_thumb)

    cyl2 = translate([0, -5.7, 0])(cylinder(r=10, h=11, center=True))
    cyl3 = translate([0, -5.7, 0])(cylinder(r=11, h=10, center=True))
    cyl3 = difference()(cyl3, cyl2)
    base = difference()(base, cyl3)

    cyl2 = translate([0, 5.7, 0])(cylinder(r=10, h=11, center=True))
    cyl3 = translate([0, 5.7, 0])(cylinder(r=11, h=10, center=True))
    cyl3 = difference()(cyl3, cyl2)
    base = difference()(base, cyl3)

    base = scale([1.1, 1, 1])(base)

    strap_hole = translate([4.8, 2, 0])(cube([.3, 1.2, 10], center=True))
    base = difference()(base, strap_hole)
    strap_hole = translate([-4, 2, 0])(cube([.3, 1.2, 10], center=True))
    base = difference()(base, strap_hole)

    point_place = [
        translate([-2.5, 3, .75])(cylinder(r=1.1*.5, h=.5, center=True)),
        translate([2.5, 3, .75])(cylinder(r=1.1*.5, h=.5, center=True)),
        translate([2.5, -3, .75])(cylinder(r=1.1*.5, h=.5, center=True)),
        translate([-2.5, -3, .75])(cylinder(r=1.1*.5, h=.5, center=True))]

    base = union()(base, *point_place)
    base = scale([10, 10, 10])(base)
    shapes.append(base)
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

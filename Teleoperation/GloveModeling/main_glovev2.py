#! /usr/bin/env python3
import sys
import numpy as np
from solid import scad_render_to_file, difference
from solid.objects import cube, translate, union, rotate, color, cylinder, scale


SEGMENTS = 48

def basic_geometry():

    shapes = []
    base = cube([104, 75, 25], center=True)
    cutter = rotate([-20, -20, 0])(translate([-20, 20, 45])(cube([1000, 7500, 50], center=True)))
    base = difference()(base, cutter)
    base_cutter = scale([1.1, 1.1, 1])(translate([0, 0, -8])(base))
    base = difference()(base, base_cutter)

    cyl2 = translate([0, -57, 0])(cylinder(r=100, h=110, center=True))
    cyl3 = translate([0, -57, 0])(cylinder(r=110, h=100, center=True))
    cyl3 = difference()(cyl3, cyl2)
    base = difference()(base, cyl3)

    cyl2 = translate([0, 57, 0])(cylinder(r=100, h=110, center=True))
    cyl3 = translate([0, 57, 0])(cylinder(r=110, h=100, center=True))
    cyl3 = difference()(cyl3, cyl2)
    base = difference()(base, cyl3)

    strap_hole = translate([48, -20, 0])(cube([3, 12, 100], center=True))
    base = difference()(base, strap_hole)
    strap_hole = translate([-48, -20, 0])(cube([3, 12, 100], center=True))
    base = difference()(base, strap_hole)

    under_cyl = translate([0, 0, -190])(rotate([90, 0, 0])(cylinder(r=200, h=100, center=True)))
    block_cyl = rotate([-20, -20, 0])(translate([-20, 20, 40])(cube([1000, 7500, 50], center=True)))
    under_cyl = difference()(under_cyl, block_cyl)
    base = difference()(base, under_cyl)

    shapes.append(base)
    #shapes.append(under_cyl)
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

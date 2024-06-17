#! /usr/bin/env python3
import sys
import numpy as np
from solid import scad_render_to_file, difference
from solid.objects import translate, union, rotate, cylinder, scale


SEGMENTS = 48
ball_count = 6

def basic_geometry():

    shapes = []
    base = cylinder(r=1.5, h=.7, center=True)
    
    step = np.pi*2/ball_count
    for i in range(ball_count):
        length = 2.
        inclination = 15. + np.random.random() * 10.
        pos = [np.sin(i*step)*.8, np.cos(i*step)*.8, length*.5+.25]
        rot = [-np.cos(i*step)*inclination, np.sin(i*step)*inclination, 0.]

        bar = cylinder(r=.4, h=length, center=True)
        bar_hole = cylinder(r=.15, h=length*2., center=True)
        bar = difference()(bar, bar_hole)
        bar = translate(pos)(bar)
        bar = rotate(rot)(bar)

        base = union()(base, bar)

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

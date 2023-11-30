#! /usr/bin/env python3
import sys

from solid import scad_render_to_file
from solid.objects import cube, cylinder, difference, translate, union, rotate
from solid.utils import right
import numpy as np
SEGMENTS = 48


small1_transy = 35
wheel_dist = 10

def basic_geometry():
    # SolidPython code can look a lot like OpenSCAD code.  It also has
    # some syntactic sugar built in that can make it look more pythonic.
    # Here are two identical pieces of geometry, one left and one right.

    # left_piece uses standard OpenSCAD grammar (note the commas between
    # block elements; OpenSCAD doesn't require this)
    big_piece1 = union()(
            translate((0, -33, 0))(
            rotate((0, 0 ,0))(
                    cube([46, 2, 4], center=True)
                    )
            ),
            
            
    )
    big_piece2 = union()(
            translate((27, 15, 0))(
            rotate((0, 0 ,-60))(
                    cube([46, 2, 4], center=True)
                    )
            ),
            
            
    )
    big_piece3 = union()(
            translate((-27, 15, 0))(
            rotate((0, 0 ,60))(
                    cube([46, 2, 4], center=True)
                    )
            ),
            
            
    )

    # Right piece uses a more Pythonic grammar.  + (plus) is equivalent to union(), 
    # - (minus) is equivalent to difference() and * (star) is equivalent to intersection
    # solid.utils also defines up(), down(), left(), right(), forward(), and back()
    # for common transforms.

    small_piece1 = union()(
            translate((0, 35, 0))(
            rotate((0, 0 ,0))(
                    cube([30, 2, 4], center=True)
                    )
            ),
            
            
    )
    small_piece2 = union()(
            translate((31,-19, 0))(
            rotate((0, 0 ,60))(
                    cube([30, 2, 4], center=True)
                    )
            ),
            
    )
    small_piece3 = union()(
            translate((-31,-19, 0))(
            rotate((0, 0 ,-60))(
                    cube([30, 2, 4], center=True)
                    )
            ),
            
    )
    small_middlepiece1 = union()(
            translate((0, small1_transy-wheel_dist, 0))(
            rotate((0, 0 ,0))(
                    cube([39, 2, 4], center=True)
                    )
            ),
            
            
    )
    small_middlepiece2 = union()(
            translate((31-wheel_dist,-14, 0))(
            rotate((0, 0 ,60))(
                    cube([41, 2, 4], center=True)
                    )
            ),
            
    )
    small_middlepiece3 = union()(
            translate((-31+wheel_dist,-14, 0))(
            rotate((0, 0 ,-60))(
                    cube([41, 2, 4], center=True)
                    )
            ),
            
    )
    middlepiecevert1 = union()(
            translate((7.5,-3, -1))(
            rotate((0, 0 ,90))(
                    cube([57, 2, 2], center=True)
                    )
            ),
            
    )
    middlepiecevert2 = union()(
            translate((-7.5,-3, -1))(
            rotate((0, 0 ,90))(
                    cube([57, 2, 2], center=True)
                    )
            ),
            
    )
    middlepiecehor1 = union()(
            translate((0,7.5, 1))(
            rotate((0, 0 ,0))(
                    cube([60, 2, 2], center=True)
                    )
            ),
            
    )
    middlepiecehor2 = union()(
            translate((0,-7.5, 1))(
            rotate((0, 0 ,0))(
                    cube([46, 2, 2], center=True)
                    )
            ),
            
    )


   
    return union()(big_piece1, big_piece2, big_piece3, small_piece1, small_piece2, small_piece3,small_middlepiece1, small_middlepiece2, small_middlepiece3, middlepiecevert1, middlepiecevert2, middlepiecehor1, middlepiecehor2)


if __name__ == '__main__':
    out_dir = sys.argv[1] if len(sys.argv) > 1 else None

    a = basic_geometry()

    # Adding the file_header argument as shown allows you to change
    # the detail of arcs by changing the SEGMENTS variable.  This can
    # be expensive when making lots of small curves, but is otherwise
    # useful.
    file_out = scad_render_to_file(a, out_dir=out_dir, file_header=f'$fn = {SEGMENTS};')
    print(f"{__file__}: SCAD file written to: \n{file_out}")

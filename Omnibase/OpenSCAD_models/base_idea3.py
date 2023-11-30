#! /usr/bin/env python3
import sys
import numpy as np
from solid import scad_render_to_file
from solid.objects import cube, translate, union, rotate, color

SEGMENTS = 48

radial_vector0 = np.array([ 31., -19., 0.])
radial_vector0 /= np.linalg.norm(radial_vector0)
radial_vector1 = np.array([-31., -19., 0.])
radial_vector1 /= np.linalg.norm(radial_vector1)

side_count = 3
step_angle = 360/side_count
step_angle_rad = np.deg2rad(step_angle)
rad_size = 30 # Length from center to outer wheel
piece_width = 2 # cm
hpw = piece_width*.5

wheel_gap = 9
ctiw = 44 # Center to inner wheel support
ctow = wheel_gap+ctiw # Center to outer wheel support
wsl = 30 # Wheel support length

p1 = np.array([wsl*.5, ctiw, 0.])
p2 = np.array([np.sin(step_angle_rad), np.cos(step_angle_rad), 0.]) * ctiw - np.array([np.sin(step_angle_rad+np.pi*.5), np.cos(step_angle_rad+np.pi*.5), 0.]) * wsl*.5
boll = np.linalg.norm(p1-p2) # Big outer lines length
bolr = np.linalg.norm((p1+p2)*.5)# Big outer lines rad length

# Circular inner support
p3 = (p1+p2)*.5
p2 = np.array([-np.sin(step_angle_rad), np.cos(step_angle_rad), 0.]) * ctiw - np.array([-np.sin(step_angle_rad+np.pi*.5), np.cos(step_angle_rad+np.pi*.5), 0.]) * wsl*.5
p1 = np.array([-wsl*.5, ctiw, 0.])
p4 = (p1+p2)*.5
cil = np.linalg.norm(p3-p4) # Big outer lines length
cir = np.linalg.norm((p3+p4)*.5)# Big outer lines rad length

# Radial connections
p1 = np.array([0, ctiw, 0.])
p2 = (p4+p3)*.5
rcl = np.linalg.norm(p1-p2) - piece_width
rcr = np.linalg.norm((p1+p2)*.5)

# Wheel side support
wssl = wheel_gap - piece_width

def rotate_vec(vec, angle):
    angle = np.deg2rad(angle)
    new_vec = np.copy(vec)
    new_vec[0] = vec[0] * np.cos(angle) - vec[1] * np.sin(angle)
    new_vec[1] = vec[0] * np.sin(angle) + vec[1] * np.cos(angle)
    return new_vec

def basic_geometry():

    shapes = []
    for i in range(side_count):
        dir_vector = np.array([np.sin(i*step_angle_rad), np.cos(i*step_angle_rad), 0.])
        rot = -step_angle*i
        outer_wheel_support = union()(
                                color("Red")(
                                    translate(dir_vector*(ctow+hpw))(
                                        rotate((0, 0, rot))(
                                            cube([wsl, 2, 4], center=True)))))
        shapes.append(outer_wheel_support)
        inner_wheel_support = union()(
                                color("Orange")(
                                    translate(dir_vector*(ctiw+hpw))(
                                        rotate((0, 0, rot))(
                                            cube([wsl, 2, 4], center=True)))))
        shapes.append(inner_wheel_support)

        inner_circle = union()(
                            color("Yellow")(
                                translate(dir_vector*(cir+hpw))(
                                    rotate((0, 0, rot))(
                                        cube([cil, 2, 4], center=True)))))
        shapes.append(inner_circle)

        radial_support = union()(
                            color("Green")(
                                translate(dir_vector*(rcr+hpw))(
                                    rotate((0, 0, rot+90))(
                                        cube([rcl, 2, 4], center=True)))))
        shapes.append(radial_support)

        wheel_side_support_l = union()(
                                    color("Blue")(
                                        translate(dir_vector*(ctiw+hpw)+rotate_vec(np.array([-wsl*.5+hpw, wheel_gap*.5, 0.]), rot))(
                                            rotate((0, 0, rot+90))(
                                                cube([wssl, 2, 4], center=True)))))
        wheel_side_support_r = union()(
                                    color("Blue")(
                                        translate(dir_vector*(ctiw+hpw)+rotate_vec(np.array([wsl*.5-hpw, wheel_gap*.5, 0.]), rot))(
                                            rotate((0, 0, rot+90))(
                                                cube([wssl, 2, 4], center=True)))))
        shapes.append(wheel_side_support_l)
        shapes.append(wheel_side_support_r)

        dir_vector = np.array([np.sin(i*step_angle_rad+step_angle_rad*.5), np.cos(i*step_angle_rad+step_angle_rad*.5), 0.])
        rot -= step_angle*.5
        big_outer_line = union()(
                            color("Purple")(
                                translate(dir_vector*(bolr+hpw))(
                                    rotate((0, 0, rot))(
                                        cube([boll, 2, 4], center=True)))))
        shapes.append(big_outer_line)

    print(f"Red:         {wsl:.1f} cm")
    print(f"Orange:      {wsl:.1f} cm")
    print(f"Yellow:      {cil:.1f} cm - angle cut")
    print(f"Green:       {rcl:.1f} cm")
    print(f"Blue:         {wssl:.1f} cm")
    print(f"Purple:      {boll:.1f} cm")
    print("--------------------------")
    print(f"Diameter:   {(np.linalg.norm(np.array([wsl*.5, ctow]))*2):.1f} cm")

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

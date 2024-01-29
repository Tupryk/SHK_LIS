#! /usr/bin/env python3
import sys
import numpy as np
from solid import scad_render_to_file
from solid.objects import cube, translate, union, rotate, color, cylinder
from armComponents2 import controlBox, powerInverter, robotBase

SEGMENTS = 48

radial_vector0 = np.array([31., -19., 0.])
radial_vector0 /= np.linalg.norm(radial_vector0)
radial_vector1 = np.array([-31., -19., 0.])
radial_vector1 /= np.linalg.norm(radial_vector1)

side_count = 3
step_angle = 360/side_count
step_angle_rad = np.deg2rad(step_angle)
rad_size = .3  # Length from center to outer wheel
piece_width = .02  # cm
hpw = piece_width*.5

wheel_gap = .09
ctiw = .352  # Center to inner wheel support
ctow = ctiw-wheel_gap  # Center to outer wheel support
wsl = .3  # Wheel support length

p1 = np.array([wsl*.5, ctiw, 0.])
p2 = np.array([np.sin(step_angle_rad), np.cos(step_angle_rad), 0.]) * ctiw - \
    np.array([np.sin(step_angle_rad+np.pi*.5),
             np.cos(step_angle_rad+np.pi*.5), 0.]) * wsl*.5
boll = np.linalg.norm(p1-p2)  # Big outer lines length
bolr = np.linalg.norm((p1+p2)*.5)  # Big outer lines rad length

# Inner inner wheel support
iiwsl = wsl+((wheel_gap+piece_width*.5) *
             np.tan((side_count*2-2)*np.pi/(side_count*2)-np.pi*.5))*2
print(iiwsl)


def rotate_vec(vec, angle):
    angle = np.deg2rad(angle)
    new_vec = np.copy(vec)
    new_vec[0] = vec[0] * np.cos(angle) - vec[1] * np.sin(angle)
    new_vec[1] = vec[0] * np.sin(angle) + vec[1] * np.cos(angle)
    return new_vec


def basic_geometry():

    shapes = []
    for i in range(side_count):
        dir_vector = np.array(
            [np.sin(i*step_angle_rad), np.cos(i*step_angle_rad), 0.])
        rot = -step_angle*i
        outer_wheel_support = union()(
            color("Red")(
                translate(dir_vector*(ctow+hpw))(
                    rotate((0, 0, rot))(
                        cube([iiwsl, piece_width, piece_width*2], center=True)))))
        shapes.append(outer_wheel_support)
        inner_wheel_support = union()(
            color("Orange")(
                translate(dir_vector*(ctiw+hpw))(
                    rotate((0, 0, rot))(
                        cube([wsl, piece_width, piece_width*2], center=True)))))
        shapes.append(inner_wheel_support)

        # Wheels
        wheel_pos = ((dir_vector*(ctow+hpw))+(dir_vector*(ctiw+hpw)))*.5
        wheel_pos[2] -= .04
        print(wheel_pos)
        wheel = union()(
            color("Green")(
                translate(wheel_pos)(
                    rotate((90, 0, rot))(
                        cylinder(r=.06, h=.05, center=True)))))
        shapes.append(wheel)

        dir_vector = np.array([np.sin(i*step_angle_rad+step_angle_rad*.5),
                              np.cos(i*step_angle_rad+step_angle_rad*.5), 0.])
        rot -= step_angle*.5
        big_outer_line = union()(
            color("Purple")(
                translate(dir_vector*(bolr+hpw))(
                    rotate((0, 0, rot))(
                        cube([boll, piece_width, piece_width*2], center=True)))))
        shapes.append(big_outer_line)

    vertical_bar1 = union()(
        color("Blue")(
            translate([-.085, -.02, .01])(
                rotate((0, 0, -90))(
                    cube([.568, piece_width, piece_width], center=True)))))
    shapes.append(vertical_bar1)

    vertical_bar2 = union()(
        color("Blue")(
            translate([.085, -.02, .01])(
                rotate((0, 0, -90))(
                    cube([.568, piece_width, piece_width], center=True)))))
    shapes.append(vertical_bar2)

    horizontal_bar = union()(
        color("Blue")(
            translate([0, .1, -.01])(
                cube([.6, piece_width, piece_width], center=True))))
    shapes.append(horizontal_bar)

    battery_pack = union()(
        color("Yellow")(
            translate([.19, .06, .1])(
                    cube([.18, .135, .2], center=True))))
    shapes.append(battery_pack)
    
    power_inverter = powerInverter()
    power_inverter = translate([-.055, .195, .02+.06])(power_inverter)
    shapes.append(power_inverter)
    
    control_box = controlBox()
    control_box = translate([0, .195, .02+.3575*.5])(rotate([90, 0, 0])(control_box))
    shapes.append(control_box)

    robot_base = robotBase()
    shapes.append(robot_base)

    return union()(rotate([0, 0, 60])(*shapes))


if __name__ == '__main__':
    out_dir = sys.argv[1] if len(sys.argv) > 1 else None

    a = basic_geometry()

    # Adding the file_header argument as shown allows you to change
    # the detail of arcs by changing the SEGMENTS variable.  This can
    # be expensive when making lots of small curves, but is otherwise
    # useful.
    file_out = scad_render_to_file(
        a, out_dir=out_dir, file_header=f'$fn = {SEGMENTS};')
    print(f"{__file__}: SCAD file written to: \n{file_out}")

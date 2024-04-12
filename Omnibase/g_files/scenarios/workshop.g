world: {}

### cage

floor (world): { shape: ssBox, Q: "t(0 0. .05)", size: [4.5, 6.5, .1, .02], color: [.3, .3, .3] }
#roof (world): { shape: ssBox, Q: "t(-.5 -1. 3.)", size: [3., 4.5, .1, .02], color: [.9, .9, .9, .2] }
wall (world): { shape: ssBox, Q: "t(0 -3. 1.)", size: [4.1, .1, 2., .02], color: [.3, .3, .3] }
back (world): { shape: ssBox, Q: "t(-2. 0 1.)", size: [.1, 6.1, 2., .02], color: [.3, .3, .3] }
#right(world): { shape: ssBox, Q: "t(0 3. 1.)", size: [4.1, .1, 2., .02], color: [.9, .9, .9, .2] }
# front(world): { shape: ssBox, Q: "t(2. 0 1.5)", size: [.1, 6.1, 3., .02], color: [.9, .9, .9, .2] }
#right(world): { shape: ssBox, Q: "t(0 1.75 1.)", size: [4.1, .1, 2., .02], color: [.9, .9, .9, .2] }


### two pandas

Prefix: "l_"
Include: <panda_mobile.g>

Prefix: "r_"
Include: <panda_mobile.g>

Prefix!

Edit l_base_origin (world): { Q: [-.4, -.4, .15] }
Edit r_base_origin (world): { Q: [.4, -.4, .15] }


### tables
table1 (world): { shape: ssBox, Q: [-1.5, 0., .5], size: [.8, 6., .1, .02], color: [.3, .3, .3], friction: .1, logical:{table} }
table2 (world): { shape: ssBox, Q: [+1.5 0. .5], size: [.8, 6., .1, .02], color: [.3, .3, .3], friction: .1, logical:{table} }
#table3 (world): { shape: ssBox, Q: [0, -2.5, .5], size: [2., .8, .1, .02], color: [.3, .3, .3], friction: .1, logical:{table} }

 
block1 (table2): { joint: rigid, shape: ssBox, Q: [0, 0, .095, 1, 0, 0, 1], size: [.06, .15, .09, .01], color: [.6, .6, .6] }
block2 (table2): { joint: rigid, shape: ssBox, Q: [0, -.2, .095, 1, 0, 0, 1], size: [.06, .15, .09, .01], color: [.6, .6, .6] }
block3 (table2): { joint: rigid, shape: ssBox, Q: [0, -.4, .095, 1, 0, 0, 1], size: [.06, .15, .09, .01], color: [.6, .6, .6] }
block4 (table2): { joint: rigid, shape: ssBox, Q: [0, -.6, .095, 1, 0, 0, 1], size: [.06, .15, .09, .01], color: [.6, .6, .6] }
block5 (table2): { joint: rigid, shape: ssBox, Q: [0, -.8, .095, 1, 0, 0, 1], size: [.06, .15, .09, .01], color: [.6, .6, .6] }
block6 (table2): { joint: rigid, shape: ssBox, Q: [0, -1., .095, 1, 0, 0, 1], size: [.06, .15, .09, .01], color: [.6, .6, .6] }

board1 (table2): { joint: rigid, shape: ssBox, Q: [-.25, -2.4, .095], size: [.5, .15, .03, .01], color: [.6, .6, .6] }
board2 (table2): { joint: rigid, shape: ssBox, Q: [-.25, -2.2, .095], size: [.5, .15, .03, .01], color: [.6, .6, .6] }
board3 (table2): { joint: rigid, shape: ssBox, Q: [-.25, -2, .095], size: [.5, .15, .03, .01], color: [.6, .6, .6] }
board4 (table2): { joint: rigid, shape: ssBox, Q: [-.25, -1.8, .095], size: [.5, .15, .03, .01], color: [.6, .6, .6] }
board5 (table2): { joint: rigid, shape: ssBox, Q: [-.25, -1.6, .095], size: [.5, .15, .03, .01], color: [.6, .6, .6] }
board6 (table2): { joint: rigid, shape: ssBox, Q: [-.25, -1.4, .095], size: [.5, .15, .03, .01], color: [.6, .6, .6] }

stick1 (table1): { joint: rigid, shape: capsule, Q: "t(.25 -2.4 .07) d(90 0 1 0) ", size: [.5, .02], color: [.6, .6, .6] }
stick2 (table1): { joint: rigid, shape: capsule, Q: "t(.25 -2.2 .07) d(90 0 1 0) ", size: [.5, .02], color: [.6, .6, .6] }
stick3 (table1): { joint: rigid, shape: capsule, Q: "t(.25 -2. .07) d(90 0 1 0) ", size: [.5, .02], color: [.6, .6, .6] }
stick4 (table1): { joint: rigid, shape: capsule, Q: "t(.25 -1.4 .07) d(45 0 0 1) d(90 0 1 0) ", size: [.8, .02], color: [.6, .6, .6] }
stick5 (table1): { joint: rigid, shape: capsule, Q: "t(.25 -1.2 .07) d(45 0 0 1) d(90 0 1 0) ", size: [.8, .02], color: [.6, .6, .6] }
stick6 (table1): { joint: rigid, shape: capsule, Q: "t(.25 -1. .07) d(45 0 0 1) d(90 0 1 0) ", size: [.8, .02], color: [.6, .6, .6] }

Include: <ringBox.g>
Edit box (table1): { joint: rigid, Q: "t(0 0 .15) d(45 0 0 1)" }
 
box1 (floor): {
 shape: ssBox, Q: [-.5, 1, .35], size: [.4, .2, .6, .02], color: [.6, .6, .6]
 joint: rigid
 friction: .1
}

box2 (floor): {
 shape: ssBox, Q: [.5, 1.6, .25], size: [1., .2, .4, .02], color: [.6, .6, .6]
 joint: rigid
 friction: .1
}


ball1 (table1): { joint: rigid, Q: [.0, 1., .1], shape: sphere, size: [.05], color: [1, 1, .6] }

ball2 (table2): { joint: rigid, Q: [0, 2.5, .1], shape: sphere, size: [.05], color: [1, 1, .6] }

ball3 (floor): { joint: rigid, Q: [.5, 2.8, .1], shape: sphere, size: [.05], color: [1, 1, .6] }

ball4 (floor): { joint: rigid, Q: [-.5, 2.2, .1], shape: sphere, size: [.05], color: [1, 1, .6] }
 
drawer (floor): { Q: [-.5, 2.2, .1] }
drawerA (drawer): { shape: ssBox, size: [.6, .05, .1, .02], Q: [0, -.18, 0], color: [.6, .6, .6] }
drawerB (drawer): { shape: ssBox, size: [.05, .4, .1, .02], Q: [-.28, 0, 0], color: [.6, .6, .6] }
drawerC (drawer): { shape: ssBox, size: [.05, .4, .1, .02], Q: [.28, 0, 0], color: [.6, .6, .6] }

# (floor): { shape: capsule, Q: "t(2. 1.75 .8)", size: [1.6, .02], color: [.6, .6, .6] }
# (floor): { shape: capsule, Q: "t(1.6 1.75 .8)", size: [1.6, .02], color: [.6, .6, .6] }
# (floor): { shape: capsule, Q: "t(1.2 1.75 .8)", size: [1.6, .02], color: [.6, .6, .6] }
(floor): { shape: capsule, Q: "t(0.8 1.75 .8)", size: [1.6, .02], color: [.6, .6, .6] }
(floor): { shape: capsule, Q: "t(0.4 1.75 .8)", size: [1.6, .02], color: [.6, .6, .6] }
(floor): { shape: capsule, Q: "t(0.0 1.75 .8)", size: [1.6, .02], color: [.6, .6, .6] }
(floor): { shape: capsule, Q: "t(-0.4 1.75 .8)", size: [1.6, .02], color: [.6, .6, .6] }
(floor): { shape: capsule, Q: "t(-0.8 1.75 .8)", size: [1.6, .02], color: [.6, .6, .6] }
# (floor): { shape: capsule, Q: "t(-1.2 1.75 .8)", size: [1.6, .02], color: [.6, .6, .6] }
# (floor): { shape: capsule, Q: "t(-1.6 1.75 .8)", size: [1.6, .02], color: [.6, .6, .6] }
# (floor): { shape: capsule, Q: "t(-2.0 1.75 .8)", size: [1.6, .02], color: [.6, .6, .6] }

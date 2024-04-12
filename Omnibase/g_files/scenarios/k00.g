world: {}

floor (world): { shape: ssBox, Q: "t(0 0. .05)", size: [4.5, 6.5, .1, .02], color: [.3, .3, .3] }
roof (world): { shape: ssBox, Q: "t(-.5 -1. 3.)", size: [3., 4.5, .1, .02], color: [.9, .9, .9, .2] }
wall (world): { shape: ssBox, Q: "t(0 -3. 1.5)", size: [4.1, .1, 3., .02], color: [.3, .3, .3] }
back (world): { shape: ssBox, Q: "t(-2. 0 1.5)", size: [.1, 6.1, 3., .02], color: [.3, .3, .3] }
#right(world): { shape: ssBox, Q: "t(0 3. 1.5)", size: [4.1, .1, 3., .02], color: [.9, .9, .9, .2] }
# front(world): { shape: ssBox, Q: "t(2. 0 1.5)", size: [.1, 6.1, 3., .02], color: [.9, .9, .9, .2] }
#right(world): { shape: ssBox, Q: "t(0 1.75 1.)", size: [4.1, .1, 2., .02], color: [.9, .9, .9, .2] }


Include: <../conic/conic.g>
Edit handA(floor): { joint: rigid, Q: "t(-.6 .5 .05) d(90 0 0 1)" }

hooverJoint(floor): { joint: transXY }
hoover(hooverJoint): { Q: [0, 0, .15], shape: ssBox, size: [.4, .4, .2, .02], color: [.9, .9, .9] }

box1 (floor): {
 shape: ssBox, Q: "t(1. -2.5 .35) d(20 0 0 1) d(90 0 0 1)", size: [.2, .4, .6, .02], color: [.6, .6, .6]
 joint: rigid
 friction: .1
}

box2 (floor): {
 shape: ssBox, Q: "t(.0 -2.5 .25) d(20 0 0 1) d(90 1 0 0)", size: [.2, .4, .6, .02], color: [.6, .6, .6]
 joint: rigid
 friction: .1
}

box3 (floor): {
 shape: ssBox, Q: "t(-1. -2.5 .15) d(-20 0 0 1) d(90 0 1 0)", size: [.2, .4, .6, .02], color: [.6, .6, .6]
 joint: rigid
 friction: .1
}

box4 (box3): {
 shape: ssBox, Q: "d(-90 0 1 0) t(0 0 .4) d(30 0 0 1)", size: [.2, .4, .6, .02], color: [.6, .6, .6]
 joint: rigid
 friction: .1
}

box5 (floor): {
 shape: ssBox, Q: [1.5, 1.6, .25], size: [1., .2, .4, .02], color: [.6, .6, .6]
 joint: rigid
 friction: .1
}

stick1 (floor): {
 shape: capsule, Q: "t(-1.5 -1.7 .07) d(20 0 0 1) d(90 1 0 0) ", size: [.8, .02], color: [.6, .6, .6]
 joint: rigid
 friction: .1
 }

stick2 (floor): {
 shape: capsule, Q: "t(-1. -.5 .07) d(10 0 0 1) d(90 1 0 0) ", size: [.8, .02], color: [.6, .6, .6]
 joint: rigid
 friction: .1
 }

pend1anchor (roof): { Q: "t(.5 -.5 0)" }
pend1joint (pend1anchor): { joint: quatBall }
pend1(pend1joint): { Q: "t(0 0 -.8)", shape: capsule, size: [1.6, .02], color: [.6, .6, .6]
}

pend2anchor (roof): { Q: "t(1.3 1 0)" }
pend2joint (pend2anchor): { joint: quatBall, ctrl_H: 0.01, q: [1.,.5,0.2,0.] }
pend2(pend2joint): { Q: "t(0 0 -.6)", shape: capsule, size: [1.2, .02], color: [.6, .6, .6]
}

pend3anchor (roof): { Q: "t(0 2 0)" }
pend3joint (pend3anchor): { joint: quatBall }
pend3(pend3joint): { Q: "t(0 0 -.6)", shape: capsule, size: [1.2, .02], color: [.6, .6, .6]
}

ball0 (floor): { joint: rigid, Q: [.5, -.2, .1], shape: sphere, size: [.05], color: [1, 1, .6] }

ball1 (pend3): { joint: rigid, Q: [0, 0, -.6], shape: sphere, size: [.05], color: [1, 1, .6] }

ball2 (floor): { joint: rigid, Q: [1.5, 2.2, .1], shape: sphere, size: [.05], color: [1, 1, .6] }

ball3 (floor): { joint: rigid, Q: [0, 2.8, .1], shape: sphere, size: [.05], color: [1, 1, .6] }

ball4 (floor): { joint: rigid, Q: [-1.5, 2.2, .1], shape: sphere, size: [.05], color: [1, 1, .6] }
 
drawer (floor): { Q: [-1.5, 2.2, .1] }
drawerA (drawer): { shape: ssBox, size: [.6, .05, .1, .02], Q: [0, -.18, 0], color: [.6, .6, .6] }
drawerB (drawer): { shape: ssBox, size: [.05, .4, .1, .02], Q: [-.28, 0, 0], color: [.6, .6, .6] }
drawerC (drawer): { shape: ssBox, size: [.05, .4, .1, .02], Q: [.28, 0, 0], color: [.6, .6, .6] }

(floor): { shape: capsule, Q: "t(2. 1.75 .8)", size: [1.6, .02], color: [.6, .6, .6] }
(floor): { shape: capsule, Q: "t(1.6 1.75 .8)", size: [1.6, .02], color: [.6, .6, .6] }
(floor): { shape: capsule, Q: "t(1.2 1.75 .8)", size: [1.6, .02], color: [.6, .6, .6] }
(floor): { shape: capsule, Q: "t(0.8 1.75 .8)", size: [1.6, .02], color: [.6, .6, .6] }
(floor): { shape: capsule, Q: "t(0.4 1.75 .8)", size: [1.6, .02], color: [.6, .6, .6] }
(floor): { shape: capsule, Q: "t(0.0 1.75 .8)", size: [1.6, .02], color: [.6, .6, .6] }
(floor): { shape: capsule, Q: "t(-0.4 1.75 .8)", size: [1.6, .02], color: [.6, .6, .6] }
(floor): { shape: capsule, Q: "t(-0.8 1.75 .8)", size: [1.6, .02], color: [.6, .6, .6] }
(floor): { shape: capsule, Q: "t(-1.2 1.75 .8)", size: [1.6, .02], color: [.6, .6, .6] }
(floor): { shape: capsule, Q: "t(-1.6 1.75 .8)", size: [1.6, .02], color: [.6, .6, .6] }
(floor): { shape: capsule, Q: "t(-2.0 1.75 .8)", size: [1.6, .02], color: [.6, .6, .6] }

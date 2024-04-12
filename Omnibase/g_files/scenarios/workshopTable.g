Include: <pandasTable.g>

# add E.T. hooks
l_hookA (l_panda_joint5): { Q: [0, .2, 0], shape: sphere, size: [.025], color: [.6, .9, .9] }
(l_hookA): { Q: [0, -.05, 0, 1, 1, 0, 0], shape: capsule, size: [.075, .01], color: [.9, .9, .9] }

l_hookB (l_gripper): { Q: [.13, 0, .15], shape: sphere, size: [.025], color: [.6, .9, .9] }
(l_hookB): { Q: [-.05, 0, 0, 1, 0, 1, 0], shape: capsule, size: [.075, .01], color: [.9, .9, .9] }

railA(table): { shape: ssBox, size: [3.05, .05, .1, .02], Q: [0, 1., .1], color: [.6, .6, .6], contact }
railB(table): { shape: ssBox, size: [.05, 2.05, .1, .02], Q: [-1.5, 0, .1], color: [.6, .6, .6], contact }
railC(table): { shape: ssBox, size: [.05, 2.05, .1, .02], Q: [1.5, 0, .1], color: [.6, .6, .6], contact }

box1 (table): { joint: rigid, shape: ssBox, Q: "t(-.2 .6 .25) d(-45 0 0 1)", size: [.6, .2, .4, .02], color: [.6, .6, .6], contact }
box2 (table): { joint: rigid, shape: ssBox, Q: [-1.3, -.5, .25], size: [.2, 1., .4, .02], color: [.6, .6, .6], contact }
cyl1 (table): { joint: rigid, shape: ssCylinder, Q: [.2, .7, .13], size: [.16, .16, .02], color: [.6, .6, .6], contact }
cyl2 (table): { joint: rigid, shape: ssCylinder, Q: [-.6, .5, .13], size: [.16, .16, .02], color: [.6, .6, .6], contact }
 
block1 (table): { joint: rigid, shape: ssBox, Q: [1, .2, .095, 1, 0, 0, 1], size: [.06, .15, .09, .01], color: [.6, .6, .6], contact }
block2 (table): { joint: rigid, shape: ssBox, Q: [1, 0, .095, 1, 0, 0, 1], size: [.06, .15, .09, .01], color: [.6, .6, .6], contact }
block3 (table): { joint: rigid, shape: ssBox, Q: [1, -.2, .095, 1, 0, 0, 1], size: [.06, .15, .09, .01], color: [.6, .6, .6], contact }
block4 (table): { joint: rigid, shape: ssBox, Q: [1, -.4, .095, 1, 0, 0, 1], size: [.06, .15, .09, .01], color: [.6, .6, .6], contact }
block5 (table): { joint: rigid, shape: ssBox, Q: [1, -.6, .095, 1, 0, 0, 1], size: [.06, .15, .09, .01], color: [.6, .6, .6], contact }
block6 (table): { joint: rigid, shape: ssBox, Q: [1, -.8, .095, 1, 0, 0, 1], size: [.06, .15, .09, .01], color: [.6, .6, .6], contact }

board1 (box2): { joint: rigid, shape: ssBox, Q: [.0, -.2, .215], size: [.5, .15, .03, .01], color: [.6, .6, .6], contact }
board2 (box2): { joint: rigid, shape: ssBox, Q: [.0, -.0, .215], size: [.5, .15, .03, .01], color: [.6, .6, .6], contact }
board3 (box2): { joint: rigid, shape: ssBox, Q: [.0, , .2, .215], size: [.5, .15, .03, .01], color: [.6, .6, .6], contact }

stick1 (table): { joint: rigid, shape: capsule, Q: "t(.1 -.5 .07) d(90 1 0 0) ", size: [.5, .02], color: [.6, .6, .6], contact }
#stick2 (table): { joint: rigid, shape: capsule, Q: "t(.4 -.9 .07) d(90 1 0 0) ", size: [.5, .02], color: [.6, .6, .6], contact }
#stick3 (table): { joint: rigid, shape: capsule, Q: "t(.6 -.9 .07) d(90 1 0 0) ", size: [.5, .02], color: [.6, .6, .6], contact }

hook(table): { joint: rigid, type: ssBox, Q: [-.1, -.6, .07], size: [.03, .8, .04, .01], color: [.6, .6, .6], contact, logical:{ object } }
hookTip (hook): { Q: [.085, -.385, 0], type: ssBox, size: [.2, .03, .04, .01], color: [.6, .6, .6], contact, logical:{ object, pusher } }

Include: <ringBox.g>
Edit box (table): { joint: rigid, Q: "t(1.25 -.7 .15) d(45 0 0 1)" }

drawer (table): { Q: [-1.2, .5, .1] }
drawerA (drawer): { shape: ssBox, size: [.45, .05, .1, .02], Q: [0, -.15, 0], color: [.6, .6, .6], contact }
drawerB (drawer): { shape: ssBox, size: [.05, .35, .1, .02], Q: [-.2, 0, 0], color: [.6, .6, .6], contact }
drawerC (drawer): { shape: ssBox, size: [.05, .35, .1, .02], Q: [.2, 0, 0], color: [.6, .6, .6], contact }

bucket (table): { Q: [.5, 1., .15] }
bucketA (bucket): { shape: ssBox, size: [.33, .33, .03, .01], Q: [0, 0, -.075], color: [.6, .6, .6], contact }
bucketA (bucket): { shape: ssBox, size: [.33, .03, .15, .01], Q: [0, -.15, 0], color: [.6, .6, .6], contact }
bucketA (bucket): { shape: ssBox, size: [.33, .03, .15, .01], Q: [0, , .15, 0], color: [.6, .6, .6], contact }
bucketB (bucket): { shape: ssBox, size: [.03, .33, .15, .01], Q: [-.15, 0, 0], color: [.6, .6, .6], contact }
bucketC (bucket): { shape: ssBox, size: [.03, .33, .15, .01], Q: [.15, 0, 0], color: [.6, .6, .6], contact }
 

ball1 (drawer): { joint: rigid, Q: [.0, 0., .0], shape: sphere, size: [.05], color: [1, 1, .6] }

ball2 (table): { joint: rigid, Q: [.8, .5, .1], shape: sphere, size: [.05], color: [1, 1, .6] }

ball3 (bucket): { joint: rigid, Q: [0, 0, 0], shape: sphere, size: [.05], color: [1, 1, .6] }
 

 

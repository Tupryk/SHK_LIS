world: {}

### table

table(world): {
 shape: ssBox, Q: "t(0 0. .6)", size: [2.5, 2.5, .1, .02], color: [.3, .3, .3],
 contact: 1, logical: { },
 friction: .1
}

### one panda

Prefix: "l_"
#Include: <panda_fixRobotiq.g>
Include: <panda_fixGripper.g>

Prefix: False

Edit l_panda_base (table): { Q: "t(0 -.2 .05) d(90 0 0 1)" joint:rigid }

Edit l_panda_joint2: { q: -.5 }
Edit l_panda_joint4: { q: -2 }
Edit l_panda_joint7: { q: -.5 }

### camera

cameraTop(world): {
 Q: "t(-0.01 -.2 1.8) d(-150 1 0 0)",
 shape: marker, size: [.1],
 focalLength: 0.895, width: 640, height: 360, zRange: [.5, 100]
}

cameraWrist(l_panda_joint7): {
 Q: [0.0566288, -0.0138618, 0.158583, 0.371288, -0.0124238, 0.0272688, -0.928034]
#, Q: "d(180 0 1 0) d(-45 0 0 1) t(-.03 -.045 -.16) t(0 0 .01)",
 shape: camera, size: [.1],
 focalLength: 0.895, width: 640, height: 360, zRange: [.1, 10]
}

panda_collCameraWrist(cameraWrist): {
 Q: "d(90 0 1 0) t(-.02 0 0)"
 , shape: capsule, color: [1.,1.,1.,.2], size: [.05, .03], contact: -3 }

#bellybutton(l_panda_joint2): {
# Q: "d(90 0 1 0) t(-.0184 .0 .0545)",
# shape: cylinder, size: [.001, .0125], color: [0, 0, 1]
#}

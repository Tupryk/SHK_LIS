world: {}

### table

table(world): {
 shape: ssBox, Q: "t(0 0. .6)", size: [2.5, 2.5, .1, .02], color: [.3, .3, .3],
 contact: 1, logical: { },
 friction: .1
}


### robot

Prefix: "l_"
#Include: <panda_fixRobotiq.g>
Include: <panda_fixGripper.g>

Prefix: False

Edit l_panda_base (table): { Q: "t(0 -.2 .05) d(90 0 0 1)" joint:rigid }

Edit l_panda_joint2: { q: -.5 }
Edit l_panda_joint4: { q: -2 }
Edit l_panda_joint7: { q: -.5 }


### camera

cameraWorld(world): {
    X: "t(.56 .25 .83)"
}

cameraFrame(cameraWorld): {
    Q: "d(80 0 1 0) d(-90 0 0 1) d(180 0 1 0)",
    shape: camera, size: [.1],
    focalLength: 0.895, width: 640, height: 360, zRange: [.1, 10]
}

collCameraFrame(cameraFrame): {
    Q: "d(90 0 1 0) t(-.02 0 0)"
    , shape: capsule, color: [1.,1.,1.,.2], size: [.05, .03], contact: -3
}

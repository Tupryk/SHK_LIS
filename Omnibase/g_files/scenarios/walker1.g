frame endeffA: {
 X: "t(0 0 .1)", ctrl_H: .1,
 shape: sphere, size: [.1], color: [1., .5, 0], contact: -1,
 logical:{gripper, support} }

frame arm0(endeffA): {
 joint: hingeZ, pre: "t(0 0 .22)"
 shape: ssBox, mass: 1, size: [0.1, 0.1, .5, .03], contact: -1 }

frame arm1(arm0): {
 joint: hingeX, pre: "t(0 0 .22)", post: "t(0 0 .22)", q: 1.
 shape: ssBox, mass: 1, size: [0.1, 0.1, .5, .03], contact: -1 }

frame arm2(arm1): {
 joint: hingeX, pre: "t(0 0 .22)", post: "t(0 0 .22)", q: 1.
 shape: ssBox, mass: 1, size: [0.1, 0.1, .5, .03], contact: -1 }

frame endeffB(arm2): {
 joint: hingeZ, pre: "t(0 0 .22)", ctrl_H: .1,
 shape: sphere, size: [.1], color: [1., 0, .5], contact: -1,
 logical:{gripper} }



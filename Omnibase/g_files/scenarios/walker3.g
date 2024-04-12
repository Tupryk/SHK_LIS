endeffA: {
 X: "t(0 0 .05)"
 shape: sphere, size: [.06], color: [.9, .9, .9], contact: -2,
 logical:{gripper} }

arm0 (endeffA): {
 Q: "t(0 0 .1)"
 shape: ssBox, mass: .5, size: [0.1, 0.1, .2, .03], contact: -2 }

jointA(arm0): {
 joint: quatBall, pre: "t(0 0 .1)", post: "t(0 0 .25)", Q: "d(30 1 0 0)"
 shape: ssBox, mass: 1, size: [0.1, 0.1, .5, .03], contact: -2 }

jointMid(jointA): {
 joint: hingeX, pre: "t(0 0 .25)", post: "t(0 0 .25) ", q: 2.
 shape: ssBox, mass: 1, size: [0.1, 0.1, .5, .03], contact: -2 }

jointB(jointMid): {
 joint: quatBall, pre: "t(0 0 .25)", post: "t(0 0 .1)", Q: "d(30 1 0 0)"
 shape: ssBox, mass: 1, size: [0.1, 0.1, .2, .03], contact: -2 }

endeffB(jointB): {
 Q: "t(0 0 .1)" 
 shape: sphere, size: [.06], color: [.9, .9, .9], contact: -2,
 logical:{gripper} }

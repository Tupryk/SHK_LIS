## mobile base

world: { X: "t (0 0 .2)" }

base (world): {
 shape: ssBox, size: [.4, .4, .4, .05], joint: transXYPhi, limits: [-10,10,-10,10,-4,4], contact: 1 }

arm0 (base): {
 Q: "t (0 0 .3)", shape: capsule, size: [.4, .08], contact: -1 }

joint1 (arm0): {
 pre: "t (0 0 .2)"
 joint: hingeX, limits: [-1., 2.], q: .5 }

arm1 (joint1): {
 Q: "t (0 0 .4)", shape: capsule, size: [.8, .06] }
arm1_coll(arm1): { shape: capsule, color: [1.,1.,1.,.2], size: [.8, .2], contact: -2 }

joint2(arm1): {
 pre: "t(0 0 .4)"
 joint: hingeX, limits: [-1., 3.], q: 1.5 }

arm2(joint2): {
 Q: "t (0 0 .2)", shape: capsule, size: [.4, .04] }

arm2_coll(arm2): { shape: capsule, color: [1.,1.,1.,.2], size: [.25, .13], contact: -2 }

joint2a(arm2): {
 pre: "t(0 0 .25)"
 joint: hingeZ, limits: [-2., 2.], q: 0 }

joint2b(joint2a): {
 joint: hingeX, limits: [-2., 2.], q: 0 }

joint2c(joint2b): {
 joint: hingeY, limits: [-1., 1.], q: 0 }

arm3(joint2c): {
 shape: ssBox, size: [.1, , .05, .03, .01] }

Include: <gripper.g>

Edit gripper(joint2c): { Q: "d(180 1 0 0)" }
#joint joint3(arm2 gripper): {
#, joint: hingeZ, limits: [-1, 1], pre: "t(0 0 .25) d(180 1 0 0)" }



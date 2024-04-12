base: {
 X: "t(0 0 .1)"
 shape: ssBox, mass: .5, size: [0.1, 0.1, .25, .03], contact: -1 }

joint1(base): {
 joint: hingeZ, pre: "t(0 0 .1)", limits: [-2, 2] }
(joint1): { Q: [0, 0, .1], shape: ssBox, mass: .5, size: [0.1, 0.1, .25, .03], contact: -1 }

joint2(joint1): { joint: hingeX, pre: "t(0 0 .2)", limits: [-2, 2], q: -.3 }
(joint2): { Q: [0, 0, .1], shape: ssBox, mass: 1, size: [0.1, 0.1, .25, .03], contact: -1 }

joint3(joint2): { joint: hingeZ, pre: "t(0 0 .2)", limits: [-2, 2] }
(joint3): { Q: [0, 0, .1], shape: ssBox, mass: 1, size: [0.1, 0.1, .25, .03], contact: -1 }

joint4(joint3): { joint: hingeX, pre: "t(0 0 .2)", limits: [-2, 2], q: -1.3 }
(joint4): { Q: [0, 0, .1], shape: ssBox, mass: 1, size: [0.1, 0.1, .25, .03], contact: -1 }

joint5(joint4): { joint: hingeZ, pre: "t(0 0 .2)", limits: [-2, 2] }
(joint5): { Q: [0, 0, .1], shape: ssBox, mass: 1, size: [0.1, 0.1, .25, .03], contact: -1 }

joint6(joint5): { joint: hingeX, pre: "t(0 0 .2)", limits: [-2, 2], q: -1.3 }
(joint6): { Q: [0, 0, .1], shape: ssBox, mass: 1, size: [0.1, 0.1, .25, .03], contact: -1 }

joint7(joint6): { joint: hingeZ, pre: "t(0 0 .2)", limits: [-2, 2] }
(joint7): { Q: [0, 0, .1], shape: ssBox, mass: 1, size: [0.1, 0.1, .25, .03], contact: -1 }

endeff(joint7): {
#, Q: "t(0 0 .15) d(180 1 0 0)", shape: marker, size: [.02], color: [1., 1., 0], contact: -1 }
 Q: "t(0 0 .25) d(180 1 0 0)", shape: ssBox, size: [.03, .03, .05, .01], color: [1., 1., 0], contact: -1 }


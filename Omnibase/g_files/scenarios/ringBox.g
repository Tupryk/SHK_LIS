box: {
 shape: ssBox, X: [0, 0, .1], size: [.2, .2, .2, .02], color: [.6, .6, .6]
 mass: .5, friction: .1 }

ring (box): { Q: "t(0 0 .2)" }

ring0(ring): { shape: capsule, Q: "t(0 0 -.1)", size: [.05, .02], color: [.6, .6, .6], logical:{object} }
ring1(ring): { shape: capsule, Q: "t(.07 0 0)", size: [.1, .02], color: [.6, .6, .6], logical:{object} }
ring2(ring): { shape: capsule, Q: "t(-.07 0 0)", size: [.1, .02], color: [.6, .6, .6], logical:{object} }
ring3(ring): { shape: capsule, Q: "t(0 0 -.05) d(90 0 1 0)", size: [.1, .02], color: [.6, .6, .6], logical:{object} }
ring4(ring): { shape: capsule, Q: "t(0 0 .05) d(90 0 1 0)", size: [.1, .02], color: [.6, .6, .6], logical:{object}, contact: -1 }

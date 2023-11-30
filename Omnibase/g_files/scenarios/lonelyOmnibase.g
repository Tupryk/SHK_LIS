world: {}

### floor

floor(world): {
 shape: ssBox, Q: "t(0 0. .1)", size: [2, 2, .02, .02], color: [.3, .3, .3],
 contact: 1, logical: { },
 friction: .1
}

### omnibase
Include: <../omniBase/omniBase.g>

Edit omnibase_base (floor): { Q: "t(0 0 .1)" joint:rigid }

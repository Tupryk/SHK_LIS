## create standard base frame before including

omnibase_base: {shape: ssBox, Q: "t(0 0. .6)", size: [.01, .01, .01, .02], color: [.0, .0, .0, .0]}
omnibase_joint(omnibase_base): { joint: transXYPhi }
omnibase(omnibase_joint): { shape: mesh, mesh: <models/omnibase_battery.stl>, visual: True, contact: -2 }

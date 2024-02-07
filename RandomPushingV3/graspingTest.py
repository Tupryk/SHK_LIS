from highLevelManipulation import Robot

robot = Robot(real_robot=True)

robot.updateObjectPosition()

robot.goHome()
print(robot.obj_dims)

robot.graspObject()

robot.moveBack(dir="z")

robot.graspObject(place=True)

robot.moveBack(dir="z")

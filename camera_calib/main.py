import cv2
import rowan
import numpy as np
import robotic as ry
import matplotlib.pyplot as plt


C = ry.Config()
C.addFile(ry.raiPath('scenarios/pandaSingle_tableCam.g'))

p1 = np.array([.0, .1, .89])
p2 = np.array([.0, .4, .99])
p3 = np.array([.0, .25, .75])
calib_marker = C.addFrame("calib_marker").setShape(ry.ST.sphere, [.0175]).setPosition(p1).setColor([1., .0, .0])


viewed_p = []
for p in [p1, p2, p3]:
    calib_marker.setPosition(p)
    bot = ry.BotOp(C, False)
    bot.home(C)
    bot.gripperClose(ry._left)
    image, depth, points = bot.getImageDepthPcl('cameraFrame', False)

    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    lower_red = np.array([0, 100, 100])
    upper_red = np.array([10, 255, 255])

    mask = cv2.inRange(hsv, lower_red, upper_red)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    circle_contour = None
    for contour in contours:
        perimeter = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.04 * perimeter, True)
        if len(approx) > 6:
            circle_contour = contour
            break

    if circle_contour is not None:
        M = cv2.moments(circle_contour)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            print("Center point of the red circle:", (cX, cY))
        else:
            print("No center point found")
    else:
        print("No red circle found")

    fig, axs = plt.subplots(2, 2, figsize=(10, 5))
    axs[0, 0].imshow(image)
    axs[0, 1].matshow(depth)
    cv2.drawContours(image, [circle_contour], -1, (0, 255, 0), 2)
    cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)

    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    axs[1, 0].imshow(image)
    plt.show()

    point = points[cY, cX]

    viewed_p.append(point)

# Gripper y width offset
# plane_midpoint += .01*C.eval(ry.FS.vectorY, ["l_gripper"])[0] # idk...

def create_plane(point1: np.ndarray, point2: np.ndarray, point3: np.ndarray, frame_name: str="plane", parent_frame: str=""):
    plane_midpoint = (point1 + point2 + point3) / 3.
    b = point2-point1
    b /= np.linalg.norm(b)
    d = point3-point1
    d /= np.linalg.norm(d)
    a = np.cross(b, d)
    a /= np.linalg.norm(a)
    c = np.cross(a, b)
    c /= np.linalg.norm(c)
    plane_rot_mat = np.array([b, c, a]).T

    rot = rowan.from_matrix(plane_rot_mat)
    if len(parent_frame):
        C.addFrame(frame_name, parent_frame).setShape(ry.ST.marker, [.02]).setRelativePosition(plane_midpoint).setRelativeQuaternion(rot).setColor([1., .0, 1.])
    else:
        C.addFrame(frame_name).setShape(ry.ST.marker, [.02]).setPosition(plane_midpoint).setQuaternion(rot).setColor([1., 1., .0])

# C.addFrame("cameraPos").setShape(ry.ST.sphere, [.03]).setPosition(pred_cam_pos).setColor([.0, 1., .0])
# C.addFrame("camera2point", "cameraFrame").setShape(ry.ST.sphere, [.03]).setRelativePosition(point).setColor([1., 1., .0])

C.view(True)

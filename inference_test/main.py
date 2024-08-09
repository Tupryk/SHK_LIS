import cv2
import zmq
import pickle
import robotic as ry
import numpy as np


def list_cameras(max_cameras=10):
    available_cameras = []
    
    for i in range(max_cameras):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            available_cameras.append(i)
            cap.release()
    
    return available_cameras

# List available cameras
cameras = list_cameras()
print(cameras)

C = ry.Config()
C.addFile(ry.raiPath('scenarios/pandaSingle.g'))

bot = ry.BotOp(C, True)
bot.home(C)

cap = cv2.VideoCapture(4)

if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()


for i in range(50):
    ret, frame = cap.read()
cv2.imwrite('buffer_image.jpg', frame)

context = zmq.Context()
socket = context.socket(zmq.REP)
socket.bind("tcp://*:69420")

prev_gripper_open = False
gripper_open = False
while True:
    ret, frame = cap.read()

    print("Waiting for client input...")
    client_input = socket.recv()
    client_input = pickle.loads(client_input)
    print(f"Received request: {client_input}")
    
    if "init" in client_input.keys():
        bot.home(C)
        bot.gripperMove(ry._left)
        while not bot.gripperDone(ry._left):
            bot.sync(C)
    else:
        gripper_open = client_input["action"][-1] > .5

        bot.moveTo(client_input["action"][:7], overwrite=True)
        bot.sync(C)

        if gripper_open != prev_gripper_open:
            if gripper_open:
                bot.gripperClose(ry._left)
                while not bot.gripperDone():
                    bot.sync(C)
            else:
                bot.gripperMove(ry._left)
                while not bot.gripperDone():
                    bot.sync(C)

        prev_gripper_open = gripper_open

    qpos = np.append(bot.get_q(), 1. if gripper_open else 0.)
    reply = {
        "cam1": cv2.resize(frame, (320, 180)),
        "cam2": cv2.resize(frame, (320, 180)),
        "qpos": qpos
    }
    to_send = pickle.dumps(reply)
    socket.send(to_send)
    print("Sent a response.")
    
cap.release()

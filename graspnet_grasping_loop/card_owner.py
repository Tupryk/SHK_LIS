import zmq
import pickle
import numpy as np

import sys
import os

sys.path.append(os.path.join(os.path.dirname(__file__), './contact_graspnet/contact_graspnet'))

from custom_inference import GraspnetModel


def process_client_inputs(model, verbose: int=0):
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.bind("tcp://*:69420")

    while True:
        client_input = socket.recv()
        client_input = pickle.loads(client_input)
        if verbose:
            print(f"Received request: {client_input}")

        output = model.predict(client_input)
        to_send = pickle.dumps(output)
        socket.send(to_send)
        if verbose:
            print("Sent a response.")


model = GraspnetModel()
process_client_inputs(model, verbose=1)

import zmq
import pickle
import numpy as np


def request_card_processing(
        input: np.ndarray,
        address: str="tcp://localhost:69420",
        verbose: int=0):
    
    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    socket.connect(address)

    if verbose:
        print(f"Sending input {input} to be processed...")
    to_send = pickle.dumps(input)
    socket.send(to_send)

    reply = socket.recv()
    reply = pickle.loads(reply)
    if verbose:
        print(f"Received reply {reply} [ {input} ]")

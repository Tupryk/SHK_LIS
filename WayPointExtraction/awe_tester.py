import numpy as np
import time 
from robot_awe import extract_waypoints


def get_waypoints(tau, eta , M):
    pass


def preprocess_traj(W, tau):
    pass


def main():  
    eta = .005
    D = np.load("logs/qReal.npy")


    waypoints  = extract_waypoints(D, eta )





if __name__ == "__main__":
    main()
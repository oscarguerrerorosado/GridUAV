/*
Webots controller class for the UAV.
This class is responsible for the communication with the Webots simulator.

It communicates with the following classes:
- the VelNoise class as orchestration layer.
- the UAVControl class to get the current state of the UAV and to set the target velocity.
- the Navigation class to get the target position and velocity.

It centralises the interaction with the webots simulator, controlling the time loop and state of the simulation.
It processes the UAV state and the lidar scan from the UAVControl class, to detect possible collisions.
It transfers this information to the Navigation class to generate the next step in the movement path and returns this to the UAVControl class.
*/

from tqdm import tqdm
import numpy as np

class WebotsController: 
    def __init__(self):

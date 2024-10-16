import math
from typing import Dict
import numpy as np
from coppeliaRobot import CoppeliaRobot
import time
import copy
import matplotlib.pyplot as plt
import machinevisiontoolbox as mvt
from cv2 import findHomography

def PickAndPlaceRobot(robotObj,img:mvt.Image,target_positions:Dict):
    """
    Reposition objects to a desired location.
    Note: You must not change the name of this file or this function.

    Parameters
    ----------
    robotObj
        Dobot object; see fakeRobot.py for the API
        You can pass in a FakeRobot or CoppeliaRobot object for testing
    img
        A warped mvt image of the robot workspace.
    target_positions
        A dictionary containing the positions in the robot's XY plane that the
        objects must be placed.
    """


    for shape,place_position in target_positions.items():
        pick_position = np.array([183,0,177])
        place_position = np.array([183,0,177])
        PickUp(robotObj,pick_position)
        Place(robotObj,place_position)
    return

# Note: The remainder of the file is a template on how we would solve this task.
# You are free to use our template, or to write your own code.
def PickUp(robotObj:CoppeliaRobot, target_pos: np.array):
    '''
    This function should move the robot to the given position and pick up 
    an item if present.
    Note: We recommend the following strategy:
    1. Move the robot to a position 50mm above the target position
    2. Move the robot to the target position
    3. Activate the suction cup
    4. Move the robot to a position 50mm above the target position
    This strategy will prevent dragging the object.
    '''
    # Move the robot to a position 50mm above the target position
    robotObj.set_suction_cup(0)
    pos = copy.deepcopy(target_pos)
    pos[2] = pos[2] + 50
    j1, j2, j3 = ikine(pos)
    robotObj.move_arm(j1, j2, j3)
    time.sleep(2)

    # Move the robot to the target position
    pos = target_pos  # You will need to change this
    pj1, pj2, pj3 = ikine(pos)
    robotObj.move_arm(pj1, pj2, pj3)
    time.sleep(0.5)

    # Active suction cup
    robotObj.set_suction_cup(1)
    time.sleep(0.5)
    
    robotObj.move_arm(j1, j2, j3)
    time.sleep(0.5)

def Place(robotObj, target_pos: np.array):
    ''' 
    This function should move the robot to the given position and release a
    held item if present.
    Note: We recommend the following strategy:
    1. Move the robot to a position 50mm above the target position
    2. Move the robot to the target position
    3. Release the suction cup
    4. Move the robot to a position 50mm above the target position
    This strategy will prevent dragging a held object.
    '''
    # Move the robot to a position 50mm above the target position
    pos = target_pos
    pos = copy.deepcopy(target_pos)
    pos[2] = pos[2] + 50
    j1, j2, j3 = ikine(pos)
    robotObj.move_arm(j1, j2, j3)
    time.sleep(2)
    pj1, pj2, pj3 = ikine(target_pos)
    robotObj.move_arm(pj1, pj2, pj3)
    time.sleep(0.5)
    robotObj.set_suction_cup(0)
    
    time.sleep(0.5)
    
    robotObj.move_arm(j1, j2, j3)
    time.sleep(0.5)



def ikine(pos: np.array) -> np.array:
    """
    Inverse kinematics using geometry

    Parameters
    ----------
    pos
        A numpy array of shape (3,) of the desired end-effector position [x, y, z] (mm)

    Returns
    -------
    theta
        A numpy array of shape (3,) of joint angles [theta1, theta2, theta3] (radians)

    """
    pass




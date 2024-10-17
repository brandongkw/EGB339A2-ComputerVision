import math
from typing import Dict
import scipy.optimize
import sympy as sp
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





# -------- Define Kinematics Functions --------
def rotation_matrix(axis, angle):
    if axis == "x":
        R = np.array([[1, 0, 0],
                      [0, np.cos(angle), -np.sin(angle)],
                      [0, np.sin(angle), np.cos(angle)]], dtype=np.float64)
    elif axis == "y":
        R = np.array([[np.cos(angle), 0, np.sin(angle)],
                      [0, 1, 0],
                      [-np.sin(angle), 0, np.cos(angle)]], dtype=np.float64)
    elif axis == "z":
        R = np.array([[np.cos(angle), -np.sin(angle), 0],
                      [np.sin(angle), np.cos(angle), 0],
                      [0, 0, 1]], dtype=np.float64)
    return R

def transformation_matrix(axis, angle, t):
    T = np.eye(4)
    R = rotation_matrix(axis, angle)

    T[:3, 3] = t # [:3,3] ]
    T[:3, :3] = R
    return T

def forward_kinematics_q(q):
    """
    Calculate the forward kinematics of the robot for a given set of joint angles

    Parameters
    ----------
    q
        A numpy array of shape (4,) of joint angles [q1, q2, q3, q4] in radians

    Returns
    -------
    p
        A numpy array of shape (3,) of the position of the end effector in the world
        frame in units of mm

    """
    q1, q2, q3, q4 = q
    q_1 = np.eye(4)
    q_1[:3, :3] = rotation_matrix("z", q1)
    q_2 = q_1 @ transformation_matrix("y", q2, [0, 0, L0])
    q_3 = q_2 @ transformation_matrix("y", q3, [0, 0, L1])
    q_4 = q_3 @ transformation_matrix("y", q4, [L2, 0, 0])
    p = q_4 @ transformation_matrix("x", 0, [L3, 0, L4])
    return p[:3, 3]
def joint_mapping(th):
    """
    Map the physical joint angles to the kinematic joint angles

    Parameters
    ----------
    th
        A numpy array array of shape (3,) of physical joint angles
        [theta1, theta2, theta3] in radians

    Returns
    -------
    q
        A numpy array of shape (4,) of kinematic joint angles [q1, q2, q3, q4]
        in radians
    """
    theta1, theta2, theta3 = th
    q = np.zeros(4)

    q[0] = theta1
    q[1] = theta2
    q[2] = theta3 - theta2
    q[3] = -theta3
    return q
def forward_kinematics(theta):
    """
    Calculate the forward kinematics of the robot for a given set of
    physical joint angles

    Parameters
    ----------
    theta
        A numpy array of shape (3,) of physical joint angles [theta1, theta2, theta3]
        in radians

    Returns
    -------
    p
        A numpy array of shape (3,) of the position of the end effector in the world
        frame in millimeters

    """
    q = joint_mapping(theta)
    return forward_kinematics_q(q)



def cost(theta, pstar):
    """
    Cost function for inverse kinematics

    Parameters
    ----------
    theta
        A numpy array of shape (3,) of joint angles [theta1, theta2, theta3] (radians)
    pstar
        A numpy array of shape (3,) of the desired end-effector position [x, y, z] (mm)

    Returns
    -------
    c
        A scalar value cost which is the Euclidean distance between
        forward kinematics and pstar

    """
    p = forward_kinematics(theta)
    c = np.linalg.norm(p - pstar)

    return c

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
    theta = np.zeros(3)
    theta = scipy.optimize.fmin(cost, np.array([0.0,0.0,0.0], dtype=np.float64), (pos,))
    return theta
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
import cv2

L0 = 138
L1 = 135
L2 = 147
L3 = 60
L4 = -80



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

    # 1. Locate shapes in the image
    shapes = robotObj.REF_LocateShapes(img)
    if shapes is None or 'calibration markers' not in shapes:
        raise ValueError("Failed to locate shapes or calibration markers in the image.")

    # 2. Unpack shapes data: separate calibration markers and object positions
    sorted_calibration_markers = []
    object_positions = {}
    for shape_name, blob in shapes.items():
        if isinstance(blob, list):  # Calibration markers are lists of blobs
            print(f"{shape_name}:")
            for b in blob:
                if b is not None:  # Ensure valid blobs are added
                    print(f"  Blob ID: {b.id}, Center: ({b.uc}, {b.vc})")
                    sorted_calibration_markers.append(b)
        else:
            print(f"{shape_name}: Center: ({blob.u}, {blob.v})")
            object_positions[shape_name] = np.array([blob.u, blob.v])

    # 3. Print the calibration markers for debugging
    print("\nsorted_calibration_markers:")
    for blob in sorted_calibration_markers:
        print(f"Blob ID: {blob.id}, uc: {blob.uc}, vc: {blob.vc}")

    # 4. Calculate the homography matrix
    homography = get_homography(sorted_calibration_markers)
    print("Homography Matrix:\n", homography)

    # 5. Transform object positions to the robot's XY plane
    object_positions_robot = robotObj.REF_GetObjectXY(object_positions, homography)
    print("Object Positions in Robot's XY Plane:\n", object_positions_robot)

    # 6. Pick and place objects according to the target positions
    for shape, target_pos in target_positions.items():
        if shape not in object_positions_robot:
            print(f"Warning: {shape} not detected, skipping...")
            continue
        
        # Get the pick position for the current object
        pick_position = np.array([*object_positions_robot[shape], 0])  # z = 0 for 2D workspace

        # Add height (z) to target position for placement
        place_position = np.array([*target_pos, 0])

        # Print debug info
        print(f"Picking {shape} from {pick_position} and placing at {place_position}")

        # Perform pick and place operations
        PickUp(robotObj, pick_position)
        Place(robotObj, place_position)

    print("Pick-and-place operation complete.")

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
    # 1. Move to a position 50mm above the target position
    pos_above = copy.deepcopy(target_pos)
    pos_above[2] += 50  # 50mm above the target
    j1, j2, j3 = ikine(pos_above)  # Compute joint angles for this position

    print(f"Moving to above the target position: {pos_above}")
    robotObj.move_arm(j1, j2, j3)  # Move robot arm to this position
    time.sleep(2)  # Allow time for movement

    # 2. Move to the target position
    j1, j2, j3 = ikine(target_pos)  # Compute joint angles for the actual target position
    print(f"Moving to the target position: {target_pos}")
    robotObj.move_arm(j1, j2, j3)  # Move to the target
    time.sleep(0.5)

    # 3. Activate suction cup to pick up the object
    print("Activating suction cup.")
    robotObj.set_suction_cup(1)  # Suction ON
    time.sleep(0.5)  # Allow suction time

    # 4. Move back to the position 50mm above the target position
    print(f"Moving back to above the target position: {pos_above}")
    robotObj.move_arm(j1, j2, j3)  # Return to the position 50mm above
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
# 1. Move the robot to a position 50mm above the target position
    pos_above = copy.deepcopy(target_pos)
    pos_above[2] += 50  # Add 50mm in z-direction
    j1, j2, j3 = ikine(pos_above)

    if j1 is None or j2 is None or j3 is None:
        raise ValueError("Inverse kinematics failed for the above-target position.")

    print(f"Moving to 50mm above the target: {pos_above}")
    robotObj.move_arm(j1, j2, j3)
    time.sleep(1)  # Allow time for the movement

    # 2. Move the robot to the target position
    j1, j2, j3 = ikine(target_pos)
    if j1 is None or j2 is None or j3 is None:
        raise ValueError("Inverse kinematics failed for the target position.")

    print(f"Moving to the target position: {target_pos}")
    robotObj.move_arm(j1, j2, j3)
    time.sleep(0.5)

    # 3. Release the suction cup to drop the object
    print("Releasing the suction cup.")
    robotObj.set_suction_cup(0)  # Deactivate suction
    time.sleep(0.5)

    # 4. Move back to the position 50mm above the target position
    print(f"Moving back to 50mm above the target: {pos_above}")
    j1, j2, j3 = ikine(pos_above)
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


# -------- Define Image Processing Functions --------
def get_homography(blob_list):
    """
    Calculate the homography matrix for the given image warping parameters

    Parameters
    ----------

    Returns
    -------
    homography
        An numpy array that can be used to warp an image.
    """
    ref_points_array = np.array([[blob.uc, blob.vc] for blob in blob_list], dtype=np.float32)
    ground_points = np.array([
        [223, 22.5],
        [223, -22.5],
        [178, 22.5],
        [178, -22.5]
    ], dtype=np.float32)
    homography_matrix, _ = cv2.findHomography(ref_points_array.reshape(-1, 1, 2), ground_points.reshape(-1, 1, 2))
    return homography_matrix

def locate_shapes(img: mvt.Image):
    """
    Locate shapes in the image

    Parameters
    ----------
    img
        A warped mvt image of the robot workspace.

    Returns
    -------
    shapes
        A dictionary containing the shapes in the image
    """
    shapes = {}
    
    # Locate shapes in the image
    # shapes = robotObj.REF_LocateShapes(img)


    return shapes
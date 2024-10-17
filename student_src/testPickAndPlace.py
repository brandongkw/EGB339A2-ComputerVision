'''
You can run this file to test your code.
We have provided a CoppeliaRobot class that you can use to test your code.
The CoppeliaRobot class uses the same GenericRobotAPI as the real Dobot robot.
You must not submit this file - you only submit the wordTypingRobot.py file.
'''

import argparse

from coppeliaRobot import CoppeliaRobot
from genericRobotAPI import GenericRobotAPI
from PickAndPlacerobot import PickAndPlaceRobot
import multiprocessing as mp
import cv2
import numpy as np
import time
import machinevisiontoolbox as mvt
import matplotlib.pyplot as plt
import pickle

def create_homography(theta,skew_x,skew_y,perspective_warp1,perspective_warp2):
    '''
    Creates a homography that can be used to warp an image.

    Parameters
    ----------
    theta
        The angle in radians that the image should be warped.
    skew_x
        The factor that the image should be skewed in the x-direction
    skew_y
        Same as above but for the y-direction
    perspective_warp1
        One of the two factors for applying perspective warp.
    perspective_warp2
        One of the two factors for applying perspective warp.

    Returns
    -------
    homography
        An numpy array that can be used to warp an image.
    '''
    
    tx = 0 # Translation is useless when the tile option is used when warping.
    ty = 0

    He = np.array([ # Translation/rotation
        [np.cos(theta),-np.sin(theta),tx],
        [np.sin(theta), np.cos(theta),ty],
        [0,0,1]
    ])
    Ha = np.array([ # Skew
        [1,skew_y,0],
        [skew_x,1,0],
        [0,0,1]
    ])
    Hp = np.array([ # Perspective warping.
        [1,0,0],
        [0,1,0],
        [perspective_warp1,perspective_warp2,1]
    ])
    return He@Ha@Hp

def test_final_positions(robot,target_positions):
    if robot.REF_ikine_used:
        print("reference ikine function used.")
    if robot.REF_GetHomography_used:
        print("reference GetHomography function used.")
    if robot.REF_GetObjectXY_used:
        print("reference GetObjectXY function used.")
    if robot.REF_SortCalibrationMarkers_used:
        print("reference SortCalibrationMarkers function used.")
    if robot.REF_LocateShapes_used:
        print("reference LocateShapes function used.")

    positions = robot.get_object_positions()
    errors = {}
    for shape,target in target_positions.items():
        place_pos = positions[shape][:2]

        errors[shape] = np.linalg.norm(target-place_pos)
        print(f"{shape} error = {errors[shape]} mm.")
    return errors

def gui(robot: CoppeliaRobot):
    while True:
        time.sleep(1)
        # img = robot.get_image()
        cv2.imshow('img', img)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--printonly", action="store_true", help="Do not use the CoppeliaSim API; instead, print the joint angles to the console")
    args = parser.parse_args()

    if args.printonly:
        print("Running in print-only mode")
        robot = GenericRobotAPI()
    else:
        print("Running in CoppeliaSim mode")
        robot = CoppeliaRobot()
    print("Robot object created")
    robot.home()
    # mp.Process(target=gui, args=(robot,)).start() 
    print("Robot homed")

    warping_homography = create_homography(0,0,0,0,0)
    print(warping_homography)

    img = robot.get_image()
    warped_img = img.warp_perspective(warping_homography,tile=True)[0]
    # mvt.idisp(warped_img.to_float(),block=True)
    target_positions = {
        "blue square"   : np.array([300,-130],dtype=np.float64), # Go to 
        "green square"  : np.array([260, 170],dtype=np.float64),
        "red square"    : np.array([300, 100],dtype=np.float64),
        "blue circle"   : np.array([130, 150],dtype=np.float64),
        "green circle"  : np.array([300,   0],dtype=np.float64),
    }

    PickAndPlaceRobot(robot,warped_img,target_positions)
    test_final_positions(robot,target_positions)

    robot.stop_sim()
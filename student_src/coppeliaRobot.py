'''
This file contains the CoppeliaRobot class, which is a wrapper around the CoppeliaSim API.
There is no need for you to touch or understand this file.
'''

import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

from genericRobotAPI import GenericRobotAPI
import cv2
import numpy as np
import machinevisiontoolbox as mvt
import pickle
import random


class CoppeliaRobot(GenericRobotAPI):
    
    def __init__(self, host='localhost', port=23000):
        self.host = host
        self.port = port
        self.nDoF = 4
        self.robot_name = '/Dobot'
        self.motor_control_mode = 'position'
        self.REF_ikine_used = False
        self.REF_GetHomography_used = False
        self.REF_GetObjectXY_used = False
        self.REF_SortCalibrationMarkers_used = False
        self.REF_LocateShapes_used = False

        print('[CoppeliaRobot] Connecting to remote API server...', end='', flush=True)
        self.client = RemoteAPIClient(host=self.host, port=self.port)
        if self.client is not None:
            print(' Connected')
        else:
            print(' Failed!')
            exit(0)

        print('[CoppeliaRobot] Trying to get sim object... make sure the ZMQ connection is enabled!', end='', flush=True)
        self.sim = self.client.getObject("sim")
        if self.sim is None:
            print(' Could not get "sim" object; exit')
            exit(0)
        else:
            print(' Got it!')
        
        # ensure sim is stopped if reconnecting
        self.sim.stopSimulation()
        while self.sim.getSimulationState() != self.sim.simulation_stopped:
            time.sleep(0.01)

        self.robot = self.sim.getObject(self.robot_name)
        if self.robot is None:
            print(f'[CoppeliaRobot] Could not get {self.robot} object')

        # waist, shoulder, elbow
        self.joints = [self.sim.getObject(f'/Dobot/motor{idx}') for idx in range(1, self.nDoF+1)]

        self.end_effector = self.sim.getObject('/Dobot/suctionCup/DummyA')

        self.sim.addLog(self.sim.verbosity_scriptinfos, 'Python connected to EGB339 Prac Simulator!')


        self.camera = self.sim.getObjectHandle('/Vision_sensor')
        self.resolution = self.sim.getVisionSensorResolution(self.camera)
        # self.camera = sim.getVisionSensorImg(int sensorHandle,int options=0,float rgbaCutOff=0.0,list pos=[0,0],list size=[0,0])

        self.workspace = self.sim.getObject('/workspace')
        self.screen = self.sim.getObject('/Screen')

        self.start_sim()
     
    def get_image(self):
        buff, res = self.sim.getVisionSensorImg(self.camera)
        img = np.frombuffer(buff, dtype=np.uint8)
        img.resize([res[1], res[0], 3])
        img = cv2.flip(img, 0)
        return mvt.Image(img)

    def move_arm_wait(self, j1: float, j2: float, j3: float, j4:float = 0.0):
        self.move_arm(j1, j2, j3, j4)
        self.wait_joints()
        return

    def move_arm(self, j1: float, j2: float, j3: float, j4:float = 0.0):
        if not all(isinstance(x, (int, float)) for x in [j1, j2, j3]):
            raise Exception("[CoppeliaRobot] All parameters for move_arm() must be integers or floats")

        # set robot's joint angles in units of radians
        # theta is the angles: waist, shoulder, elbow
        theta = [j1, j2, j3 , j4]

        if self.motor_control_mode != 'position':
            # change motor control mode for each joint
            for idx in range(1, self.nDoF+1):
                self.sim.setObjectInt32Parameter(self.joints[idx], self.sim.jointintparam_ctrl_enabled, 1)
            self.motor_control_mode = 'position'

        if len(theta) == self.nDoF:
            for i in range(self.nDoF):
                # TODO: need to add some error checking here
                self.sim.setJointTargetPosition(self.joints[i], theta[i])
        else:
            raise ValueError('[CoppeliaRobot] theta not the same size as number of joints')
        
    def home(self):
        self.move_arm(0, 0, 0, 0)
        # TODO: REMOVE
        self.wait_joints()
        
    def wait_joints(self):
        for joint in self.joints:
            while not np.isclose(self.sim.getJointPosition(joint), self.sim.getJointTargetPosition(joint), atol=0.01):
                pass

    def get_joint_config(self) -> list:
        return [self.sim.getJointPosition(joint) for joint in self.joints][0:4]

    def get_end_effector_pose(self):
        # TODO: Double check whether it should be w.r.t. world frame or Dobot frame
        pose = self.sim.getObjectPose(self.end_effector, self.sim.handle_world)
        return pose

    def set_suction_cup(self, signal):
        self.sim.setInt32Signal('/Dobot/suctionCup', int(signal))

    def load_scene(self, scene_path: str):
        # scene_path needs to be full path to scene, scene name is not sufficient
        print(f'[CoppeliaRobot] Loading scene {scene_path}')
        self.sim.loadScene(scene_path)

    def close_scene(self):
        print('[CoppeliaRobot] Closing scene')
        self.sim.closeScene()

    def start_sim(self, sync=False):
        if sync:
            self.client.setStepping(True)
        print('[CoppeliaRobot] Starting simulation')
        self.sim.startSimulation()
        # startSimulation is just a request, wait till it actually started here
        while self.sim.getSimulationTime()==0 or self.sim.getSimulationState()==self.sim.simulation_stopped:
            time.sleep(0.01)

    def step_sim(self):
        self.client.step(wait=True)
        cv2.imshow('Camera Feed', self.get_image())
        
    def stop_sim(self):
        print('[CoppeliaRobot] Stopping simulation')
        self.sim.stopSimulation()

    def get_object_positions(self):
        green_cil = self.sim.getObject('/green_cylinder')
        green_box = self.sim.getObject('/green_box')
        blue_cil  = self.sim.getObject('/blue_cylinder')
        blue_box  = self.sim.getObject('/blue_box')
        red_box   = self.sim.getObject('/red_box')

        positions = { # in mm.
            "blue square"   : np.array(self.sim.getObjectPosition(blue_box ,-1)) * 1000,
            "green square"  : np.array(self.sim.getObjectPosition(green_box,-1)) * 1000,
            "red square"    : np.array(self.sim.getObjectPosition(red_box,-1)) * 1000,
            "blue circle"   : np.array(self.sim.getObjectPosition(blue_cil ,-1)) * 1000,
            "green circle"  : np.array(self.sim.getObjectPosition(green_cil,-1)) * 1000,
        }

        return positions


    ########### Reference solutions available to students at cost of a penalty
    def REF_ikine(self,pos: np.array) -> np.array:
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
        self.REF_ikine_used = True
        # robot dimensions
        L0 = 138  # height of shoulder raise joint above ground plane
        L1 = 135  # length of upper arm
        L2 = 147  # length of lower arm
        L3 = 60   # horizontal displacement from wrist "passive" joint
        L4 = -80  # vertical displacement down from wrist "passive" joint

        with open("reference values.pkl","rb") as fp:
            data = pickle.load(fp)
            for key,val in data.items():
                if isinstance(key,tuple):
                    if np.isclose(pos,key,atol=5).all():
                        theta=val
                        break
        return theta
    
    def REF_GetHomography(self,sorted_calibration_markers):
        """
        Creates a homography from image space to robot's XY plane.

        Parameters
        ----------
        sorted_calibration_markers
            A list of mvt blobs corresponding to the calibration markers. This
            list must be sorted such that the blobs are in clockwise order, 
            starting from the largest blob.
        
        Returns
        -------
        homography
            An numpy array containing the computed homography.
        """
        class Blob:
            def __init__(
                self,
                id = None,
                bbox = None,
                moments = None,
                touch = None,
                perimeter = None,
                a = None,
                b = None,
                orientation = None,
                children = None,
                parent = None,
                uc = None,
                vc = None,
                level = None,
            ):
                self.id = id
                self.bbox = bbox
                self.moments = moments
                self.touch = touch
                self.perimeter = perimeter
                self.a = a
                self.b = b
                self.orientation = orientation
                self.children = children
                self.parent = parent
                self.uc = uc
                self.vc = vc
                self.level = level
        
        calibration_blobs = []
        with open("reference values.pkl","rb") as fp:
            data = pickle.load(fp)
            for i in range(4):
                new_blob = Blob(
                    id = data["calibration_markers"]["id"][i],
                    bbox = data["calibration_markers"]["bbox"][i],
                    #moments = data["calibration_markers"]["moments"][i],
                    touch = data["calibration_markers"]["touch"][i],
                    perimeter = data["calibration_markers"]["perimeter"][i],
                    a = data["calibration_markers"]["a"][i],
                    b = data["calibration_markers"]["b"][i],
                    orientation = data["calibration_markers"]["orientation"][i],
                    children = data["calibration_markers"]["children"][i],
                    parent = data["calibration_markers"]["parent"][i],
                    uc = data["calibration_markers"]["uc"][i],
                    vc = data["calibration_markers"]["vc"][i],
                    level = data["calibration_markers"]["level"][i],                    
                )
                calibration_blobs.append(new_blob)
        for b1,b2 in zip(calibration_blobs,sorted_calibration_markers):
            if not (np.abs(b1.uc-b2.uc) < 5 and np.abs(b1.vc-b2.vc) < 5):
                print("Provided calibration markers are incorrect")
                    
        self.GetHomography_used = True
        with open("reference values.pkl","rb") as fp:
            data = pickle.load(fp)
            homography = data["homography"]
        return homography

    def REF_GetObjectXY(self,shapes,homography):
        """
        Converts UV coordinates into XY coordinates using the provided homography

        Parameters
        ----------
        shapes
            A Python dictionary containing UV coordinates of objects.
        homography
            A numpy array containing the homography used to convert between UV 
            and XY coordinates in mm.
        
        Returns
        -------
        object_positions
            A Python dict containing the XY coordinates of the objects found in
            the 'shapes' argument. Any invalid conversion will be not be present
            in this dictionary.
        """
        with open("reference values.pkl","rb") as fp:
            data = pickle.load(fp)
            ref_homography = data["homography"]
        if not np.all(np.abs(homography-ref_homography) < 1e-4):
            print("homography is incorrect for the standard configuration")
        self.REF_GetObjectXY_used = True
        object_positions = {
            "red square"    : np.array([200,-125]),
            "green square"  : np.array([300,   0]),
            "blue square"   : np.array([300,  65]),
            "green circle"  : np.array([200, 125]),
            "blue circle"   : np.array([300,-65]),
        }
        return object_positions

    def REF_SortCalibrationMarkers(self,calibration_markers):
        """
        Sorts a list of calibration blobs into clockwise order, starting from the
        largest calibration marker.

        Parameters
        ----------
        calibration_markers
            A list containing mvt blobs
        
        Returns
        -------
        sorted_calibration_markers
            A list containing mvt blobs in the required order
        """
        self.REF_SortCalibrationMarkers_used = True
        class Blob:
            def __init__(
                self,
                id = None,
                bbox = None,
                moments = None,
                touch = None,
                perimeter = None,
                a = None,
                b = None,
                orientation = None,
                children = None,
                parent = None,
                uc = None,
                vc = None,
                level = None,
            ):
                self.id = id
                self.bbox = bbox
                self.moments = moments
                self.touch = touch
                self.perimeter = perimeter
                self.a = a
                self.b = b
                self.orientation = orientation
                self.children = children
                self.parent = parent
                self.uc = uc
                self.vc = vc
                self.level = level
        calibration_blobs = []
        with open("reference values.pkl","rb") as fp:
            data = pickle.load(fp)
            for i in range(4):
                new_blob = Blob(
                    id = data["calibration_markers"]["id"][i],
                    bbox = data["calibration_markers"]["bbox"][i],
                    #moments = data["calibration_markers"]["moments"][i],
                    touch = data["calibration_markers"]["touch"][i],
                    perimeter = data["calibration_markers"]["perimeter"][i],
                    a = data["calibration_markers"]["a"][i],
                    b = data["calibration_markers"]["b"][i],
                    orientation = data["calibration_markers"]["orientation"][i],
                    children = data["calibration_markers"]["children"][i],
                    parent = data["calibration_markers"]["parent"][i],
                    uc = data["calibration_markers"]["uc"][i],
                    vc = data["calibration_markers"]["vc"][i],
                    level = data["calibration_markers"]["level"][i],                    
                )
                calibration_blobs.append(new_blob)

        for b1 in calibration_blobs:
            for b2 in calibration_markers:
                if np.abs(b1.uc-b2.uc) < 5 and np.abs(b1.vc-b2.vc) < 5:
                    break
            else:
                print("At least 1 blob not located properly")
        return calibration_blobs

    def REF_LocateShapes(self,img:mvt.Image):
        """
        Locates shapes in the provided image.

        Parameters
        ----------
        img
            An mvt.Image object containing the shapes to be located.
        
        Returns
        -------
        shapes
            A Python dictionary containing mvt blobs of each located shape.
            Calibration markers will be a list of mvt blobs.
        """
        self.REF_LocateShapes_used = True

        class Blob:
            def __init__(
                self,
                id = None,
                bbox = None,
                moments = None,
                touch = None,
                perimeter = None,
                a = None,
                b = None,
                orientation = None,
                children = None,
                parent = None,
                uc = None,
                vc = None,
                level = None,
            ):
                self.id = id
                self.bbox = bbox
                self.moments = moments
                self.touch = touch
                self.perimeter = perimeter
                self.a = a
                self.b = b
                self.orientation = orientation
                self.children = children
                self.parent = parent
                self.uc = uc
                self.vc = vc
                self.level = level
        
        blobs = mvt.Blobs()
        calibration_blobs = []
        with open("reference values.pkl","rb") as fp:
            data = pickle.load(fp)
            for i in range(5):
                new_blob = Blob(
                    id = data["id"][i],
                    bbox = data["bbox"][i],
                    #moments = data["moments"][i],
                    touch = data["touch"][i],
                    perimeter = data["perimeter"][i],
                    a = data["a"][i],
                    b = data["b"][i],
                    orientation = data["orientation"][i],
                    children = data["children"][i],
                    parent = data["parent"][i],
                    uc = data["uc"][i],
                    vc = data["vc"][i],
                    level = data["level"][i],                    
                )
                blobs.data.append(new_blob)
            for i in range(4):
                new_blob = Blob(
                    id = data["calibration_markers"]["id"][i],
                    bbox = data["calibration_markers"]["bbox"][i],
                    #moments = data["calibration_markers"]["moments"][i],
                    touch = data["calibration_markers"]["touch"][i],
                    perimeter = data["calibration_markers"]["perimeter"][i],
                    a = data["calibration_markers"]["a"][i],
                    b = data["calibration_markers"]["b"][i],
                    orientation = data["calibration_markers"]["orientation"][i],
                    children = data["calibration_markers"]["children"][i],
                    parent = data["calibration_markers"]["parent"][i],
                    uc = data["calibration_markers"]["uc"][i],
                    vc = data["calibration_markers"]["vc"][i],
                    level = data["calibration_markers"]["level"][i],                    
                )
                calibration_blobs.append(new_blob)
        random.shuffle(calibration_blobs)
        shapes = {
            "red square"            : blobs[0],
            "green square"          : blobs[1],
            "blue square"           : blobs[3],
            "green circle"          : blobs[2],
            "blue circle"           : blobs[4],
            "calibration markers"   : calibration_blobs,
        }
        return shapes

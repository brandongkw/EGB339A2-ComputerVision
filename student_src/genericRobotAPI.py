'''
This file contains the GenericRobotAPI class.
The CoppeliaRobot class inherits from this class, and so does the real Dobot class.
There is no need for you to touch this file, however, it provides you with a list
of all the functions that you can use to control the robot.
'''
class GenericRobotAPI(object):
    def __init__(self):
        print("[GenericRobotAPI] connect to robot.")

    def __del__(self):
        print("[GenericRobotAPI] disconnect from robot.")

    def move_arm(self, j1: float, j2: float, j3: float):
        if not all(isinstance(x, (int, float)) for x in [j1, j2, j3]):
            raise Exception("All parameters for move_arm() must be integers or floats")
        self.check_joint_config(j1, j2, j3)
        print(f"[GenericRobotAPI] is pretending to move to {[j1, j2, j3]} [rads]")
        # API.move_to_joint_config(self.apiHandle, j1, j2, j3)

    def get_joint_config(self) -> list:
        print("[GenericRobotAPI] is pretending to get the current joint configuration; returning fake values")
        return [0, 0, 0]
    
    def home(self):
        print("[GenericRobotAPI] is pretending to home the robot")

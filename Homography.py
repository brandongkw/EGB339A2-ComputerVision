import csv
import cv2
import numpy as np
from picamera2 import Picamera2

"""
Homography module for loading homography matrix, capturing images, and calibrating the homography matrix.
"""



class Homography:
    def __init__(self):
        self.homography_matrix = None

    def load_homography_matrix(self, csv_file):
        self.homography_matrix = []
        with open(csv_file, mode='r') as file:
            csv_reader = csv.reader(file)
            next(csv_reader)  # Skip header
            for row in csv_reader:
                self.homography_matrix.append([float(val) for val in row])
        self.homography_matrix = np.array(self.homography_matrix)
        return self.homography_matrix

class CalibrateHomography:
    # Define the angle in degrees
    angle_deg = 30

    # Convert the angle to radians
    angle_rad = np.radians(angle_deg)

    ground_points = np.array([
        [-0.2, 0.36],
        [0.215, 0.365],
        [0.2, 0.90],
        [-0.215, 1.29]
    ], dtype=np.float32)
    def __init__(self):
        # Global variables for calibration
        self.homography_matrix = None
        self.image_points = []
        self.found_homography = False
        self.captured_image = None

    def draw_crosshair(self, frame, color=(255, 255, 255), thickness=2):
        # Get the dimensions of the frame
        height, width = frame.shape[:2]
        
        # Calculate the center of the frame
        center_x = width // 2
        center_y = height // 2
        FrameCenter = (center_x, center_y)
        # Define the length of the crosshair arms
        crosshair_length = 5
        
        # Draw the vertical line of the crosshair

        cv2.drawMarker(frame, FrameCenter, color, markerType=cv2.MARKER_CROSS, markerSize=10, thickness=2)
        # Draw the horizontal line of the crosshair
        return FrameCenter

    # Function to handle mouse click events for homography calibration
    def onClick(self, event, x, y, flags, params):
        if event == cv2.EVENT_LBUTTONDOWN:
            print(f"Clicked coordinates: ({x}, {y})")
            if not self.found_homography:
                self.image_points.append([x, y])
                if len(self.image_points) == 4:
                    # Convert to NumPy array of type float32
                    image_points_array = np.array(self.image_points, dtype=np.float32)
                    # Compute the homography matric
                    self.homography_matrix, _ = cv2.findHomography(image_points_array.reshape(-1, 1, 2), self.ground_points.reshape(-1, 1, 2))
                    self.found_homography = True
                    print("Homography matrix found:")
                    print(self.homography_matrix)
            else:
                print("Homography already calibrated.")

    # Function to save homography matrix to a CSV file
    def save_homography(self, homography_matrix):
        with open("calibrate_homography.csv", "w", newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["Homography Matrix"])
            for row in homography_matrix:
                writer.writerow(row)
        print("Homography matrix saved to calibrate_homography.csv")

    # Function to resize the captured image
    def resize_image(self, image, scale=0.5):
        return cv2.resize(image, (0, 0), fx=scale, fy=scale)

    def calibrate(self):
        global captured_image

        # Initialize the Picamera2 feed
        picam2 = Picamera2()
        picam2.configure(picam2.create_preview_configuration(main={"format": "XRGB8888", "size": (820, 616)}))  # Set a larger resolution
        picam2.start()
        
        while True:
            frame = picam2.capture_array()
            frame = cv2.flip(frame, -1)  # Flip horizontally and vertically
            # frame = self.draw_crosshair(frame)
            cv2.imshow("Live Feed", frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('c'):  # Capture the image if 'c' is pressed
                captured_image = frame.copy()
                captured_image = self.resize_image(captured_image)  # Resize the captured image
                print("Image captured!")
                cv2.destroyWindow("Live Feed")
                break

        # If an image was captured, display it in a new window and perform calibration steps
        if captured_image is not None:
            cv2.imshow("Captured Image", captured_image)
            cv2.setMouseCallback("Captured Image", self.onClick, captured_image)

            while True:
                key = cv2.waitKey(1) & 0xFF

                if key == ord('q'):  # Exit the loop if 'q' is pressed
                    break
                elif key == ord('s'):  # Save the homography matrix if 's' is pressed
                    if self.homography_matrix is not None:
                        self.save_homography(self.homography_matrix)
                        print("Calibration data saved.")
                    else:
                        print("Ensure the homography matrix is calibrated before saving.")

        picam2.close()
        cv2.destroyAllWindows()

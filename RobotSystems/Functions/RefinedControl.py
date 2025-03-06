#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/ArmPi/')
import cv2
import numpy as np
import time
import threading
from LABConfig import *
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board
from CameraCalibration.CalibrationConfig import *
import Camera
from RefinedTracking import ExtendedBlockDetector
from RefinedGrabbing import MotionController

# Set operation mode: "sorting" or "stacking"
OPERATION_MODE = "sorting"  # Change as needed

# Define LAB color ranges (update these with your calibration values)
color_range = {
    'red':   (np.array([20, 150, 150]), np.array([255, 200, 200])),
    'green': (np.array([20, 80, 40]),   np.array([110, 255, 150])),
    'blue':  (np.array([20, 70, 150]),  np.array([110, 150, 255])),
}
target_colors = ('red', 'green', 'blue')
square_length = 10

# Instantiate the detector and motion controller
detector = ExtendedBlockDetector(target_colors, color_range, square_length, mode=OPERATION_MODE)
motion = MotionController(mode=OPERATION_MODE)
motion.init_arm()

# Open the camera
my_camera = Camera.Camera()
my_camera.camera_open()

print("Automatic protocol executing upon stable detection; press ESC to exit.")
while True:
    frame = my_camera.frame
    if frame is not None:
        annotated = detector.process_frame(frame, size=(640, 480))
        cv2.imshow("Integrated Operation", annotated)
        # You may use either stable or immediate detection results
        _, world_coords, detected_color, rotation_angle = detector.detect_block(frame, size=(640, 480))
        if world_coords is not None and detected_color is not None:
            if OPERATION_MODE == "sorting":
                motion.perform_pick_and_place(world_coords, detected_color, rotation_angle)
            elif OPERATION_MODE == "stacking":
                motion.perform_stack(world_coords, detected_color, rotation_angle)
            time.sleep(2)
    if cv2.waitKey(1) == 27:
        break

my_camera.camera_close()
cv2.destroyAllWindows()

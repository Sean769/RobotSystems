#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/ArmPi/')
import cv2
import time
import Camera
import threading
import numpy as np
import math

from LABConfig import *
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board
from CameraCalibration.CalibrationConfig import *

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

AK = ArmIK()

# ---------------- Arm and Board Initialization ----------------
servo1 = 500

def initMove():
    Board.setBusServoPulse(1, servo1 - 50, 300)
    Board.setBusServoPulse(2, 500, 500)
    AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)

def init():
    print("Circle Detection Init")
    initMove()

def start():
    global __isRunning
    __isRunning = True
    print("Circle Detection Start")

def stop():
    global __isRunning
    __isRunning = False
    print("Circle Detection Stop")

def exit():
    global __isRunning
    __isRunning = False
    print("Circle Detection Exit")

# Global flag for running status
__isRunning = False

# ---------------- Minimal Arm-Control Thread ----------------
# The original move thread is maintained for library usage,
# but its behavior is now minimal (no pick-up actions).
def move():
    while True:
        if __isRunning:
            # The arm remains in its initial position.
            time.sleep(0.01)
        else:
            time.sleep(0.01)

# Start the move thread
th = threading.Thread(target=move)
th.setDaemon(True)
th.start()

# ---------------- Vision Functions ----------------
def draw_smiley_face(img, center, radius):
    """
    Draws a smiley face inside a circle.
    :param img: Image on which to draw.
    :param center: Tuple (x, y) for circle center.
    :param radius: Radius of the detected circle.
    """
    x, y = center
    # Draw the face outline (circle)
    cv2.circle(img, (x, y), radius, (0, 255, 0), 2)
    
    # Define eye parameters relative to face size
    eye_radius = max(1, radius // 8)
    eye_offset_x = radius // 3
    eye_offset_y = radius // 3
    
    # Draw eyes (filled circles)
    cv2.circle(img, (x - eye_offset_x, y - eye_offset_y), eye_radius, (0, 0, 0), -1)
    cv2.circle(img, (x + eye_offset_x, y - eye_offset_y), eye_radius, (0, 0, 0), -1)
    
    # Draw smiling mouth as an elliptical arc
    cv2.ellipse(img, (x, y + radius // 8), (radius // 2, radius // 2), 0, 20, 160, (0, 0, 0), 2)
    
    return img

def run(img):
    """
    Processes the camera frame to detect a circle and draw a smiley face inside it.
    """
    # Create a copy of the image to work on
    img_copy = img.copy()
    
    # Convert image to grayscale and blur to reduce noise
    gray = cv2.cvtColor(img_copy, cv2.COLOR_BGR2GRAY)
    gray_blurred = cv2.medianBlur(gray, 5)
    
    # Detect circles using Hough Circle Transform
    circles = cv2.HoughCircles(gray_blurred,
                               cv2.HOUGH_GRADIENT,
                               dp=1.2,
                               minDist=50,
                               param1=50,
                               param2=30,
                               minRadius=20,
                               maxRadius=0)
    
    # If a circle is detected, draw a smiley face in the first circle found
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for circle in circles[0, :]:
            x, y, r = circle
            img_copy = draw_smiley_face(img_copy, (x, y), r)
            break  # Process only the first detected circle
    return img_copy

# ---------------- Main Loop ----------------
if __name__ == '__main__':
    init()
    start()
    my_camera = Camera.Camera()
    my_camera.camera_open()
    while True:
        img = my_camera.frame
        if img is not None:
            frame = img.copy()
            Frame = run(frame)
            cv2.imshow('Frame', Frame)
            key = cv2.waitKey(1)
            if key == 27:  # ESC key to exit
                break
    my_camera.camera_close()
    cv2.destroyAllWindows()

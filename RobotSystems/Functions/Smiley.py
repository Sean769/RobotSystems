#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/ArmPi/')
import cv2
import time
import threading
import numpy as np
import math
import Camera

from LABConfig import *
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board
from CameraCalibration.CalibrationConfig import *  # Assumes definitions for square_length and convertCoordinate

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

AK = ArmIK()

# ---------------- Global Variables and Flags ----------------
servo1 = 500    # Base servo value (for pen up/down)
__isRunning = False
drawing_in_progress = False  # Prevent re-triggering drawing

# ---------------- Arm and Board Initialization ----------------
def initMove():
    Board.setBusServoPulse(1, servo1 - 50, 300)
    Board.setBusServoPulse(2, 500, 500)
    AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)

def init():
    print("Circle Detection & Drawing Init")
    initMove()

def start():
    global __isRunning
    __isRunning = True
    print("Circle Detection & Drawing Start")

def stop():
    global __isRunning
    __isRunning = False
    print("Circle Detection & Drawing Stop")

def exit_program():
    global __isRunning
    __isRunning = False
    print("Circle Detection & Drawing Exit")

# ---------------- Minimal Arm-Control Thread ----------------
def move():
    while True:
        if __isRunning:
            time.sleep(0.01)
        else:
            time.sleep(0.01)

th = threading.Thread(target=move)
th.setDaemon(True)
th.start()

# ---------------- Vision Functions ----------------
def draw_smiley_face(img, center, radius):
    """
    Overlays a smiley face on the image for visualization.
    """
    x, y = center
    cv2.circle(img, (x, y), radius, (0, 255, 0), 2)
    eye_radius = max(1, radius // 8)
    eye_offset = radius // 3
    cv2.circle(img, (x - eye_offset, y - eye_offset), eye_radius, (0, 0, 0), -1)
    cv2.circle(img, (x + eye_offset, y - eye_offset), eye_radius, (0, 0, 0), -1)
    cv2.ellipse(img, (x, y + radius // 8), (radius // 2, radius // 2), 0, 20, 160, (0, 0, 0), 2)
    return img

# ---------------- Arm Drawing Routine ----------------
def arm_draw_smiley(world_center, r_pixels):
    """
    Moves the robot arm to the detected circle’s world coordinates and draws the smiley:
    two eye dots and a smiling arc.
    """
    global drawing_in_progress
    drawing_in_progress = True
    safe_height = 12    # Safe height above drawing surface (cm)
    draw_height = 1.5   # Height at which drawing occurs (cm)

    # Helper functions for pen control
    def pen_up():
        Board.setBusServoPulse(1, servo1 - 70, 300)
        time.sleep(0.5)
    def pen_down():
        Board.setBusServoPulse(1, servo1, 500)
        time.sleep(0.5)

    print("Starting drawing at world coordinates:", world_center)

    # 1. Move above the circle center
    target = (world_center[0], world_center[1], safe_height)
    result = AK.setPitchRangeMoving(target, -90, -90, 0)
    if result:
        time.sleep(result[2] / 1000)

    # 2. Lower to drawing height at center
    target = (world_center[0], world_center[1], draw_height)
    result = AK.setPitchRangeMoving(target, -90, -90, 0)
    if result:
        time.sleep(result[2] / 1000)

    # 3. Draw left eye dot
    # Calculate offset: conversion from pixels to cm using square_length (cm per pixel)
    eye_offset_pixels = r_pixels // 3
    eye_offset_world = eye_offset_pixels * square_length
    left_eye = (world_center[0] - eye_offset_world, world_center[1] - eye_offset_world, draw_height)
    result = AK.setPitchRangeMoving(left_eye, -90, -90, 0)
    if result:
        time.sleep(result[2] / 1000)
    pen_down()   # Mark the dot
    time.sleep(0.5)
    pen_up()

    # 4. Return to center
    target = (world_center[0], world_center[1], draw_height)
    result = AK.setPitchRangeMoving(target, -90, -90, 0)
    if result:
        time.sleep(result[2] / 1000)

    # 5. Draw right eye dot
    right_eye = (world_center[0] + eye_offset_world, world_center[1] - eye_offset_world, draw_height)
    result = AK.setPitchRangeMoving(right_eye, -90, -90, 0)
    if result:
        time.sleep(result[2] / 1000)
    pen_down()   # Mark the dot
    time.sleep(0.5)
    pen_up()

    # 6. Draw the smile (arc)
    # Define the smile’s center and radius in world units
    smile_center = (world_center[0], world_center[1] + (r_pixels // 8) * square_length)
    smile_radius_world = (r_pixels // 2) * square_length

    # Return to center before drawing the smile
    target = (world_center[0], world_center[1], draw_height)
    result = AK.setPitchRangeMoving(target, -90, -90, 0)
    if result:
        time.sleep(result[2] / 1000)
    pen_down()

    num_points = 10
    for i in range(num_points + 1):
        angle_deg = 20 + (140 * i / num_points)  # from 20° to 160°
        angle_rad = math.radians(angle_deg)
        x = smile_center[0] + smile_radius_world * math.cos(angle_rad)
        y = smile_center[1] + smile_radius_world * math.sin(angle_rad)
        target = (x, y, draw_height)
        result = AK.setPitchRangeMoving(target, -90, -90, 0)
        if result:
            time.sleep(result[2] / 1000)
    time.sleep(0.5)
    pen_up()

    # 7. Raise the arm to safe height
    target = (world_center[0], world_center[1], safe_height)
    result = AK.setPitchRangeMoving(target, -90, -90, 0)
    if result:
        time.sleep(result[2] / 1000)

    print("Finished drawing smiley.")
    drawing_in_progress = False

def run(img):
    """
    Processes the camera frame to detect a circle and overlay a smiley face.
    If a circle is detected and no drawing is in progress, convert the circle center
    to world coordinates and trigger the arm drawing routine.
    """
    global drawing_in_progress
    img_copy = img.copy()
    gray = cv2.cvtColor(img_copy, cv2.COLOR_BGR2GRAY)
    gray_blurred = cv2.medianBlur(gray, 5)

    circles = cv2.HoughCircles(gray_blurred,
                               cv2.HOUGH_GRADIENT,
                               dp=1.2,
                               minDist=50,
                               param1=50,
                               param2=30,
                               minRadius=20,
                               maxRadius=0)
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for circle in circles[0, :]:
            x, y, r = circle
            img_copy = draw_smiley_face(img_copy, (x, y), r)
            # Trigger drawing only if not already drawing
            if not drawing_in_progress:
                # Convert the detected circle center from image to world coordinates.
                world_x, world_y = convertCoordinate(x, y, (img.shape[1], img.shape[0]))
                threading.Thread(target=arm_draw_smiley, args=((world_x, world_y), r)).start()
            break
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
            processed_frame = run(frame)
            cv2.imshow('Frame', processed_frame)
            key = cv2.waitKey(1)
            if key == 27:  # ESC key to exit
                break
    my_camera.camera_close()
    cv2.destroyAllWindows()

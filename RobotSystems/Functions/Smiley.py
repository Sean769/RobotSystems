#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/ArmPi/')
import cv2
import time
import threading
import numpy as np
import math

from LABConfig import *
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board
from CameraCalibration.CalibrationConfig import *  # Must provide square_length and convertCoordinate

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

AK = ArmIK()

# ---------------- Global Variables ----------------
servo1 = 500   # Base value for the pen (or jaw) control
__isRunning = False
_stop = False
unreachable = False
get_roi = False
# Here, detect_color is used as a flag (set to "circle") once a circle is detected.
detect_color = 'None'
start_pick_up = False  # Set to True when the detected circle is stable and ready for drawing
rotation_angle = 0     # (Unused in this example, but kept for structure compatibility)
world_X, world_Y = 0, 0  # World coordinates of the detected circle center
circle_radius = 0      # Detected circle radius (in pixels) for scaling drawing features
drawing_in_progress = False  # When True, detection is locked until drawing completes

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

# ---------------- Arm-Control Thread ----------------
def move():
    global _stop, get_roi, unreachable, __isRunning, detect_color, start_pick_up
    global rotation_angle, world_X, world_Y, circle_radius, drawing_in_progress
    while True:
        if __isRunning:
            if detect_color != 'None' and start_pick_up and not drawing_in_progress:
                # Lock detection so that new circles are ignored until drawing is done.
                drawing_in_progress = True
                print("Drawing sequence started...")

                # --- 1. Move above the circle center (safe height: 7 cm) ---
                if not __isRunning:
                    continue
                Board.setBusServoPulse(2, 500, 500)
                result = AK.setPitchRangeMoving((world_X, world_Y, 7), -90, -90, 0)
                if result:
                    time.sleep(result[2] / 1000)
                
                # --- 2. Lower to drawing height (1.5 cm) ---
                if not __isRunning:
                    continue
                result = AK.setPitchRangeMoving((world_X, world_Y, 1.5), -90, -90, 0, 1000)
                if result:
                    time.sleep(result[2] / 1000)
                
                # --- 3. Draw left eye ---
                if not __isRunning:
                    continue
                eye_offset_pixels = circle_radius // 3
                eye_offset_world = eye_offset_pixels * square_length
                left_eye = (world_X - eye_offset_world, world_Y - eye_offset_world, 1.5)
                result = AK.setPitchRangeMoving(left_eye, -90, -90, 0, 1000)
                if result:
                    time.sleep(result[2] / 1000)
                Board.setBusServoPulse(1, servo1, 500)   # Pen down (simulate drawing a dot)
                time.sleep(0.5)
                Board.setBusServoPulse(1, servo1 - 70, 300)  # Pen up
                time.sleep(0.3)
                
                # --- 4. Return to center ---
                if not __isRunning:
                    continue
                result = AK.setPitchRangeMoving((world_X, world_Y, 1.5), -90, -90, 0, 1000)
                if result:
                    time.sleep(result[2] / 1000)
                
                # --- 5. Draw right eye ---
                if not __isRunning:
                    continue
                right_eye = (world_X + eye_offset_world, world_Y - eye_offset_world, 1.5)
                result = AK.setPitchRangeMoving(right_eye, -90, -90, 0, 1000)
                if result:
                    time.sleep(result[2] / 1000)
                Board.setBusServoPulse(1, servo1, 500)   # Pen down
                time.sleep(0.5)
                Board.setBusServoPulse(1, servo1 - 70, 300)  # Pen up
                time.sleep(0.3)
                
                # --- 6. Return to center before drawing the smile ---
                if not __isRunning:
                    continue
                result = AK.setPitchRangeMoving((world_X, world_Y, 1.5), -90, -90, 0, 1000)
                if result:
                    time.sleep(result[2] / 1000)
                
                # --- 7. Draw the smile (arc) ---
                if not __isRunning:
                    continue
                smile_center = (world_X, world_Y + (circle_radius // 8) * square_length)
                smile_radius_world = (circle_radius // 2) * square_length
                Board.setBusServoPulse(1, servo1, 500)  # Pen down for smile
                time.sleep(0.5)
                num_points = 10
                for i in range(num_points + 1):
                    if not __isRunning:
                        continue
                    angle_deg = 20 + (140 * i / num_points)  # from 20° to 160°
                    angle_rad = math.radians(angle_deg)
                    x = smile_center[0] + smile_radius_world * math.cos(angle_rad)
                    y = smile_center[1] + smile_radius_world * math.sin(angle_rad)
                    result = AK.setPitchRangeMoving((x, y, 1.5), -90, -90, 0, 1000)
                    if result:
                        time.sleep(result[2] / 1000)
                time.sleep(0.5)
                Board.setBusServoPulse(1, servo1 - 70, 300)  # Pen up after smile
                time.sleep(0.3)
                
                # --- 8. Raise arm to safe height ---
                if not __isRunning:
                    continue
                result = AK.setPitchRangeMoving((world_X, world_Y, 7), -90, -90, 0, 1000)
                if result:
                    time.sleep(result[2] / 1000)
                
                # --- 9. Reset detection flags ---
                detect_color = 'None'
                start_pick_up = False
                drawing_in_progress = False
                print("Drawing sequence completed.")
            else:
                time.sleep(0.01)
        else:
            if _stop:
                _stop = False
                Board.setBusServoPulse(1, servo1 - 70, 300)
                time.sleep(0.5)
                Board.setBusServoPulse(2, 500, 500)
                AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)
                time.sleep(1.5)
            time.sleep(0.01)

# Start the move thread
th = threading.Thread(target=move)
th.setDaemon(True)
th.start()

# ---------------- Vision Functions ----------------
def draw_smiley_overlay(img, center, radius):
    """
    Draws a smiley face overlay on the image for visualization.
    """
    x, y = center
    cv2.circle(img, (x, y), radius, (0, 255, 0), 2)
    eye_radius = max(1, radius // 8)
    eye_offset = radius // 3
    cv2.circle(img, (x - eye_offset, y - eye_offset), eye_radius, (0, 0, 0), -1)
    cv2.circle(img, (x + eye_offset, y - eye_offset), eye_radius, (0, 0, 0), -1)
    cv2.ellipse(img, (x, y + radius // 8), (radius // 2, radius // 2), 0, 20, 160, (0, 0, 0), 2)
    return img

def run(img):
    """
    Processes the camera frame to detect a circle.
    When a circle is stably detected and no drawing is in progress,
    its center and radius are converted to world coordinates and the global
    flags are set so that the move() thread triggers the drawing sequence.
    """
    global detect_color, start_pick_up, world_X, world_Y, circle_radius, drawing_in_progress
    # If a drawing sequence is in progress, do not update detection.
    if drawing_in_progress:
        return img
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
        # Process the first detected circle
        for circle in circles[0, :]:
            x, y, r = circle
            img_copy = draw_smiley_overlay(img_copy, (x, y), r)
            # Lock this detection: convert image coordinates to world coordinates.
            world_x, world_y = convertCoordinate(x, y, (img.shape[1], img.shape[0]))
            world_X, world_Y = world_x, world_y
            circle_radius = r
            detect_color = 'circle'
            start_pick_up = True
            break
    else:
        detect_color = 'None'
    return img_copy

# ---------------- Main Loop ----------------
if __name__ == '__main__':
    init()
    start()
    my_camera = __import__("Camera").Camera()
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

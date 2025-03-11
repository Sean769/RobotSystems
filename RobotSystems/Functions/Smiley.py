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
from CameraCalibration.CalibrationConfig import *  # Assumes square_length and convertCoordinate are defined here

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

AK = ArmIK()

# ---------------- Global Variables and Flags ----------------
servo1 = 500  # Base servo value for the drawing tool
__isRunning = False  # Global flag for arm-control thread
draw_triggered = False  # Ensure drawing is triggered only once per stable detection

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

def exit():
    global __isRunning
    __isRunning = False
    print("Circle Detection & Drawing Exit")

# ---------------- Minimal Arm-Control Thread ----------------
# We keep the move thread active for library usage.
def move():
    while True:
        if __isRunning:
            # Currently no continuous arm motion needed.
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
    # Draw face outline
    cv2.circle(img, (x, y), radius, (0, 255, 0), 2)
    
    # Calculate eye parameters (relative to the detected circle size)
    eye_radius = max(1, radius // 8)
    eye_offset = radius // 3
    # Draw eyes as filled circles
    cv2.circle(img, (x - eye_offset, y - eye_offset), eye_radius, (0, 0, 0), -1)
    cv2.circle(img, (x + eye_offset, y - eye_offset), eye_radius, (0, 0, 0), -1)
    # Draw the smiling mouth as an arc
    cv2.ellipse(img, (x, y + radius // 8), (radius // 2, radius // 2), 0, 20, 160, (0, 0, 0), 2)
    
    return img

def arm_draw_smiley(world_center, r_pixels):
    """
    Moves the robot arm to the circle (world_center) and draws the smiley's eyes and smile.
    The conversion from pixel to world offsets is done using the calibration factor 'square_length'.
    """
    # Define drawing parameters
    safe_height = 12    # Safe height above the drawing surface in cm
    draw_height = 1.5   # Height at which to draw (touching the surface)
    
    # Helper functions for controlling the pen (assumed to be controlled by servo1)
    def pen_up():
        Board.setBusServoPulse(1, servo1 - 70, 300)
        time.sleep(0.5)
    
    def pen_down():
        Board.setBusServoPulse(1, servo1, 500)
        time.sleep(0.5)
    
    # 1. Move above the circle center
    target = (world_center[0], world_center[1], safe_height)
    result = AK.setPitchRangeMoving(target, -90, -90, 0)
    if result:
        time.sleep(result[2] / 1000)
    
    # 2. Move down to drawing height at the center
    target = (world_center[0], world_center[1], draw_height)
    result = AK.setPitchRangeMoving(target, -90, -90, 0)
    if result:
        time.sleep(result[2] / 1000)
    
    pen_down()  # Lower the pen to contact the drawing surface

    # 3. Compute offsets for the eyes
    # Using a simple conversion: pixel offset * square_length (cm per pixel)
    eye_offset_pixels = r_pixels // 3
    eye_offset_world = eye_offset_pixels * square_length
    
    left_eye = (world_center[0] - eye_offset_world, world_center[1] - eye_offset_world, draw_height)
    right_eye = (world_center[0] + eye_offset_world, world_center[1] - eye_offset_world, draw_height)
    
    # Draw left eye dot:
    target = left_eye
    result = AK.setPitchRangeMoving(target, -90, -90, 0)
    if result:
        time.sleep(result[2] / 1000)
    time.sleep(0.5)  # Pause to simulate a dot
    pen_up()      # Lift pen to separate the dots
    time.sleep(0.3)
    
    # Return to center before drawing the right eye
    target = (world_center[0], world_center[1], draw_height)
    result = AK.setPitchRangeMoving(target, -90, -90, 0)
    if result:
        time.sleep(result[2] / 1000)
    pen_down()
    
    # Draw right eye dot:
    target = right_eye
    result = AK.setPitchRangeMoving(target, -90, -90, 0)
    if result:
        time.sleep(result[2] / 1000)
    time.sleep(0.5)
    pen_up()
    time.sleep(0.3)
    
    # 4. Draw the smile (an arc)
    # Compute the smile arc center and radius in world coordinates.
    # The smile center is offset downward from the circle center.
    smile_center = (world_center[0], world_center[1] + (r_pixels // 8) * square_length)
    smile_radius_world = (r_pixels // 2) * square_length
    # Return to center to start drawing the smile
    target = (world_center[0], world_center[1], draw_height)
    result = AK.setPitchRangeMoving(target, -90, -90, 0)
    if result:
        time.sleep(result[2] / 1000)
    pen_down()
    
    # Draw the smile as an arc from 20 to 160 degrees (sampled in 10 segments)
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
    
    # 5. Return to a safe height above the drawing area
    target = (world_center[0], world_center[1], safe_height)
    result = AK.setPitchRangeMoving(target, -90, -90, 0)
    if result:
        time.sleep(result[2] / 1000)
    print("Finished drawing smiley on the board.")

def run(img):
    """
    Processes each camera frame to detect a circle and overlay a smiley face.
    When a circle is stably detected, it converts its center to world coordinates
    and triggers the arm to draw the smiley (eyes and smile).
    """
    global draw_triggered
    img_copy = img.copy()
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
    
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for circle in circles[0, :]:
            x, y, r = circle
            # Overlay the smiley for visual feedback
            img_copy = draw_smiley_face(img_copy, (x, y), r)
            # Convert the detected circle center (x,y) from image to world coordinates.
            # (Assumes convertCoordinate(x, y, size) returns (world_x, world_y))
            world_x, world_y = convertCoordinate(x, y, (img.shape[1], img.shape[0]))
            # Only trigger the arm drawing once per stable detection
            if not draw_triggered:
                draw_triggered = True
                # Run the drawing routine in a separate thread so vision remains responsive.
                threading.Thread(target=arm_draw_smiley, args=((world_x, world_y), r)).start()
            break  # Only process the first detected circle
    else:
        draw_triggered = False  # Reset if no circle is detected
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

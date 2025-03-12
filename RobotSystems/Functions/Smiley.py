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
import Camera

servo1 = 500   # Base value for the pen (or gripper) control
AK = ArmIK()

# ----------------- Smiley Detector -----------------
class SmileyDetector:
    def __init__(self):
        self.size = (640, 480)   # Fixed image size
        self.current_shape = "None"  # "circle" when detected
        self.img_center = (0, 0)
        self.r = 0
        self.world_x = 0
        self.world_y = 0
        self.circle_radius = 0

    def process_frame(self, img):
        frame_resized = cv2.resize(img, self.size, interpolation=cv2.INTER_NEAREST)
        gray = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2GRAY)
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
                self.img_center = (x, y)
                self.r = r
                self.circle_radius = r
                world_x, world_y = convertCoordinate(x, y, self.size)
                self.world_x = world_x
                self.world_y = world_y
                self.current_shape = "circle"
                break
        else:
            self.current_shape = "None"
        return frame_resized

    def annotate_image(self, img):
        if self.current_shape == "circle":
            x, y = self.img_center
            r = self.r
            cv2.circle(img, (x, y), r, (0, 255, 0), 2)
            eye_offset = r // 3
            eye_radius = max(1, r // 8)
            cv2.circle(img, (x - eye_offset, y - eye_offset), eye_radius, (0, 0, 0), -1)
            cv2.circle(img, (x + eye_offset, y - eye_offset), eye_radius, (0, 0, 0), -1)
            cv2.ellipse(img, (x, y + r // 8), (r // 2, r // 2), 0, 20, 160, (0, 0, 0), 2)
        return img

# ----------------- Smiley Move Handler -----------------
class SmileyMoveHandler:
    def __init__(self, detector):
        self.detector = detector
        self.start_pick_up = False
        self.drawing_in_progress = False

    def move(self):
        while True:
            if self.detector.current_shape != "None" and self.start_pick_up and not self.drawing_in_progress:
                self.drawing_in_progress = True
                print("Drawing sequence started...")
                wx = self.detector.world_x
                wy = self.detector.world_y
                r = self.detector.circle_radius

                # 1. Move above the circle center (safe height: 7 cm)
                Board.setBusServoPulse(2, 500, 500)
                result = AK.setPitchRangeMoving((wx, wy, 7), -90, -90, 0, 1000)
                print("Step 1 (Move above):", (wx, wy, 7), "Result:", result)
                if result:
                    time.sleep(result[2] / 1000)

                # 2. Lower to drawing height (1.5 cm)
                result = AK.setPitchRangeMoving((wx, wy, 1.5), -90, -90, 0, 1000)
                print("Step 2 (Lower to drawing height):", (wx, wy, 1.5), "Result:", result)
                if result:
                    time.sleep(result[2] / 1000)

                # 3. Draw left eye
                eye_offset_pixels = r // 3
                eye_offset_world = eye_offset_pixels * square_length
                left_eye = (wx - eye_offset_world, wy - eye_offset_world, 1.5)
                result = AK.setPitchRangeMoving(left_eye, -90, -90, 0, 1000)
                print("Step 3 (Left eye):", left_eye, "Result:", result)
                if result:
                    time.sleep(result[2] / 1000)
                Board.setBusServoPulse(1, servo1, 500)   # Pen down
                time.sleep(0.5)
                Board.setBusServoPulse(1, servo1 - 70, 300)  # Pen up
                time.sleep(0.3)

                # 4. Return to center
                result = AK.setPitchRangeMoving((wx, wy, 1.5), -90, -90, 0, 1000)
                print("Step 4 (Return to center):", (wx, wy, 1.5), "Result:", result)
                if result:
                    time.sleep(result[2] / 1000)

                # 5. Draw right eye
                right_eye = (wx + eye_offset_world, wy - eye_offset_world, 1.5)
                result = AK.setPitchRangeMoving(right_eye, -90, -90, 0, 1000)
                print("Step 5 (Right eye):", right_eye, "Result:", result)
                if result:
                    time.sleep(result[2] / 1000)
                Board.setBusServoPulse(1, servo1, 500)   # Pen down
                time.sleep(0.5)
                Board.setBusServoPulse(1, servo1 - 70, 300)  # Pen up
                time.sleep(0.3)

                # 6. Return to center before drawing smile
                result = AK.setPitchRangeMoving((wx, wy, 1.5), -90, -90, 0, 1000)
                print("Step 6 (Return to center for smile):", (wx, wy, 1.5), "Result:", result)
                if result:
                    time.sleep(result[2] / 1000)

                # 7. Draw the smile (arc)
                smile_center = (wx, wy + (r // 8) * square_length)
                smile_radius_world = (r // 2) * square_length
                Board.setBusServoPulse(1, servo1, 500)  # Pen down
                time.sleep(0.5)
                num_points = 10
                for i in range(num_points + 1):
                    angle_deg = 20 + (140 * i / num_points)
                    angle_rad = math.radians(angle_deg)
                    x = smile_center[0] + smile_radius_world * math.cos(angle_rad)
                    y = smile_center[1] + smile_radius_world * math.sin(angle_rad)
                    result = AK.setPitchRangeMoving((x, y, 1.5), -90, -90, 0, 1000)
                    print("Step 7 (Smile point", i, "):", (x, y, 1.5), "Result:", result)
                    if result:
                        time.sleep(result[2] / 1000)
                time.sleep(0.5)
                Board.setBusServoPulse(1, servo1 - 70, 300)  # Pen up after smile
                time.sleep(0.3)

                # 8. Raise arm to safe height
                result = AK.setPitchRangeMoving((wx, wy, 7), -90, -90, 0, 1000)
                print("Step 8 (Raise arm):", (wx, wy, 7), "Result:", result)
                if result:
                    time.sleep(result[2] / 1000)

                # 9. Reset detection flags
                self.detector.current_shape = "None"
                self.start_pick_up = False
                self.drawing_in_progress = False
                print("Drawing sequence completed.")
            else:
                time.sleep(0.01)

# ----------------- Robot Position Printer -----------------
def print_robot_position():
    # This thread prints the robot's current position every 0.5 seconds.
    while True:
        try:
            pos = AK.getXYZ()  # Replace with your actual method to get robot position
            print("Current robot position:", pos)
        except Exception as e:
            print("Error reading robot position:", e)
        time.sleep(0.5)

# ----------------- Initialization and Main Loop -----------------
def initMove():
    Board.setBusServoPulse(1, servo1 - 50, 300)
    Board.setBusServoPulse(2, 500, 500)
    AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)

if __name__ == '__main__':
    initMove()

    detector = SmileyDetector()
    move_handler = SmileyMoveHandler(detector)

    move_thread = threading.Thread(target=move_handler.move)
    move_thread.daemon = True
    move_thread.start()

    pos_thread = threading.Thread(target=print_robot_position)
    pos_thread.daemon = True
    pos_thread.start()

    my_camera = Camera.Camera()
    my_camera.camera_open()

    while True:
        img = my_camera.frame
        if img is not None:
            processed_frame = detector.process_frame(img)
            annotated_img = detector.annotate_image(processed_frame)
            cv2.imshow('Smiley Detection', annotated_img)
            if detector.current_shape != "None":
                move_handler.start_pick_up = True
        if cv2.waitKey(1) == 27:
            break

    my_camera.camera_close()
    cv2.destroyAllWindows()

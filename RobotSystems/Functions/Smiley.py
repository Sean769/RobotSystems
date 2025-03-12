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
                r = self.detector.circle_radius  # radius in pixels

                # 1. Move above the circle center at safe height (7 cm)
                center_safe = (wx, wy, 7)
                Board.setBusServoPulse(2, 500, 500)
                result = AK.setPitchRangeMoving(center_safe, -90, -90, 0, 1000)
                print("Step 1 (Move above circle center):", center_safe, "Result:", result)
                time.sleep(1.0)

                # 2. Draw left eye:
                eye_offset_world = (r / 3.0) * map_param_
                left_eye_safe = (wx - eye_offset_world, wy - eye_offset_world, 7)
                left_eye_draw = (wx - eye_offset_world, wy - eye_offset_world, 1.5)
                # Navigate to left eye at safe height
                result = AK.setPitchRangeMoving(left_eye_safe, -90, -90, 0, 1000)
                print("Step 2 (Navigate to left eye safe):", left_eye_safe, "Result:", result)
                time.sleep(1.0)
                # Lower to drawing height at left eye and mark the dot
                result = AK.setPitchRangeMoving(left_eye_draw, -90, -90, 0, 1000)
                print("Step 3 (Lower to drawing height at left eye):", left_eye_draw, "Result:", result)
                time.sleep(1.0)
                # Raise back to safe height at left eye
                result = AK.setPitchRangeMoving(left_eye_safe, -90, -90, 0, 1000)
                print("Step 4 (Raise back to safe height at left eye):", left_eye_safe, "Result:", result)
                time.sleep(1.0)

                # 3. Draw right eye:
                # Return to circle center safe
                result = AK.setPitchRangeMoving(center_safe, -90, -90, 0, 1000)
                print("Step 5 (Return to circle center safe):", center_safe, "Result:", result)
                time.sleep(1.0)
                right_eye_safe = (wx + eye_offset_world, wy - eye_offset_world, 7)
                right_eye_draw = (wx + eye_offset_world, wy - eye_offset_world, 1.5)
                # Navigate to right eye at safe height
                result = AK.setPitchRangeMoving(right_eye_safe, -90, -90, 0, 1000)
                print("Step 6 (Navigate to right eye safe):", right_eye_safe, "Result:", result)
                time.sleep(1.0)
                # Lower to drawing height at right eye and mark the dot
                result = AK.setPitchRangeMoving(right_eye_draw, -90, -90, 0, 1000)
                print("Step 7 (Lower to drawing height at right eye):", right_eye_draw, "Result:", result)
                time.sleep(1.0)
                # Raise back to safe height at right eye
                result = AK.setPitchRangeMoving(right_eye_safe, -90, -90, 0, 1000)
                print("Step 8 (Raise back to safe height at right eye):", right_eye_safe, "Result:", result)
                time.sleep(1.0)

                # 4. Draw the smile:
                # Return to circle center safe before starting the smile
                result = AK.setPitchRangeMoving(center_safe, -90, -90, 0, 1000)
                print("Step 9 (Return to circle center safe for smile):", center_safe, "Result:", result)
                time.sleep(1.0)
                # Define smile arc parameters
                smile_center = (wx, wy + (r / 8.0) * map_param_)
                smile_radius_world = (r / 2.0) * map_param_
                num_points = 10
                # Determine the first smile point at safe height
                angle_deg = 20  # starting angle
                angle_rad = math.radians(angle_deg)
                first_smile_safe = (smile_center[0] + smile_radius_world * math.cos(angle_rad),
                                     smile_center[1] + smile_radius_world * math.sin(angle_rad),
                                     7)
                first_smile_draw = (first_smile_safe[0], first_smile_safe[1], 1.5)
                # Navigate to first smile point at safe height
                result = AK.setPitchRangeMoving(first_smile_safe, -90, -90, 0, 1000)
                print("Step 10 (Navigate to first smile point safe):", first_smile_safe, "Result:", result)
                time.sleep(1.0)
                # Lower to drawing height at first smile point
                result = AK.setPitchRangeMoving(first_smile_draw, -90, -90, 0, 1000)
                print("Step 11 (Lower to drawing height at first smile point):", first_smile_draw, "Result:", result)
                time.sleep(1.0)
                # Draw the remaining smile points while staying at drawing height
                for i in range(1, num_points + 1):
                    angle_deg = 20 + (140 * i / num_points)
                    angle_rad = math.radians(angle_deg)
                    draw_point = (smile_center[0] + smile_radius_world * math.cos(angle_rad),
                                  smile_center[1] + smile_radius_world * math.sin(angle_rad),
                                  1.5)
                    result = AK.setPitchRangeMoving(draw_point, -90, -90, 0, 1000)
                    print("Step 12 (Draw smile point", i, "at drawing height):", draw_point, "Result:", result)
                    time.sleep(1.0)
                # After finishing the smile, raise from the last smile point to safe height
                last_smile_safe = (draw_point[0], draw_point[1], 7)
                result = AK.setPitchRangeMoving(last_smile_safe, -90, -90, 0, 1000)
                print("Step 13 (Raise to safe height at end of smile):", last_smile_safe, "Result:", result)
                time.sleep(1.0)

                # 5. Final move: Return to final safe position (0, 0, 10)
                result = AK.setPitchRangeMoving((0, 0, 10), -90, -90, 0, 1000)
                print("Final move (Return to safe position 0,0,10):", (0, 0, 10), "Result:", result)
                time.sleep(1.0)

                # Reset detection flags
                self.detector.current_shape = "None"
                self.start_pick_up = False
                self.drawing_in_progress = False
                print("Drawing sequence completed.")
            else:
                time.sleep(0.01)

# ----------------- Initialization and Main Loop -----------------
def initMove():
    Board.setBusServoPulse(1, servo1 - 50, 300)
    Board.setBusServoPulse(2, 500, 500)
    AK.setPitchRangeMoving((0, 0, 10), 30, 30, 90, 1500)

if __name__ == '__main__':
    initMove()

    detector = SmileyDetector()
    move_handler = SmileyMoveHandler(detector)

    move_thread = threading.Thread(target=move_handler.move)
    move_thread.daemon = True
    move_thread.start()

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

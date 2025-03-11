#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/ArmPi/')
import cv2
import time
import Camera
import threading
from LABConfig import *
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board
from CameraCalibration.CalibrationConfig import *
import time
from Perception import ColorDetector

AK = ArmIK()
servo1 = 500

# initial position
def initMove():
    Board.setBusServoPulse(1, servo1 - 50, 300)
    Board.setBusServoPulse(2, 500, 500)
    AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)

def setBuzzer(timer):
    Board.setBuzzer(0)
    Board.setBuzzer(1)
    time.sleep(timer)
    Board.setBuzzer(0)
    
class MoveHandler:
    def __init__(self, color_tracker):
        self.color_tracker = color_tracker
        self.track = False
        self._stop = False
        self.get_roi = False
        self.unreachable = False
        self.detect_color = 'None'
        self.action_finish = True
        self.rotation_angle = 0
        self.world_X = 0
        self.world_Y = 0
        self.world_x = 0
        self.world_y = 0
        self.center_list = []
        self.count = 0
        self.start_pick_up = False
        self.first_move = True

    # Robotic arm moving thread
    def move(self):
        # Quick placement coordinates (x, y, z) of different colors of wood
        coordinate = {
            'red':   (-15 + 0.5, 12 - 0.5, 1.5),
            'green': (-15 + 0.5, 6 - 0.5,  1.5),
            'blue':  (-15 + 0.5, 0 - 0.5,  1.5),
        }
        while True:
            if self.color_tracker.current_color != "None":
                current_color = self.color_tracker.current_color
                desired_x, desired_y, desired_angle = self.color_tracker.last_x, self.color_tracker.last_y, self.color_tracker.rotation_angle
                result = AK.setPitchRangeMoving((desired_x, desired_y - 2, 5), -90, -90, 0) 
                if result:
                    time.sleep(result[2]/1000)
                    AK.setPitchRangeMoving((desired_x, desired_y - 2, 5), -90, -90, 0, 20)
                    time.sleep(0.02)                    
                    self.track = False
                    self.action_finish = False # Stop and exit flag detection
                    Board.setBusServoPulse(1, servo1 - 280, 500)  # Claws spread
                    # Calculate the angle by which the gripper needs to be rotated
                    servo2_angle = getAngle(desired_x, desired_y, desired_angle)
                    Board.setBusServoPulse(2, servo2_angle, 500)
                    time.sleep(0.8)
                    AK.setPitchRangeMoving((desired_x, desired_y, 2), -90, -90, 0, 1000)  # lower height
                    time.sleep(2)
                    Board.setBusServoPulse(1, servo1, 500)  # Gripper closed
                    time.sleep(1)
                    Board.setBusServoPulse(2, 500, 500)
                    AK.setPitchRangeMoving((desired_x, desired_y, 12), -90, -90, 0, 1000)  # Robotic arm raised
                    time.sleep(1)
                    
                    # Classify and place blocks of different colors
                    print(f"_______CURRENT COLOR: {current_color}")
                    result = AK.setPitchRangeMoving((coordinate[current_color][0], coordinate[current_color][1], 12), -90, -90, 0)   
                    time.sleep(result[2]/1000)
                    
                    servo2_angle = getAngle(coordinate[current_color][0], coordinate[current_color][1], -90)
                    Board.setBusServoPulse(2, servo2_angle, 500)
                    time.sleep(0.5)

                    AK.setPitchRangeMoving((coordinate[current_color][0], coordinate[current_color][1], coordinate[current_color][2] + 3), -90, -90, 0, 500)
                    time.sleep(0.5)
                    
                    AK.setPitchRangeMoving((coordinate[current_color]), -90, -90, 0, 1000)
                    time.sleep(0.8)
                    
                    Board.setBusServoPulse(1, servo1 - 200, 500)  # Open your claws and drop the object
                    time.sleep(0.8)
                                        
                    AK.setPitchRangeMoving((coordinate[current_color][0], coordinate[current_color][1], 12), -90, -90, 0, 800)
                    time.sleep(0.8)

                    initMove()  # Return to initial position
                    time.sleep(1.5)

                    self.detect_color = 'None'
                    self.color_tracker.current_color = 'None'
                    self.first_move = True
                    self.get_roi = False
                    self.action_finish = True
                    self.start_pick_up = False
                    self.color_tracker.last_x = 0
                    self.color_tracker.last_y = 0
                    self.color_tracker.rotation_angle = 0
                else:
                    print('debug location 3')
                    time.sleep(0.01)
                        

if __name__ == "__main__":
    detector = ColorDetector()
    my_camera = Camera.Camera()
    my_camera.camera_open()
    move_handler = MoveHandler(detector)

    
    move_thread = threading.Thread(target=move_handler.move)
    move_thread.daemon = True
    move_thread.start()
    
    while True:
        img = my_camera.frame
        if img is not None:
            frame = img.copy()
            img_processed = detector.preprocess_image(frame)
            box, detected_color_temp = detector.find_box_space(img_processed)
            annotated_img = detector.annotate_box(box, detected_color_temp)
            
            cv2.imshow('annotated image', annotated_img)
            
            if detected_color_temp:
                move_handler.detect_color = detected_color_temp
                move_handler.world_X, move_handler.world_Y = detector.world_x, detector.world_y
                move_handler.track = detector.track
                
                if move_handler.first_move:
                    move_handler.start_pick_up = True
            
            key = cv2.waitKey(1)
            if key == 27:
                break
    
    my_camera.camera_close()
    cv2.destroyAllWindows()
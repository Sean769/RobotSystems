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
from Perception import ColorDetector

AK = ArmIK()
servo1 = 500

# Initial position
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
    def __init__(self, color_tracker, mode='sort'):
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
        # Mode: 'sort' for sorting (fixed placement) or 'stack' for stacking (incremental height)
        self.mode = mode  
        if self.mode == 'stack':
            # Stacking placement coordinates (x, y, base z)
            self.place_coords = {
                'red':   (-15 + 1, -7 - 0.5, 1.5),
                'green': (-15 + 1, -7 - 0.5, 1.5),
                'blue':  (-15 + 1, -7 - 0.5, 1.5),
            }
            self.stack_increment = 2.5
            # For each color, maintain the current stacking height
            self.stack_heights = { color: self.place_coords[color][2] for color in self.place_coords }
            # Define a maximum stacking height (here base + 2 increments)
            self.stack_max = { color: self.place_coords[color][2] + 2*self.stack_increment for color in self.place_coords }
        else:
            # Sorting mode: fixed placement coordinates
            self.place_coords = {
                'red':   (-15 + 0.5, 12 - 0.5, 1.5),
                'green': (-15 + 0.5, 6 - 0.5,  1.5),
                'blue':  (-15 + 0.5, 0 - 0.5,  1.5),
            }

    # Robotic arm moving thread
    def move(self):
        while True:
            if self.color_tracker.current_color != "None":
                current_color = self.color_tracker.current_color
                desired_x = self.color_tracker.last_x
                desired_y = self.color_tracker.last_y
                desired_angle = self.color_tracker.rotation_angle

                # Move above the block to pick it up
                result = AK.setPitchRangeMoving((desired_x, desired_y - 2, 5), -90, -90, 0)
                if result:
                    time.sleep(result[2] / 1000)
                    AK.setPitchRangeMoving((desired_x, desired_y - 2, 5), -90, -90, 0, 20)
                    time.sleep(0.02)
                    
                    self.track = False
                    self.action_finish = False  # Stop further flag detection
                    Board.setBusServoPulse(1, servo1 - 280, 500)  # Open the claws
                    # Calculate gripper rotation
                    servo2_angle = getAngle(desired_x, desired_y, desired_angle)
                    Board.setBusServoPulse(2, servo2_angle, 500)
                    time.sleep(0.8)
                    # Lower to grasp the block
                    AK.setPitchRangeMoving((desired_x, desired_y, 2), -90, -90, 0, 1000)
                    time.sleep(2)
                    Board.setBusServoPulse(1, servo1, 500)  # Close the gripper
                    time.sleep(1)
                    Board.setBusServoPulse(2, 500, 500)
                    # Raise the arm with the block
                    AK.setPitchRangeMoving((desired_x, desired_y, 12), -90, -90, 0, 1000)
                    time.sleep(1)
                    
                    print(f"_______CURRENT COLOR: {current_color}")
                    if self.mode == 'stack':
                        # In stacking mode, use dynamic placement height
                        base = self.place_coords[current_color]
                        current_z = self.stack_heights[current_color]
                        result = AK.setPitchRangeMoving((base[0], base[1], 12), -90, -90, 0)
                        time.sleep(result[2] / 1000)
                        
                        servo2_angle = getAngle(base[0], base[1], -90)
                        Board.setBusServoPulse(2, servo2_angle, 500)
                        time.sleep(0.5)

                        # Move to a position above the stacking area (current height + offset)
                        AK.setPitchRangeMoving((base[0], base[1], current_z + 3), -90, -90, 0, 500)
                        time.sleep(0.5)
                        
                        # Lower to the current stacking height
                        AK.setPitchRangeMoving((base[0], base[1], current_z), -90, -90, 0, 1000)
                        time.sleep(0.8)
                        
                        Board.setBusServoPulse(1, servo1 - 200, 500)  # Open the claws to drop the block
                        time.sleep(0.8)
                                            
                        AK.setPitchRangeMoving((base[0], base[1], 12), -90, -90, 0, 800)
                        time.sleep(0.8)
                        
                        # Increment the stacking height for the current color
                        new_z = current_z + self.stack_increment
                        if new_z > self.stack_max[current_color]:
                            new_z = self.place_coords[current_color][2]  # Reset if maximum reached
                            # (Optionally, you can flag a warning that the stacking area is full)
                        self.stack_heights[current_color] = new_z
                    else:
                        # Sorting mode: use fixed placement coordinates
                        coord = self.place_coords[current_color]
                        result = AK.setPitchRangeMoving((coord[0], coord[1], 12), -90, -90, 0)
                        time.sleep(result[2] / 1000)
                        
                        servo2_angle = getAngle(coord[0], coord[1], -90)
                        Board.setBusServoPulse(2, servo2_angle, 500)
                        time.sleep(0.5)

                        AK.setPitchRangeMoving((coord[0], coord[1], coord[2] + 3), -90, -90, 0, 500)
                        time.sleep(0.5)
                        
                        AK.setPitchRangeMoving((coord), -90, -90, 0, 1000)
                        time.sleep(0.8)
                        
                        Board.setBusServoPulse(1, servo1 - 200, 500)  # Open the claws to drop the block
                        time.sleep(0.8)
                                            
                        AK.setPitchRangeMoving((coord[0], coord[1], 12), -90, -90, 0, 800)
                        time.sleep(0.8)
                    
                    # Return to initial position
                    initMove()
                    time.sleep(1.5)

                    # Reset flags after the action
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
    # Set mode to 'stack' for block stacking or 'sort' for block sorting
    mode = 'stack'  # Change to 'sort' for sorting mode
    move_handler = MoveHandler(detector, mode=mode)

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
                # These variables would typically be updated by your detector:
                move_handler.world_X, move_handler.world_Y = detector.world_x, detector.world_y
                move_handler.track = detector.track
                
                if move_handler.first_move:
                    move_handler.start_pick_up = True
            
            key = cv2.waitKey(1)
            if key == 27:
                break
            # Optional: toggle mode during runtime by pressing 'm'
            if key == ord('m'):
                if move_handler.mode == 'sort':
                    move_handler.mode = 'stack'
                    print("Switched to STACK mode")
                else:
                    move_handler.mode = 'sort'
                    print("Switched to SORT mode")
    
    my_camera.camera_close()
    cv2.destroyAllWindows()

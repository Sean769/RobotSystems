#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/ArmPi/')
import cv2
import numpy as np
import time
import math
import threading
from LABConfig import *
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board
from CameraCalibration.CalibrationConfig import *

SERVO1 = 500

def getAngle(x, y, angle):
    # Dummy implementation for angle calculation; replace with the actual computation.
    return 500

class MotionController:
    def __init__(self, mode="sorting"):
        self.AK = ArmIK()
        self.servo1 = SERVO1
        self.mode = mode
        if self.mode == "sorting":
            self.placement_coords = {
                'red':   (-15 + 0.5, 12 - 0.5, 1.5),
                'green': (-15 + 0.5, 6 - 0.5, 1.5),
                'blue':  (-15 + 0.5, 0 - 0.5, 1.5),
            }
        elif self.mode == "stacking":
            self.placement_coords = {
                'red':   (-15 + 1, -7 - 0.5, 1.5),
                'green': (-15 + 1, -7 - 0.5, 1.5),
                'blue':  (-15 + 1, -7 - 0.5, 1.5),
            }
            self.z = self.placement_coords['red'][2]
            self.dz = 2.5
        self.first_move = True
        self.unreachable = False

    def init_arm(self):
        print("Initializing arm to starting position...")
        Board.setBusServoPulse(1, self.servo1 - 50, 300)
        Board.setBusServoPulse(2, 500, 500)
        self.AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)
        self.wait_for_motion(1.5)

    def wait_for_motion(self, duration):
        print(f"Waiting for {duration} seconds for motion to complete...")
        start = time.time()
        while time.time() - start < duration:
            time.sleep(0.05)

    def move_to_block(self, world_coords):
        x, y = world_coords
        print("Moving arm to block position:", world_coords)
        result = self.AK.setPitchRangeMoving((x, y, 7), -90, -90, 0)
        if result:
            duration = result[2] / 1000.0
            self.wait_for_motion(duration + 1)
        else:
            self.unreachable = True

    def pick_block(self, world_coords, rotation_angle):
        print("Executing pick-up sequence at:", world_coords)
        Board.setBusServoPulse(1, self.servo1 - 280, 500)
        servo2_angle = getAngle(world_coords[0], world_coords[1], rotation_angle)
        print("Setting servo2 to angle:", servo2_angle)
        Board.setBusServoPulse(2, servo2_angle, 500)
        time.sleep(0.8)
        print("Lowering arm to pick block...")
        self.AK.setPitchRangeMoving((world_coords[0], world_coords[1], 2), -90, -90, 0, 1000)
        self.wait_for_motion(2)
        print("Closing gripper to pick block...")
        Board.setBusServoPulse(1, self.servo1, 500)
        self.wait_for_motion(1)
        print("Lifting arm with block...")
        Board.setBusServoPulse(2, 500, 500)
        self.AK.setPitchRangeMoving((world_coords[0], world_coords[1], 12), -90, -90, 0, 1000)
        self.wait_for_motion(1)

    def place_block(self, color, custom_z=None):
        coord = self.placement_coords[color]
        target_z = custom_z if custom_z is not None else coord[2]
        print("Moving to deposit position for", color, "block at Z =", target_z)
        result = self.AK.setPitchRangeMoving((coord[0], coord[1], 12), -90, -90, 0)
        if result:
            duration = result[2] / 1000.0
            self.wait_for_motion(duration + 0.5)
        servo2_angle = getAngle(coord[0], coord[1], -90)
        Board.setBusServoPulse(2, servo2_angle, 500)
        time.sleep(0.5)
        self.AK.setPitchRangeMoving((coord[0], coord[1], target_z + 3), -90, -90, 0, 500)
        self.wait_for_motion(0.5)
        self.AK.setPitchRangeMoving((coord[0], coord[1], target_z), -90, -90, 0, 1000)
        self.wait_for_motion(0.8)
        print("Opening gripper to release block...")
        Board.setBusServoPulse(1, self.servo1 - 200, 500)
        self.wait_for_motion(0.8)
        self.AK.setPitchRangeMoving((coord[0], coord[1], 12), -90, -90, 0, 800)
        self.wait_for_motion(0.8)
        print("Returning arm to initial position...")
        self.init_arm()

    def perform_pick_and_place(self, world_coords, detected_color, rotation_angle):
        if detected_color is None or world_coords is None:
            return
        print("Performing pick-and-place for", detected_color, "block at", world_coords)
        self.move_to_block(world_coords)
        self.pick_block(world_coords, rotation_angle)
        self.place_block(detected_color)

    def perform_stack(self, world_coords, detected_color, rotation_angle):
        if detected_color is None or world_coords is None:
            return
        current_z = self.z
        self.z += self.dz
        if self.z >= 2 * self.dz + self.placement_coords[detected_color][2]:
            self.z = self.placement_coords[detected_color][2]
        if current_z == self.placement_coords[detected_color][2]:
            print("Waiting for stacking area to clear...")
            time.sleep(3)
        self.move_to_block(world_coords)
        self.pick_block(world_coords, rotation_angle)
        self.place_block(detected_color, custom_z=self.z)

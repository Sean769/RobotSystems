#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/ArmPi/')
import cv2
import numpy as np
import time
import Camera
import threading
from LABConfig import *
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board
from CameraCalibration.CalibrationConfig import *

class ColorDetector:
    def __init__(self, target_colors=('blue','red','green')):
        # Initialize the ColorDetector with a target color.
        # :param target_color: The color name (string) to be detected (e.g., 'red').
        self.img = None
        self.target_colors = target_colors  # Store the target color for detection.
        self.size = (640, 480)
        self.get_roi = False
        self.area_max = 0
        self.areaMaxContour = 0
        self.range_rgb = {
            'red': (0, 0, 255),
            'blue': (255, 0, 0),
            'green': (0, 255, 0),
            'black': (0, 0, 0),
            'white': (255, 255, 255),
        }
        self.track = False
        self.last_x, self.last_y = 0, 0
        self.rotation_angle = 0
        self.world_x, self.world_y = 0, 0
        self.current_color = 'None'

    def getAreaMaxContour(self, contours):
        contour_area_temp = 0
        contour_area_max = 0
        area_max_contour = None

        for c in contours:  # Go through all contours
            contour_area_temp = math.fabs(cv2.contourArea(c))  # Calculate contour area
            if contour_area_temp > contour_area_max:
                contour_area_max = contour_area_temp
                if contour_area_temp > 300:  # The maximum area contour is only valid when the area is greater than 300 to filter interference
                    area_max_contour = c

        return area_max_contour, contour_area_max  # Returns the largest contour
    
    def preprocess_image(self, img):
        # Resize and apply Gaussian blur to the input image to reduce noise and improve detection, change to LAB color space.
        frame_resized = cv2.resize(img, self.size, interpolation=cv2.INTER_NEAREST)  # Resize frame.
        frame_gb = cv2.GaussianBlur(frame_resized, (11, 11), 11)  # Apply Gaussian blur to smooth the image.
        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # Convert image to LAB space
        self.img = img
        return frame_lab

    def find_box_space(self, img_processed):
        # Define LAB color range for the target color (e.g., 'red').
        color_range_eva = {
            'red': [(0, 171, 136), (255, 255, 255)],
            'green': [(0, 0, 0), (76, 115, 255)],
            'blue': [(0, 0, 0), (255, 255, 100)],
            'black': [(0, 0, 0), (56, 255, 255)],
            'white': [(193, 0, 0), (255, 250, 255)],
        }
        
        max_contour = None
        area_max = 0
        detected_color = None
        
        # Iterate over all target colors in self.target_colors
        for color in self.target_colors:
            # Check if the color exists in the color range dictionary
            if color in color_range_eva:
                frame_mask = cv2.inRange(img_processed, color_range_eva[color][0], color_range_eva[color][1])  # Create a mask for this color
                opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))  # Remove small noise
                closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))  # Fill small holes
                
                contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # Find contours
                
                # Get the largest contour
                areaMaxContour, area_temp = self.getAreaMaxContour(contours)
                if area_temp > area_max and area_temp > 2500:  # Ensure the area is large enough
                    area_max = area_temp
                    max_contour = areaMaxContour
                    detected_color = color
        
        # Now if we found a valid contour, return its information
        if max_contour is not None:
            rect = cv2.minAreaRect(max_contour)
            self.rotation_angle = rect[2]
            box = np.int0(cv2.boxPoints(rect))

            roi = getROI(box)  # Get roi area
            self.get_roi = True

            img_centerx, img_centery = getCenter(rect, roi, self.size, square_length)  # Get the center coordinates
            self.world_x, self.world_y = convertCoordinate(img_centerx, img_centery, self.size)  # Convert to real world coordinates
            self.current_color = detected_color
            return box, detected_color
        
        return None, None
    
    def annotate_box(self, box, detected_color):
        if box is None:
            return self.img
        # Use the detected color to draw the contours and the center point
        cv2.drawContours(self.img, [box], -1, self.range_rgb[detected_color], 2)
        cv2.putText(self.img, '(' + str(self.world_x) + ',' + str(self.world_y) + ')', 
                    (min(box[0, 0], box[2, 0]), box[2, 1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.range_rgb[detected_color], 1)  # Draw center point
        # Calculate the distance from the last position to the current position
        self.distance = math.sqrt(pow(self.world_x - self.last_x, 2) + pow(self.world_y - self.last_y, 2))  
        self.last_x, self.last_y = self.world_x, self.world_y
        self.track = True
        # Print out the coordinates and the color detected
        # print(f"{detected_color}: ({self.world_x},{self.world_y})")
        return self.img

if __name__ == "__main__":    
    # Initialize the color detector
    detector = ColorDetector()

    # Initialize and open the camera
    my_camera = Camera.Camera()
    my_camera.camera_open()

    while True:
        img = my_camera.frame  # Capture frame from camera
        if img is not None:
            frame = img.copy()  # Copy the frame to avoid modifying the original
            
            img_processed = detector.preprocess_image(frame)
            box, detected_color = detector.find_box_space(img_processed)
            annotated_img = detector.annotate_box(box, detected_color)
            
            cv2.imshow('annotated image', annotated_img)  # Show the processed frame

        key = cv2.waitKey(1)
        if key == 27:  # Press 'Esc' key to exit
            break

    # Cleanup resources
    my_camera.camera_close()
    cv2.destroyAllWindows()
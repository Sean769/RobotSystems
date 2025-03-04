#!/usr/bin/python3
import cv2
import numpy as np
import math
import time
import Camera  # Your custom Camera module

# For demonstration, we supply dummy implementations.
# In your actual system, these should come from your calibration modules.
def getROI(box):
    # Dummy: simply return the box as the region of interest.
    return box

def getCenter(rect, roi, size, square_length):
    # Dummy: use the center from cv2.minAreaRect.
    return rect[0]

def convertCoordinate(img_centerx, img_centery, size):
    # Dummy: simple scaling conversion.
    img_w, img_h = size
    world_x = (img_centerx - img_w / 2) * 10 / img_w
    world_y = (img_centery - img_h / 2) * 10 / img_h
    return (round(world_x, 2), round(world_y, 2))

# ---------------------------
# Extended Perception Class
# ---------------------------
class ExtendedBlockDetector:
    def __init__(self, target_colors, color_range, square_length, mode='sorting'):
        """
        Initializes the extended detector.
        :param target_colors: Tuple of target color names (e.g., ('red','green','blue')).
        :param color_range: Dict mapping each color to its (lower, upper) LAB thresholds.
        :param square_length: Calibration parameter for coordinate conversion.
        :param mode: Either 'sorting' or 'palletizing' (affects accumulation thresholds and annotations).
        """
        self.target_colors = target_colors
        self.color_range = color_range
        self.square_length = square_length
        self.mode = mode
        self.reset_accumulation()

    def reset_accumulation(self):
        self.count = 0
        self.center_list = []
        self.color_list = []
        self.last_detection_time = None
        self.last_world_coords = None

    def preprocess_image(self, img, size=(640, 480)):
        resized = cv2.resize(img, size, interpolation=cv2.INTER_NEAREST)
        blurred = cv2.GaussianBlur(resized, (11, 11), 11)
        return blurred

    def convert_to_lab(self, img):
        return cv2.cvtColor(img, cv2.COLOR_BGR2LAB)

    def generate_color_mask(self, lab_img, color):
        lower, upper = self.color_range[color]
        mask = cv2.inRange(lab_img, lower, upper)
        kernel = np.ones((6, 6), np.uint8)
        mask_opened = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask_closed = cv2.morphologyEx(mask_opened, cv2.MORPH_CLOSE, kernel)
        return mask_closed

    def get_largest_contour(self, contours, area_threshold=300):
        max_area = 0
        largest_contour = None
        for c in contours:
            area = math.fabs(cv2.contourArea(c))
            if area > max_area and area > area_threshold:
                max_area = area
                largest_contour = c
        return largest_contour, max_area

    def detect_block(self, img, size=(640, 480)):
        """
        Performs the basic detection steps:
          1. Preprocess the image and convert to LAB color space.
          2. For each target color, create a mask and find the largest contour.
          3. Choose the best candidate (largest valid contour among target colors).
          4. Compute the rotated bounding box, ROI, and convert the center to world coordinates.
        Returns:
          (annotated image, world coordinates, detected color, rotation angle)
        """
        annotated_img = img.copy()
        preprocessed = self.preprocess_image(img, size)
        lab_img = self.convert_to_lab(preprocessed)
        detected = False
        world_coordinates = None
        detected_color = None
        rotation_angle = 0

        best_area = 0
        best_rect = None
        best_color = None

        # Loop through all target colors and keep the detection with the largest area.
        for color in self.target_colors:
            mask = self.generate_color_mask(lab_img, color)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            largest_contour, area = self.get_largest_contour(contours)
            if largest_contour is not None and area > 2500 and area > best_area:
                best_area = area
                best_rect = cv2.minAreaRect(largest_contour)
                best_color = color

        if best_rect is not None:
            box = np.int0(cv2.boxPoints(best_rect))
            roi = getROI(box)  # Use your actual ROI extraction here.
            img_center = getCenter(best_rect, roi, size, self.square_length)
            world_coordinates = convertCoordinate(img_center[0], img_center[1], size)
            rotation_angle = best_rect[2]
            cv2.drawContours(annotated_img, [box], -1, (0, 0, 255), 2)
            cv2.putText(annotated_img, f'{world_coordinates}', (min(box[0, 0], box[2, 0]), box[2, 1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            detected = True
            detected_color = best_color

        return annotated_img, world_coordinates, detected_color, rotation_angle

    def update_accumulation(self, world_coords, detected_color, rotation_angle):
        """
        Updates the accumulation lists with the current detection.
        Also computes the distance from the last detection (if any).
        """
        if self.last_world_coords is not None:
            dist = math.sqrt((world_coords[0] - self.last_world_coords[0])**2 +
                             (world_coords[1] - self.last_world_coords[1])**2)
        else:
            dist = 0
        self.last_world_coords = world_coords

        self.center_list.append(world_coords)
        # Map color to a numeric value: red=1, green=2, blue=3; otherwise 0.
        color_value = 0
        if detected_color == 'red':
            color_value = 1
        elif detected_color == 'green':
            color_value = 2
        elif detected_color == 'blue':
            color_value = 3
        self.color_list.append(color_value)
        self.count += 1
        if self.last_detection_time is None:
            self.last_detection_time = time.time()

    def get_stable_detection(self):
        """
        Checks if detections have been accumulating over a threshold time.
        If so, computes the average (stable) world coordinates and a majority color.
        Threshold depends on the mode:
            - 'sorting': 1 second
            - 'palletizing': 0.5 second
        Returns a tuple (avg_coords, avg_color) or None if not yet stable.
        """
        time_threshold = 1.0 if self.mode == 'sorting' else 0.5
        if self.last_detection_time is None:
            return None
        if time.time() - self.last_detection_time > time_threshold and self.count > 0:
            avg_x = sum(coord[0] for coord in self.center_list) / self.count
            avg_y = sum(coord[1] for coord in self.center_list) / self.count
            avg_coords = (round(avg_x, 2), round(avg_y, 2))
            avg_color_val = round(sum(self.color_list) / len(self.color_list))
            if avg_color_val == 1:
                avg_color = 'red'
            elif avg_color_val == 2:
                avg_color = 'green'
            elif avg_color_val == 3:
                avg_color = 'blue'
            else:
                avg_color = 'None'
            self.reset_accumulation()
            return avg_coords, avg_color
        return None

    def process_frame(self, img, size=(640, 480)):
        """
        Runs detection on the current frame, updates accumulation data, and
        annotates the frame with a stable detection result when available.
        In palletizing mode, an extra instruction is drawn.
        """
        annotated_img, world_coords, detected_color, rotation_angle = self.detect_block(img, size)
        if world_coords is not None and detected_color is not None:
            self.update_accumulation(world_coords, detected_color, rotation_angle)
            stable = self.get_stable_detection()
            if stable is not None:
                stable_coords, stable_color = stable
                cv2.putText(annotated_img, f'Stable: {stable_coords} {stable_color}', (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
        else:
            self.reset_accumulation()

        # Draw detected color text at the bottom.
        cv2.putText(annotated_img, "Color: " + (detected_color if detected_color is not None else "None"),
                    (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0), 2)

        # For palletizing mode, add an extra instruction message.
        if self.mode == 'palletizing':
            cv2.putText(annotated_img, "Make sure no blocks in the stacking area", 
                        (15, int(img.shape[0] / 2)), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
        return annotated_img

# ---------------------------
# Simple Demonstration Program
# ---------------------------
if __name__ == '__main__':
    # Define LAB color ranges (example values—replace with your calibrated values)
    color_range = {
        'red':   (np.array([20, 150, 150]), np.array([255, 200, 200])),
        'green': (np.array([0, 150, 150]),  np.array([150, 255, 150])),
        'blue':  (np.array([0, 150, 150]),  np.array([150, 150, 255])),
    }
    target_colors = ('red', 'green', 'blue')
    square_length = 10  # Calibration parameter

    # Choose the mode: 'sorting' or 'palletizing'
    detector = ExtendedBlockDetector(target_colors, color_range, square_length, mode='sorting')

    # Open the camera using your custom Camera module
    my_camera = Camera.Camera()
    my_camera.camera_open()

    while True:
        frame = my_camera.frame
        if frame is not None:
            processed = detector.process_frame(frame, size=(640, 480))
            cv2.imshow("Extended Detection", processed)
        key = cv2.waitKey(1)
        if key == 27:  # Exit on ESC key
            break

    my_camera.camera_close()
    cv2.destroyAllWindows()

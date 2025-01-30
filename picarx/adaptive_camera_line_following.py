import cv2 as cv
import numpy as np
from time import sleep
from picarx_improved import Picarx
from picamera2 import Picamera2
import logging

# Configure logging
logging_format = "%(asctime)s: %(message)s"
logging.basicConfig(format=logging_format, level=logging.INFO, datefmt="%H:%M:%S")
logger = logging.getLogger()
logger.setLevel(logging.DEBUG)

# Initialize the PiCar
car = Picarx()

# Configuration for line-following
config = {
    "threshold": 120,
    "threshold_max": 180,
    "threshold_min": 40,
    "th_iterations": 10,
    "black_min": 4,
    "black_max": 10,
    "turn_angle": 45,
    "shift_max": 20,
    "shift_step": 0.125,
    "turn_step": 0.25,
    "straight_run": 0.5,
    "moderate_turn": 15,
    "sharp_turn": 30,
    "angle_threshold": 20,
}

# Initialize the camera
picam2 = Picamera2()
camera_config = picam2.create_preview_configuration(main={"size": (640, 480)})
picam2.configure(camera_config)
picam2.start()

def balance_pic(image, config):
    T = config["threshold"]
    for _ in range(config["th_iterations"]):
        _, gray = cv.threshold(image, T, 255, cv.THRESH_BINARY_INV)
        black_pixels = cv.countNonZero(gray)
        total_pixels = image.shape[0] * image.shape[1]
        black_percentage = (black_pixels / total_pixels) * 100

        if black_percentage > config["black_max"]:
            T -= 10
        elif black_percentage < config["black_min"]:
            T += 10
        else:
            return gray
    return gray

def find_main_contour(image):
    contours, _ = cv.findContours(image, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    main_contour = max(contours, key=cv.contourArea)
    rect = cv.minAreaRect(main_contour)
    box = cv.boxPoints(rect)
    box = np.intp(box)
    return box

def get_vector(box, image_width):
    center = np.mean(box, axis=0)
    image_center = image_width / 2
    angle = np.arctan2(box[1][1] - box[0][1], box[1][0] - box[0][0]) * 180 / np.pi
    shift = center[0] - image_center
    return angle, shift

def check_shift_turn(angle, shift, config):
    turn_strength = 0
    if abs(angle - 90) > config["angle_threshold"]:
        turn_strength = config["sharp_turn"] if abs(angle - 90) > config["angle_threshold"] * 2 else config["moderate_turn"]
    shift_state = np.sign(shift) if abs(shift) > config["shift_max"] else 0
    return turn_strength, shift_state

def control_car(turn_strength, shift_state, config):
    turn_angle = shift_state * turn_strength if shift_state != 0 else 0
    car.set_dir_servo_angle(turn_angle)
    print(f"Turning {'left' if turn_angle > 0 else 'right' if turn_angle < 0 else 'straight'}, angle: {turn_angle}")
    car.forward(30)
    sleep(config["turn_step"] if turn_strength > config["moderate_turn"] else config["straight_run"])

def main():
    car.set_cam_tilt_angle(45)
    try:
        while True:
            frame = picam2.capture_array()
            if frame is None:
                print("Failed to grab frame")
                continue

            gray = cv.cvtColor(frame, cv.COLOR_RGB2GRAY)
            blurred = cv.GaussianBlur(gray, (9, 9), 0)
            thresholded = balance_pic(blurred, config)
            box = find_main_contour(thresholded)

            if box is not None:
                angle, shift = get_vector(box, frame.shape[1])
                print(f"Angle: {angle:.2f}, Shift: {shift:.2f}")
                turn_strength, shift_state = check_shift_turn(angle, shift, config)
                print(f"Turn Strength: {turn_strength}, Shift State: {shift_state}")
                control_car(turn_strength, shift_state, config)
            else:
                print("Line not found, stopping")
                car.stop()
                break
    except KeyboardInterrupt:
        print("Program interrupted")
    finally:
        car.stop()
        picam2.stop()

if __name__ == "__main__":
    main()

import cv2 as cv
import numpy as np
from time import sleep
from concurrent.futures import ThreadPoolExecutor
from threading import Event
from readerwriterlock import rwlock
import logging

# Import car and camera modules
from picarx_improved import Picarx
from picamera2 import Picamera2

# --------------------------
# Configuration and Logging
# --------------------------
logging_format = "%(asctime)s: %(message)s"
logging.basicConfig(format=logging_format, level=logging.INFO, datefmt="%H:%M:%S")
logger = logging.getLogger()
logger.setLevel(logging.DEBUG)

# Assignment configuration for line following
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
    "ultrasonic_threshold": 10  
}


# --------------------------
# Global Shutdown Event
# --------------------------
shutdown_event = Event()

# --------------------------
# MessageBus Class
# --------------------------
class MessageBus:
    def __init__(self):
        self.message = None
        self.lock = rwlock.RWLockWriteD()  # writer-priority read–write lock

    def write(self, message):
        with self.lock.gen_wlock():
            self.message = message

    def read(self):
        with self.lock.gen_rlock():
            return self.message

# --------------------------
# Image Processing Functions
# --------------------------
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
    # Calculate the angle using two adjacent points
    angle = np.arctan2(box[1][1] - box[0][1], box[1][0] - box[0][0]) * 180 / np.pi
    shift = center[0] - image_center
    return angle, shift

def check_shift_turn(angle, shift, config):
    turn_strength = 0
    # We assume a nominal angle of 90 degrees as the “ideal” orientation
    if abs(angle - 90) > config["angle_threshold"]:
        if abs(angle - 90) > config["angle_threshold"] * 2:
            turn_strength = config["sharp_turn"]
        else:
            turn_strength = config["moderate_turn"]
    shift_state = np.sign(shift) if abs(shift) > config["shift_max"] else 0
    return turn_strength, shift_state


def control_car(car, turn_strength, shift_state, config):
    # Compute the turning angle; if no shift, go straight.
    turn_angle = shift_state * turn_strength if shift_state != 0 else 0
    car.set_dir_servo_angle(turn_angle)
    logger.info(f"Turning {'left' if turn_angle > 0 else 'right' if turn_angle < 0 else 'straight'}, angle: {turn_angle}")
    car.forward(30)
    # Adjust sleep duration based on the strength of the turn.
    sleep(config["turn_step"] if turn_strength > config["moderate_turn"] else config["straight_run"])

# --------------------------
# Consumer/Producer Functions
# --------------------------
def sensor_function(picam2, sensor_bus, sensor_delay):
    """
    Capture a frame from the camera and write it to the sensor_bus.
    """
    while not shutdown_event.is_set():
        frame = picam2.capture_array()
        if frame is None:
            logger.error("Failed to grab frame")
        else:
            sensor_bus.write(frame)
            logger.debug("Sensor: Frame captured and sent to bus.")
        sleep(sensor_delay)

def interpreter_function(sensor_bus, interpreter_bus, interpreter_delay):
    """
    Read a frame from sensor_bus, process it to compute control parameters,
    and write a tuple (turn_strength, shift_state) to interpreter_bus.
    """
    while not shutdown_event.is_set():
        frame = sensor_bus.read()
        if frame is None:
            sleep(interpreter_delay)
            continue

        # Process the frame
        gray = cv.cvtColor(frame, cv.COLOR_RGB2GRAY)
        blurred = cv.GaussianBlur(gray, (9, 9), 0)
        thresholded = balance_pic(blurred, config)
        box = find_main_contour(thresholded)

        if box is not None:
            angle, shift = get_vector(box, frame.shape[1])
            logger.debug(f"Interpreter: Angle {angle:.2f}, Shift {shift:.2f}")
            turn_strength, shift_state = check_shift_turn(angle, shift, config)
            logger.debug(f"Interpreter: Turn Strength {turn_strength}, Shift State {shift_state}")
            # Send the computed control parameters to the control bus
            interpreter_bus.write((turn_strength, shift_state))
        else:
            logger.info("Interpreter: Line not found. Sending stop command.")
            # Use a special signal (e.g., None) to indicate no valid command
            interpreter_bus.write((0, 0))
        sleep(interpreter_delay)

def control_function(car, interpreter_bus, control_delay):
    """
    Read control parameters from interpreter_bus and command the car.
    """
    while not shutdown_event.is_set():
        # Read distance from the ultrasonic sensor
        distance = car.get_distance()
        if distance is not None and distance < config.get("ultrasonic_threshold", 20):
            logger.info(f"Ultrasonic: Object detected at {distance} cm. Stopping car.")
            car.stop()
        else:
            command = interpreter_bus.read()
            if command is not None:
                turn_strength, shift_state = command
                if turn_strength == 0 and shift_state == 0:
                    logger.info("Control: No valid line detected; stopping car.")
                    car.stop()
                else:
                    logger.debug("Control: Executing control command.")
                    control_car(car, turn_strength, shift_state, config)
        sleep(control_delay)

# --------------------------
# Main Function
# --------------------------
def main():
    # Initialize the car and camera
    car = Picarx()
    car.set_cam_tilt_angle(-60)
    picam2 = Picamera2()
    camera_config = picam2.create_preview_configuration(main={"size": (640, 480)})
    picam2.configure(camera_config)
    picam2.start()

    # Create message busses for sensor and interpreter communication
    sensor_bus = MessageBus()
    interpreter_bus = MessageBus()

    # Define delay times for each thread
    sensor_delay = 0.05       # how often to capture a frame
    interpreter_delay = 0.05  # how fast to process sensor data
    control_delay = 0.1      # how fast to update control commands

    # Use ThreadPoolExecutor to run tasks concurrently
    try:
        with ThreadPoolExecutor(max_workers=3) as executor:
            executor.submit(sensor_function, picam2, sensor_bus, sensor_delay)
            executor.submit(interpreter_function, sensor_bus, interpreter_bus, interpreter_delay)
            executor.submit(control_function, car, interpreter_bus, control_delay)

            # Keep the main thread alive until a KeyboardInterrupt is received.
            while not shutdown_event.is_set():
                sleep(1)
    except KeyboardInterrupt:
        logger.info("KeyboardInterrupt received, shutting down.")
        shutdown_event.set()
    finally:
        # Gracefully shutdown all operations
        car.stop()
        picam2.stop()
        logger.info("System shutdown complete.")

if __name__ == '__main__':
    main()

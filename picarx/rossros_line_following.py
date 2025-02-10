#!/usr/bin/env python3
import cv2 as cv
import numpy as np
import time
import logging

# Import RossROS classes
from rossros import Bus, ConsumerProducer, Producer, Consumer, Timer, runConcurrently

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

# Configuration parameters for line following and obstacle detection.
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
# Helper Functions for Image Processing
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
    if abs(angle - 90) > config["angle_threshold"]:
        if abs(angle - 90) > config["angle_threshold"] * 2:
            turn_strength = config["sharp_turn"]
        else:
            turn_strength = config["moderate_turn"]
    shift_state = np.sign(shift) if abs(shift) > config["shift_max"] else 0
    return turn_strength, shift_state

def steer_car(car, turn_strength, shift_state, config):
    """Compute the steering angle from the line-interpreter output and set the servo."""
    turn_angle = shift_state * turn_strength if shift_state != 0 else 0
    car.set_dir_servo_angle(turn_angle)
    logger.info(f"Steering: {'Left' if turn_angle > 0 else 'Right' if turn_angle < 0 else 'Straight'}, angle: {turn_angle}")

# --------------------------
# Main Function using RossROS Consumer-Producers
# --------------------------
def main():
    # Initialize the car and camera.
    car = Picarx()
    car.set_cam_tilt_angle(-60)
    picam2 = Picamera2()
    camera_config = picam2.create_preview_configuration(main={"size": (640, 480)})
    picam2.configure(camera_config)
    picam2.start()
    logger.info("Camera started.")

    # Create a termination bus and a Timer that will trigger termination after (e.g.) 60 seconds.
    term_bus = Bus(False, "Termination Bus")
    termination_timer = Timer(output_buses=term_bus, duration=60, delay=0.1, name="Termination Timer")

    # Create message buses for the two pipelines.
    line_sensor_bus = Bus(None, "Line Sensor Bus")
    line_command_bus = Bus((0, 0), "Line Command Bus")
    ultra_sensor_bus = Bus(None, "Ultrasonic Sensor Bus")
    ultra_command_bus = Bus(0, "Ultrasonic Command Bus")

    # Set delays (in seconds) for each consumer/producer.
    sensor_delay = 0.05
    interpreter_delay = 0.05
    line_control_delay = 0.01

    ultra_sensor_delay = 0.1
    ultra_interpreter_delay = 0.1
    ultra_control_delay = 0.1

    # --------------------------
    # Define RossROS CP Functions (as closures)
    # --------------------------
    def line_sensor_function():
        """Producer: Capture a frame from the camera."""
        frame = picam2.capture_array()
        if frame is None:
            logger.error("Line Sensor: Failed to capture frame")
        else:
            logger.debug("Line Sensor: Captured frame")
        return frame

    def line_interpreter_function(frame):
        """Consumer-Producer: Process the frame to compute (turn_strength, shift_state)."""
        if frame is None:
            logger.info("Line Interpreter: No frame received.")
            return (0, 0)
        gray = cv.cvtColor(frame, cv.COLOR_RGB2GRAY)
        blurred = cv.GaussianBlur(gray, (9, 9), 0)
        thresholded = balance_pic(blurred, config)
        box = find_main_contour(thresholded)
        if box is not None:
            angle, shift = get_vector(box, frame.shape[1])
            logger.debug(f"Line Interpreter: Angle {angle:.2f}, Shift {shift:.2f}")
            turn_strength, shift_state = check_shift_turn(angle, shift, config)
            logger.debug(f"Line Interpreter: Turn Strength {turn_strength}, Shift State {shift_state}")
            return (turn_strength, shift_state)
        else:
            logger.info("Line Interpreter: Line not found. Commanding stop.")
            return (0, 0)

    def line_controller_function(cmd):
        """Consumer: Adjust the steering based on the interpreted line command."""
        turn_strength, shift_state = cmd
        if turn_strength == 0 and shift_state == 0:
            logger.info("Line Controller: No valid line detected; maintaining current steering.")
        else:
            steer_car(car, turn_strength, shift_state, config)
        return cmd  # Dummy return

    def ultrasonic_sensor_function():
        """Producer: Read the ultrasonic sensor value."""
        distance = car.get_distance()
        if distance is not None:
            logger.debug(f"Ultrasonic Sensor: Distance {distance} cm")
        else:
            logger.error("Ultrasonic Sensor: Failed to get distance")
        return distance

    def ultrasonic_interpreter_function(distance):
        """Consumer-Producer: Interpret the ultrasonic sensor reading."""
        if distance is None or distance < config["ultrasonic_threshold"]:
            logger.info("Ultrasonic Interpreter: Obstacle detected or invalid reading.")
            return 0  # 0 indicates “stop”
        else:
            logger.debug("Ultrasonic Interpreter: Path clear.")
            return 1  # 1 indicates “go”
    
    def ultrasonic_controller_function(safe_flag):
        """Consumer: Control the throttle—stop if an obstacle is too close; otherwise, drive forward."""
        if safe_flag == 0:
            logger.info("Ultrasonic Controller: Stopping car due to obstacle.")
            car.stop()
        else:
            logger.debug("Ultrasonic Controller: Moving car forward.")
            car.forward(30)
        return safe_flag  # Dummy return

    # --------------------------
    # Create RossROS Consumer-Producers
    # --------------------------
    # Line-following pipeline:
    line_sensor_cp = Producer(
        producer_function=line_sensor_function,
        output_buses=line_sensor_bus,
        delay=sensor_delay,
        termination_buses=term_bus,
        name="Line Sensor Producer"
    )

    line_interpreter_cp = ConsumerProducer(
        consumer_producer_function=line_interpreter_function,
        input_buses=line_sensor_bus,
        output_buses=line_command_bus,
        delay=interpreter_delay,
        termination_buses=term_bus,
        name="Line Interpreter CP"
    )

    line_controller_cp = Consumer(
        consumer_function=line_controller_function,
        input_buses=line_command_bus,
        delay=line_control_delay,
        termination_buses=term_bus,
        name="Line Controller Consumer"
    )

    # Ultrasonic (obstacle avoidance) pipeline:
    ultra_sensor_cp = Producer(
        producer_function=ultrasonic_sensor_function,
        output_buses=ultra_sensor_bus,
        delay=ultra_sensor_delay,
        termination_buses=term_bus,
        name="Ultrasonic Sensor Producer"
    )

    ultra_interpreter_cp = ConsumerProducer(
        consumer_producer_function=ultrasonic_interpreter_function,
        input_buses=ultra_sensor_bus,
        output_buses=ultra_command_bus,
        delay=ultra_interpreter_delay,
        termination_buses=term_bus,
        name="Ultrasonic Interpreter CP"
    )

    ultra_controller_cp = Consumer(
        consumer_function=ultrasonic_controller_function,
        input_buses=ultra_command_bus,
        delay=ultra_control_delay,
        termination_buses=term_bus,
        name="Ultrasonic Controller Consumer"
    )

    # --------------------------
    # Run All CPs Concurrently
    # --------------------------
    cp_list = [
        termination_timer,
        line_sensor_cp,
        line_interpreter_cp,
        line_controller_cp,
        ultra_sensor_cp,
        ultra_interpreter_cp,
        ultra_controller_cp
    ]

    try:
        runConcurrently(cp_list)
    except KeyboardInterrupt:
        logger.info("KeyboardInterrupt received, shutting down.")
    finally:
        car.stop()
        picam2.stop()
        logger.info("System shutdown complete.")

if __name__ == '__main__':
    main()

#!/usr/bin/python3
import cv2 as cv
import numpy as np
from time import sleep
import logging

# Import your car and camera modules
from picarx_improved import Picarx
from picamera2 import Picamera2

# Import RossROS classes and functions
from RossROS import Bus, ConsumerProducer, Consumer, Timer, runConcurrently

# Set up logging
logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s: %(message)s",
                    datefmt="%H:%M:%S")

# ---------------------------
# Configuration Parameters
# ---------------------------
config = {
    "threshold": 120,
    "th_iterations": 10,
    "black_min": 4,
    "black_max": 10,
    "angle_threshold": 20,
    "moderate_turn": 15,
    "sharp_turn": 30,
    "shift_max": 20,
    "turn_step": 0.25,
    "straight_run": 0.5,
    "ultrasonic_threshold": 10
}

# ---------------------------
# Initialize Hardware
# ---------------------------
car = Picarx()
car.set_cam_tilt_angle(45)
picam2 = Picamera2()
camera_config = picam2.create_preview_configuration(main={"size": (640, 480)})
picam2.configure(camera_config)
picam2.start()

# ---------------------------
# Create Buses
# ---------------------------
# Line-following buses
camera_bus = Bus(initial_message=None, name="Camera Bus")
line_control_bus = Bus(initial_message=(0, 0), name="Line Control Bus")

# Ultrasonic sensor buses
ultrasonic_bus = Bus(initial_message=1000, name="Ultrasonic Bus")
ultrasonic_interpretation_bus = Bus(initial_message="forward", name="Ultrasonic Interpretation Bus")

# Termination bus shared by all services
termination_bus = Bus(initial_message=False, name="Termination Bus")

# ---------------------------
# Timer for Run-Time Control
# ---------------------------
# The timer will update the termination bus after 'duration' seconds.
timer = Timer(output_buses=termination_bus, duration=360, delay=1, name="Run Timer")

# ---------------------------
# Define Consumer-Producer Functions
# ---------------------------

# --- Line-Following Pipeline ---
def sensor_line_follower():
    """
    Capture a camera frame.
    """
    frame = picam2.capture_array()
    return frame

def interpreter_line_follower(frame):
    """
    Process the image to extract a steering command.
    Returns a tuple: (turn_strength, shift_state)
    """
    if frame is None:
        return (0, 0)

    gray = cv.cvtColor(frame, cv.COLOR_RGB2GRAY)
    blurred = cv.GaussianBlur(gray, (9, 9), 0)
    
    # Balance threshold similar to your original code
    T = config["threshold"]
    for _ in range(config["th_iterations"]):
        _, thresh = cv.threshold(blurred, T, 255, cv.THRESH_BINARY_INV)
        black_pixels = cv.countNonZero(thresh)
        total_pixels = blurred.shape[0] * blurred.shape[1]
        black_percentage = (black_pixels / total_pixels) * 100

        if black_percentage > config["black_max"]:
            T -= 10
        elif black_percentage < config["black_min"]:
            T += 10
        else:
            break

    # Find the main contour
    contours, _ = cv.findContours(thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    if contours:
        main_contour = max(contours, key=cv.contourArea)
        rect = cv.minAreaRect(main_contour)
        box = cv.boxPoints(rect)
        box = np.intp(box)

        center = np.mean(box, axis=0)
        image_center = blurred.shape[1] / 2
        # Compute an angle using two adjacent points
        angle = np.arctan2(box[1][1] - box[0][1], box[1][0] - box[0][0]) * 180 / np.pi
        shift = center[0] - image_center

        # Determine turn strength based on deviation from 90 degrees (assumed ideal)
        if abs(angle - 90) > config["angle_threshold"]:
            turn_strength = config["sharp_turn"] if abs(angle - 90) > config["angle_threshold"] * 2 else config["moderate_turn"]
        else:
            turn_strength = 0
        shift_state = int(np.sign(shift)) if abs(shift) > config["shift_max"] else 0
        return (turn_strength, shift_state)
    else:
        return (0, 0)

def controller_line_follower(turn_strength, shift_state):
    """
    Command the car based on line-following data.
    """
    if turn_strength == 0 and shift_state == 0:
        car.stop()
    else:
        turn_angle = shift_state * turn_strength
        car.set_dir_servo_angle(turn_angle)
        car.forward(30)
    return (turn_strength, shift_state)

# --- Ultrasonic Pipeline ---
def sensor_ultrasonic():
    """
    Read distance from the ultrasonic sensor.
    """
    distance = car.get_distance()
    return distance

def interpreter_ultrasonic(distance):
    """
    Interpret the ultrasonic reading.
    Returns "stop" if the distance is below threshold; otherwise "forward".
    """
    if distance is not None and distance < config["ultrasonic_threshold"]:
        return "stop"
    else:
        return "forward"

def controller_ultrasonic(command):
    """
    Control the car based on the ultrasonic sensor.
    If the command is "stop", the car stops.
    Otherwise, it moves forward.
    """
    if command == "stop":
        car.stop()
    else:
        car.forward(30)
    return command

# ---------------------------
# Create RossROS Consumer-Producer Instances
# ---------------------------

# Line-following pipeline:
sensor_line_cp = ConsumerProducer(
    consumer_producer_function=sensor_line_follower,
    input_buses=(),  # No input required
    output_buses=camera_bus,
    delay=0.1,
    termination_buses=termination_bus,
    name="Line Sensor"
)

interpreter_line_cp = ConsumerProducer(
    consumer_producer_function=interpreter_line_follower,
    input_buses=camera_bus,
    output_buses=line_control_bus,
    delay=0.1,
    termination_buses=termination_bus,
    name="Line Interpreter"
)

controller_line_cp = ConsumerProducer(
    consumer_producer_function=controller_line_follower,
    input_buses=line_control_bus,
    output_buses=Bus(0, "Dummy Bus"),  # No output needed
    delay=0.1,
    termination_buses=termination_bus,
    name="Line Controller"
)

# Ultrasonic pipeline:
sensor_ultrasonic_cp = ConsumerProducer(
    consumer_producer_function=sensor_ultrasonic,
    input_buses=(),
    output_buses=ultrasonic_bus,
    delay=0.1,
    termination_buses=termination_bus,
    name="Ultrasonic Sensor"
)

interpreter_ultrasonic_cp = ConsumerProducer(
    consumer_producer_function=interpreter_ultrasonic,
    input_buses=ultrasonic_bus,
    output_buses=ultrasonic_interpretation_bus,
    delay=0.1,
    termination_buses=termination_bus,
    name="Ultrasonic Interpreter"
)

controller_ultrasonic_cp = ConsumerProducer(
    consumer_producer_function=controller_ultrasonic,
    input_buses=ultrasonic_interpretation_bus,
    output_buses=Bus(0, "Dummy Bus"),  # No output needed
    delay=0.1,
    termination_buses=termination_bus,
    name="Ultrasonic Controller"
)

# Optional: A Printer to monitor the line control bus (for debugging)
line_printer = Consumer(
    consumer_function=lambda val: print("Line Control:", val),
    input_buses=line_control_bus,
    delay=0.5,
    termination_buses=termination_bus,
    name="Line Printer"
)

# ---------------------------
# Run All Services Concurrently
# ---------------------------
# Build a list of all consumer-producer instances along with the Timer.
all_services = [
    timer,
    sensor_line_cp,
    interpreter_line_cp,
    controller_line_cp,
    sensor_ultrasonic_cp,
    interpreter_ultrasonic_cp,
    controller_ultrasonic_cp,
    line_printer
]

if __name__ == '__main__':
    try:
        runConcurrently(all_services)
    finally:
        # Ensure the car and camera shut down gracefully.
        car.stop()
        picam2.stop()

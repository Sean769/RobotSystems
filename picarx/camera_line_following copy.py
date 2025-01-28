import cv2 as cv
import numpy as np
from time import sleep
from picarx_improved import Picarx
from picamera2 import Picamera2

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
}

# Initialize the camera
picam2 = Picamera2()
camera_config = picam2.create_preview_configuration(main={"size": (640, 480)})
picam2.configure(camera_config)
picam2.start()

# Global variable for last known turn direction
last_turn_dir = 0

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
    box = np.intp(box)  # Updated to avoid NumPy deprecation warning
    return box

def get_vector(box, image_width):
    center = np.mean(box, axis=0)
    image_center = image_width / 2
    angle = np.arctan2(box[1][1] - box[0][1], box[1][0] - box[0][0]) * 180 / np.pi
    shift = center[0] - image_center
    return angle, shift

def check_shift_turn(angle, shift, config):
    """
    Determine the turn and shift states based on thresholds.
    """
    turn_state = 0
    if abs(angle) > config["turn_angle"]:
        turn_state = np.sign(angle)

    shift_state = 0
    if abs(shift) > config["shift_max"]:
        shift_state = np.sign(shift)

    return turn_state, shift_state

def get_turn(turn_state, shift_state, config):
    """
    Determine the turn direction and value based on the turn and shift states.
    """
    turn_dir = 0
    turn_val = 0
    if shift_state != 0:
        turn_dir = shift_state
        turn_val = config["shift_step"] if shift_state != turn_state else config["turn_step"]
    elif turn_state != 0:
        turn_dir = turn_state
        turn_val = config["turn_step"]
    return turn_dir, turn_val

def control_car(turn_dir, turn_val, last_turn_dir):
    """
    Control the car's movement based on turn direction and value.
    """
    if turn_dir != 0:
        car.set_dir_servo_angle(turn_dir * 30)  # Scale the turn direction for steering
        print(f"Turning {'left' if turn_dir > 0 else 'right'}, turn value: {turn_val}")
        car.forward(30)  # Move forward while turning
        sleep(turn_val)
        return turn_dir
    else:
        car.set_dir_servo_angle(0)
        print("Moving straight")
        car.forward(30)
        sleep(config["straight_run"])
        return last_turn_dir

def main():
    global last_turn_dir

    # Tilt the camera down by 45 degrees
    car.set_cam_tilt_angle(45)

    try:
        while True:
            try:
                frame = picam2.capture_array()
                if frame is None:
                    print("Failed to grab frame")
                    continue

                # Preprocess the frame
                gray = cv.cvtColor(frame, cv.COLOR_RGB2GRAY)
                blurred = cv.GaussianBlur(gray, (9, 9), 0)
                thresholded = balance_pic(blurred, config)
                box = find_main_contour(thresholded)

                if box is not None:
                    # Line detected, calculate steering
                    angle, shift = get_vector(box, frame.shape[1])
                    print(f"Angle: {angle:.2f}, Shift: {shift:.2f}")
                    turn_state, shift_state = check_shift_turn(angle, shift, config)
                    turn_dir, turn_val = get_turn(turn_state, shift_state, config)
                    last_turn_dir = control_car(turn_dir, turn_val, last_turn_dir)

                    # Visualization: Draw the bounding box and center
                    cv.drawContours(frame, [box], -1, (0, 255, 0), 2)
                    cv.line(frame, (frame.shape[1] // 2, 0), (frame.shape[1] // 2, frame.shape[0]), (255, 0, 0), 2)
                else:
                    # Line lost, steer hard in the last known direction
                    print("Line lost, steering hard in last known direction")
                    car.set_dir_servo_angle(last_turn_dir * 45)
                    car.forward(30)
                    sleep(config["turn_step"])

                # Display the processed frame (try to show it)
                try:
                    cv.imshow("Processed Frame", frame)
                    if cv.waitKey(1) & 0xFF == ord("q"):
                        break
                except cv.error:
                    print("No display available for visualization.")

            except Exception as e:
                print(f"Error during frame processing: {e}")
                car.stop()

    except KeyboardInterrupt:
        print("Program interrupted")
    finally:
        car.stop()
        picam2.stop()
        cv.destroyAllWindows()

if __name__ == "__main__":
    main()

from picarx_improved import Picarx
import os
try:
    from robot_hat import Pin, ADC, PWM, Servo, fileDB
    from robot_hat import Grayscale_Module, Ultrasonic, utils
except ImportError:
    import sys
    sys.path.append(os.path.abspath(os.path.join(
        os.path.dirname(__file__), "..")))
    from sim_robot_hat import Pin, ADC, PWM, Servo, fileDB
    from sim_robot_hat import Grayscale_Module, Ultrasonic, utils
import time

class GrayscaleSensor:
    def __init__(self, adc_pins=['A0', 'A1', 'A2']):
        self.adc_sensors = [ADC(pin) for pin in adc_pins]

    def read_sensors(self):
        return [sensor.read() for sensor in self.adc_sensors]

class LineInterpreter:
    def __init__(self, sensitivity=300, polarity="dark", moderate_threshold=200, high_threshold=400):
        self.sensitivity = sensitivity
        self.polarity = polarity
        self.moderate_threshold = moderate_threshold
        self.high_threshold = high_threshold

    def process_data(self, sensor_data):
        edges = [sensor_data[i + 1] - sensor_data[i] for i in range(len(sensor_data) - 1)]
        if self.polarity == "dark":
            edges = [-edge for edge in edges]
        
        position = 0
        if max(edges) > self.sensitivity:
            position = -1  # Right of the line
        elif min(edges) < -self.sensitivity:
            position = 1  # Left of the line
        else:
            position = 0  # Centered
        
        return position
    
    def determine_turn(self, sensor_data):
        diff_left = abs(sensor_data[1] - sensor_data[0])  # A1 - A0
        diff_right = abs(sensor_data[1] - sensor_data[2]) # A1 - A2
        
        turn_angle = 0
        if diff_left > self.high_threshold or diff_right > self.high_threshold:
            turn_angle = 30
        elif diff_left > self.moderate_threshold or diff_right > self.moderate_threshold:
            turn_angle = 15
        
        return turn_angle

class SteeringController:
    def __init__(self, car):
        self.car = car

    def apply_steering(self, position, turn_angle):
        steering_angle = position * turn_angle
        self.car.set_dir_servo_angle(steering_angle)
        return steering_angle

def main():
    car = Picarx()
    sensor = GrayscaleSensor()
    interpreter = LineInterpreter(sensitivity=300, polarity="dark")
    controller = SteeringController(car)

    try:
        while True:
            sensor_data = sensor.read_sensors()
            position = interpreter.process_data(sensor_data)
            turn_angle = interpreter.determine_turn(sensor_data)
            angle = controller.apply_steering(position, turn_angle)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Stopping...")
        car.stop()

if __name__ == "__main__":
    main()

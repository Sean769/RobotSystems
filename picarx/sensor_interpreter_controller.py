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
        
        #Initialize the ADC sensors.
        #:param adc_pins: List of ADC pins connected to the grayscale sensors.
        
        #from robot_hat import ADC  # Ensure ADC is imported here
        self.adc_sensors = [ADC(pin) for pin in adc_pins]

    def read_sensors(self):
        
        #Read values from the grayscale sensors.
        #:return: List of sensor readings (0 to 1000).
    
        return [sensor.read() for sensor in self.adc_sensors]


class LineInterpreter:
    def __init__(self, sensitivity=300, polarity="dark"):
        
        #Initialize the interpreter with sensitivity and polarity.
        #:param sensitivity: Threshold difference to distinguish dark/light.
        #:param polarity: 'dark' if the line is darker than the floor, otherwise 'light'.
        
        self.sensitivity = sensitivity
        self.polarity = polarity

    def process_data(self, sensor_data):
        
        #Process sensor data to determine robot's position relative to the line.
        #:param sensor_data: List of grayscale sensor readings.
        #:return: Relative position [-1, 1] (positive = left, negative = right).
        
        # Identify edges
        edges = [
            sensor_data[i + 1] - sensor_data[i]
            for i in range(len(sensor_data) - 1)
        ]

        # Adjust for polarity
        if self.polarity == "dark":
            edges = [-edge for edge in edges]

        # Determine the position
        position = 0
        if max(edges) > self.sensitivity:
            position = -1  # Robot is right of the line
        elif min(edges) < -self.sensitivity:
            position = 1  # Robot is left of the line
        else:
            position = 0  # Robot is centered

        # Scale position to [-1, 1]
        return position


class SteeringController:
    def __init__(self, car, scale=30):
        
        #Initialize the controller with a car instance and scaling factor.
        #:param car: Instance of the Picarx class.
        #:param scale: Scaling factor between position offset and steering angle.
        
        self.car = car
        self.scale = scale

    def compute_steering(self, position):
        
        #Compute the steering angle based on the position.
        #:param position: Relative position [-1, 1].
        #:return: Steering angle (degrees).
        
        return position * self.scale

    def apply_steering(self, position):
        #Apply the steering angle to the car and return the commanded angle.
        #:param position: Relative position [-1, 1].
        #:return: Commanded steering angle (degrees).
        
        angle = self.compute_steering(position)
        self.car.set_dir_servo_angle(angle)
        return angle


def main():
    car = Picarx()
    sensor = GrayscaleSensor()
    interpreter = LineInterpreter(sensitivity=300, polarity="dark")
    controller = SteeringController(car, scale=30)

    try:
        while True:
            sensor_data = sensor.read_sensors()
            position = interpreter.process_data(sensor_data)
            angle = controller.apply_steering(position)

            #print(f"Sensor Data: {sensor_data}, Position: {position}, Angle: {angle}")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Stopping...")
        car.stop()

if __name__ == "__main__":
    main()

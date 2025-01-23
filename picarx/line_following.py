from picarx_improved import Picarx
from sensor_interpreter_controller import GrayscaleSensor, LineInterpreter, SteeringController
import time
import logging

# Configure logging
logging_format = "%(asctime)s: %(message)s"
logging.basicConfig(format=logging_format, level=logging.INFO, datefmt="%H:%M:%S")
logger = logging.getLogger()
logger.setLevel(logging.DEBUG)

def line_following():
    car = Picarx()
    sensor = GrayscaleSensor()
    interpreter = LineInterpreter(sensitivity=300, polarity="dark")
    controller = SteeringController(car, scale=30)

    try:
        logger.info("Starting line-following.")
        car.set_power(30)  # Move forward with constant speed
        while True:
            sensor_data = sensor.read_sensors()
            position = interpreter.process_data(sensor_data)
            angle = controller.apply_steering(position)

            logger.debug(f"Sensor Data: {sensor_data}, Position: {position}, Angle: {angle}")
            time.sleep(0.1)
    except KeyboardInterrupt:
        logger.info("Stopping line-following.")
        car.stop()

if __name__ == "__main__":
    line_following()

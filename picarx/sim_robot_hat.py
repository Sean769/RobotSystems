# sim_robot_hat.py (simplified simulation mockup)

class Pin:
    IN = "IN"  # Mock value for input mode
    OUT = "OUT"  # Mock value for output mode
    PULL_UP = "PULL_UP"  # Mock value for pull-up resistor
    PULL_DOWN = "PULL_DOWN"  # Mock value for pull-down resistor

    def __init__(self, pin, mode=None, pull=None):
        self.pin = pin
        self.mode = mode
        self.pull = pull

    def high(self):
        print(f"Pin {self.pin} set HIGH")

    def low(self):
        print(f"Pin {self.pin} set LOW")


class ADC:
    def __init__(self, pin):
        self.pin = pin

    def read(self):
        return 0  # Return a mock value

class PWM:
    def __init__(self, pin):
        self.pin = pin

    def period(self, value):
        print(f"PWM period set to {value} on pin {self.pin}")

    def prescaler(self, value):
        print(f"PWM prescaler set to {value} on pin {self.pin}")

    def pulse_width_percent(self, percent):
        print(f"PWM pulse width set to {percent}% on pin {self.pin}")

class Servo:
    def __init__(self, pin):
        self.pin = pin

    def angle(self, value):
        print(f"Servo on pin {self.pin} set to angle {value}")

class fileDB:
    def __init__(self, db, mode=None, owner=None):
        self.db = db
        self.simulated_data = {}

    def get(self, name, default_value=None):
        return self.simulated_data.get(name, default_value)

    def set(self, name, value):
        self.simulated_data[name] = value

class Grayscale_Module:
    def __init__(self, adc0, adc1, adc2, reference=None):
        self.adc0 = adc0
        self.adc1 = adc1
        self.adc2 = adc2
        self._reference = reference or [1000, 1000, 1000]  # Use _reference for the attribute

    def reference(self, ref=None):
        """Set or get the reference."""
        if ref:
            self._reference = ref
        return self._reference

    def read(self):
        """Mock method to return grayscale data."""
        return [500, 500, 500]  # Mock data for testing

    def read_status(self, gm_val_list):
        """Mock method to return status based on threshold."""
        return [val < ref for val, ref in zip(gm_val_list, self._reference)]


class Ultrasonic:
    def __init__(self, trig, echo):
        self.trig = trig
        self.echo = echo

    def read(self):
        return 100  # Mock distance in cm

# Mock utils module
class utils:
    @staticmethod
    def reset_mcu():
        print("Simulated MCU reset")

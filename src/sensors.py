import pigpio
import time

GYRO_SCALE = 0.098
GYRO_ZERO_SAMPLES = 20

TRIGGER = 17
BUSY = 12
CONVERSION_DELAY = 150e-6

DATA_PINS = [
    27,  # PIN0, least significant bit
    23,  # PIN1
    22,  # PIN2
    24,  # PIN3
    10,  # PIN4
    25,  # PIN5
    9,   # PIN6
    11,  # PIN7, most significant bit
]

class IR:
    """
    Represent one IR sensor.

    API:
        IR(io, pin)
        read() -> 0 or 1

    Args:
        io: Connected `pigpio.pi()` handle.
        pin: GPIO pin connected to this one IR sensor.
    """
    def __init__(self, io, pin):
        self.io = io
        self.pin = pin

        self.io.set_mode(self.pin, pigpio.INPUT)

    def read(self):
        """
        Return one sensor reading: 0 or 1.
        """
        return self.io.read(self.pin)


class LineSensor:
    """
    Combine three IR sensors into one robot-level sensing object.

    API:
        LineSensor(io, pin_left, pin_middle, pin_right)
        read() -> (left, middle, right)

    Args:
        io: Connected `pigpio.pi()` handle.
        pin_left: GPIO pin for the left IR sensor.
        pin_middle: GPIO pin for the middle IR sensor.
        pin_right: GPIO pin for the right IR sensor.
    """
    def __init__(self, io, pin_left, pin_middle, pin_right):
        self.left = IR(io, pin_left)
        self.middle = IR(io, pin_middle)
        self.right = IR(io, pin_right)

    def read(self):
        """
        Return the three sensor readings as a tuple:
            (left, middle, right)
        """
        return (
            self.left.read(),
            self.middle.read(),
            self.right.read(),
        )

    def print_reading(self):
        """
        Debug helper matching the irdemo.py print style.
        """
        left, middle, right = self.read()
        print("IRs: L %d  M %d  R %d" % (left, middle, right))


class ADC:
    """
    Interface to the shared 8-bit A/D converter.

    API:
        adc = ADC(io)
        adc.read() -> integer in [0, 255]
    """
    def __init__(self, io):
        self.io = io

        self.io.set_mode(TRIGGER, pigpio.OUTPUT)
        self.io.write(TRIGGER, 1)

        for pin in DATA_PINS:
            self.io.set_mode(pin, pigpio.INPUT)

    def read(self):
        """
        Start one conversion, wait for the data pins to settle, and return the
        8 data bits as one integer.
        """
        self.io.write(TRIGGER, 0)
        self.io.write(TRIGGER, 0)
        self.io.write(TRIGGER, 1)

        t0 = time.time()
        while time.time() < t0 + CONVERSION_DELAY:
            pass

        value = 0
        for bit_index, pin in enumerate(DATA_PINS):
            bit = self.io.read(pin)
            value |= bit << bit_index

        return value


class Gyroscope:
    """
    Convert the gyro ADC reading into angular velocity.

    API:
        gyro = Gyroscope(io)
        gyro.resetzero()
        gyro.read() -> angular velocity in rad/sec

    Args:
        io: Connected `pigpio.pi()` handle.
        scale: Calibration constant in rad/sec per ADC count.
        zero_samples: Number of stationary ADC samples used for re-zeroing.
    """
    def __init__(self, io, scale=GYRO_SCALE, zero_samples=GYRO_ZERO_SAMPLES):
        self.adc = ADC(io)
        self.scale = float(scale)
        self.zero_samples = int(zero_samples)
        if self.zero_samples <= 0:
            raise ValueError("zero_samples must be positive")
        self.offset = 0.0

        self.resetzero()

    def resetzero(self):
        """
        Re-zero the gyro while the robot is stationary.
        """
        total = 0
        for _ in range(self.zero_samples):
            total += self.adc.read()

        self.offset = total / self.zero_samples
        return self.offset

    def read(self):
        """
        Return angular velocity in rad/sec.
        """
        return self.scale * (self.adc.read() - self.offset)

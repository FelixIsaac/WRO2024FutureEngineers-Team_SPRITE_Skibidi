from typing import List
import time
import RPi.GPIO as GPIO
from motors import DCMotor

from ultrasonic_sensor import UltrasonicSensor
from color_sensor import ColorSensor
from imu import IMU
from motors import DCMotor, Servo

SWITCH = None
ULTRASONIC_SENSORS: List[UltrasonicSensor] = []
COLOR_SENSORS: List[ColorSensor] = []
MOTOR: DCMotor = None
SERVO: Servo = None
IMU_SENSOR: IMU = None


def setup() -> None:
    global MOTOR

    # Using phyiscal board pin out reference
    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)

    # We'll figure out the pins later on

    ULTRASONIC_SENSORS.extend([
        # UltrasonicSensor(16, 18),  # front
    ])

    MOTOR = DCMotor(11, 13)


def main() -> None:
    # Input -> process -> output
    # inputs are ultasonic, colour sensor, imu, camera

    MOTOR.forward()
    MOTOR.start(20)


if __name__ == "__main__":
    try:
        print("Setting up")
        setup()

        time.sleep(1)

        # Run startup sequence: e.g. servo turning, move back etc. etc.

        while True:
            main()
    except KeyboardInterrupt:
        print("Exiting. Cleaning up")
        
        time.sleep(1)

        GPIO.cleanup()

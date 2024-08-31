from typing import List
import time
import RPi.GPIO as GPIO

from ultrasonic_sensor import UltrasonicSensor
from imu import IMU
from motors import DCMotor, Servo
from encoder import Encoder

# SWITCH = None
ULTRASONIC_SENSORS: List[UltrasonicSensor] = []
MOTOR: DCMotor = None
SERVO: Servo = None
IMU_SENSOR: IMU = None
ENCODER: Encoder = None


def setup() -> None:
    global MOTOR, SERVO, IMU_SENSOR, ENCODER

    # Using phyiscal board pin out reference
    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)

    ULTRASONIC_SENSORS.extend([
        UltrasonicSensor(12, 10),  # front
        UltrasonicSensor(22, 24),  # right
        UltrasonicSensor(37, 36)   # left
    ])

    MOTOR = DCMotor(11, 13)
    SERVO = Servo(32)
    ENCODER = Encoder(16)
    IMU_SENSOR = IMU(5, 3)


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

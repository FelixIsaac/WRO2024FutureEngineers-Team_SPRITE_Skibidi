from typing import List
import time
import RPi.GPIO as GPIO

from ultrasonic_sensor import UltrasonicSensor

ULTRASONIC_SENSORS: List[UltrasonicSensor] = []

def setup() -> None:

    # Using phyiscal board pin out reference
    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)

    # We'll figure out the pins later on

    ULTRASONIC_SENSORS.extend([
        UltrasonicSensor(16, 18),  # front
        UltrasonicSensor(13, 15),  # back
        UltrasonicSensor(33, 35),  # left
        UltrasonicSensor(29, 31)   # right
    ])


def main() -> None:
    # Input -> process -> output
    # inputs are ultasonic, colour sensor, imu, camera

    front_distance = ULTRASONIC_SENSORS[0].measure()
    back_distance = ULTRASONIC_SENSORS[1].measure()
    right_distance = ULTRASONIC_SENSORS[2].measure()
    left_distance = ULTRASONIC_SENSORS[3].measure()

    print(front_distance)
    time.sleep(1)


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

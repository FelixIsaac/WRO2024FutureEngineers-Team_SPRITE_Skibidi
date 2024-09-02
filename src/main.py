from typing import List
import time

from gpiozero.pins.pigpio import PiGPIOFactory
from gpiozero import DistanceSensor, Motor, AngularServo

from encoder import Encoder

# Assuming `pigpiod` is running
pin_factory = PiGPIOFactory()

ULTRASONIC_SENSORS: List[DistanceSensor] = []
MOTOR: Motor = None
SERVO: AngularServo = None
# IMU_SENSOR: IMU = None
ENCODER: Encoder = None

def setup() -> None:
    global ULTRASONIC_SENSORS, MOTOR, SERVO, IMU_SENSOR, ENCODER

    # Initialise the ultrasonic sensor with the pin factory

    ULTRASONIC_SENSORS.extend([
        DistanceSensor(echo="BOARD10", trigger="BOARD12", pin_factory=pin_factory),  # front
        DistanceSensor(echo="BOARD24", trigger="BOARD22", pin_factory=pin_factory),  # right
        DistanceSensor(echo="BOARD36", trigger="BOARD37", pin_factory=pin_factory)   # left
    ])

    MOTOR = Motor("BOARD11", "BOARD13", pin_factory=pin_factory)
    SERVO = AngularServo("BOARD32", pin_factory=pin_factory, max_angle=90, min_angle=-90)

def main() -> None:
    # SERVO.angle = 0

    # for i in range(-90, 90, 30):
    #     SERVO.angle = i
    #     print(i)
    #     time.sleep(1)


    # MOTOR.forward(0.1)

    # Note: for ultrasonic sensor and other devices to wait for it to become active then start the script

    for us in ULTRASONIC_SENSORS:
        print(f'Distance for US {us.pin}: {us.distance * 100:.2f} cm')
    time.sleep(0.2)

if __name__ == "__main__":
    try:
        print("Setting up")
        setup()

        # while True:
        main()
    except KeyboardInterrupt:
        print("Keyboard Interrupt Ctrl-C")

    except Exception as err:
        print(err)

    finally:
        print("Exiting. Cleaning up...")

        for us in ULTRASONIC_SENSORS:
            us.close()
        
        MOTOR.stop()
        MOTOR.close()
        SERVO.close()
# Standard Libraries
from typing import List
import time
import math

# Project dependences
import RPi.GPIO as GPIO
from gpiozero.pins.pigpio import PiGPIOFactory
from gpiozero import DistanceSensor, Motor, AngularServo

# Custom modules
from imu import IMU
from encoder import Encoder
# from vision import 

# Pin factory
pin_factory = None

# Electrical Components
ULTRASONIC_SENSORS: List[DistanceSensor] = []
MOTOR: Motor = None
SERVO: AngularServo = None
IMU_SENSOR: IMU = None
ENCODER: Encoder = None

def setup() -> None:
    global ULTRASONIC_SENSORS, MOTOR, SERVO, IMU_SENSOR, ENCODER, pin_factory

    # Assuming `pigpiod` is running
    pin_factory = PiGPIOFactory()
    GPIO.setmode(GPIO.BOARD)

    # Initialise the ultrasonic sensor with the pin factory

    ULTRASONIC_SENSORS.extend([
        DistanceSensor(echo="BOARD10", trigger="BOARD12", pin_factory=pin_factory),  # front
        DistanceSensor(echo="BOARD24", trigger="BOARD22", pin_factory=pin_factory),  # right
        DistanceSensor(echo="BOARD36", trigger="BOARD37", pin_factory=pin_factory)   # left
    ])

    MOTOR = Motor("BOARD11", "BOARD13", pin_factory=pin_factory)

    # Using HS5945MG. See https://servodatabase.com/servo/hitec/hs-5945mg
    # Pulse width of 900-2100 microseconds
    # Pulse cycle of 20ms
    SERVO = AngularServo(
        "BOARD32", pin_factory=pin_factory,
        max_angle=120, min_angle=-120,
        min_pulse_width=900/1_000_000, max_pulse_width=2100/1_000_000
    )

    IMU_SENSOR = IMU(5, 3)
    ENCODER = Encoder(16)


def main() -> None:
    # while True:
    #     IMU_SENSOR.read_data()
    #     time.sleep(1)
    
    SERVO.angle = -25

    # for i in range(-120, 210, 30):
    #     SERVO.angle = i
    #     print(i)
    #     time.sleep(1)

    # SERVO.angle = 0

    # for i in range(0, 120, 20):
    #     MOTOR.forward(i / 100)
    #     time.sleep(0.3)
        

    dt = 0.1  # 10 Hz update rate

    while True:
        current_yaw = IMU_SENSOR.update(dt)
        steering_adjustment = IMU_SENSOR.get_steering_adjustment()

        # Limit steering adjustment
        steering_adjustment_rad = max(-20, min(20, steering_adjustment))

        # angle = steering_adjustment_to_angle(steering_adjustment)
        print(f"Current Yaw: {math.degrees(current_yaw):.2f}°, "
            f"Target Yaw: {math.degrees(IMU_SENSOR.target_yaw):.2f}°, "
            f"Steering Adjustment: {steering_adjustment:.2f}")
        print(steering_adjustment_rad * 180)


        SERVO.angle = steering_adjustment_rad * (180 / math.pi)

        # Here you would apply the steering_adjustment to your car's steering mechanism


        time.sleep(dt)
    # time.sleep(5)
    # time.sleep(1)
    # MOTOR.backward(0.1)
    # time.sleep(1)

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
        
        for i in range(MOTOR.value * 100, -20, -20):
            MOTOR.value = i/100
            print(MOTOR.value)
            time.sleep(0.5)

        MOTOR.value = 0.7
        time.sleep(0.3)
        MOTOR.value = 0.5
        time.sleep(0.3)
        MOTOR.value = 0.3
        time.sleep(0.3)
        MOTOR.value = 0.1
        time.sleep(0.3)
        MOTOR.value = 0
        time.sleep(0.3)

    except Exception as err:
        print(err)

    finally:
        print("Exiting. Cleaning up...")

        for us in ULTRASONIC_SENSORS:
            us.close()
        
        MOTOR.stop()
        MOTOR.close()
        SERVO.close()

        GPIO.cleanup()
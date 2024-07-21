import threading
import time
import math

import RPi.GPIO as GPIO


class DCMotor:

    def __init__(self, PWM_pin: int, DIR_pin: int, frequency: int=50) -> None:
        """Initialise a DC motor object.
        
        The MD10C motor driver gets 14.8V from the LiPo battery and outputs 12V to the DC motor.
        Take note that there's no reverse polarity protection.
        
        https://sg.cytron.io/p-10amp-5v-30v-dc-motor-driver
        
        LiPo battery: https://shopee.sg/(24h-Delivery)-IMAX-B6-80W-Balance-Battery-Charger-Lipo-NiMh-Li-ion-Ni-Cd-Digital-RC-Discharger-Power-Supply-T-Tamiya-XT60-Plug-15V-6A-Adapter-i.567389541.19272852943
        Fireproof LiPo Bag: https://shopee.sg/Fireproof-Waterproof-Lipo-Battery-Explosion-Proof-Safety-Bag-Fire-Resistant-for-Lipo-Battery-FPV-Racing-Drone-RC-Model-i.53026959.10341761540

        Args:
            PWM_pin (int): The GPIO pin number connected to the PWM input of the motor driver.
            DIR_pin (int): The GPIO pin number connected to the direction control input of the motor driver.
            frequency (int, optional): The PWM frequency in Hertz. Defaults to 50.
        """
        self.PWM_pin = PWM_pin
        self.DIR_pin = DIR_pin
        self.frequency = frequency
        self.PWM = 20

        # DC motor properties
        self.Rated_Load_RPM = 300
        self.Radius = 43.03

        self.current_direction = GPIO.HIGH
        GPIO.setup(self.DIR_pin, GPIO.OUT)

        GPIO.setup(self.PWM_pin, GPIO.OUT)
        self.motor = GPIO.PWM(PWM_pin, frequency)
        self.motor.start(self.PWM)

    def __getattr__(self, attr):
        # Delegate unknown attributes to GPIO.PWM object
        # e.g. start and stop, ChangeFrequency methods
        # in https://sourceforge.net/p/raspberry-gpio-python/wiki/PWM/
        return getattr(self.motor, attr)

    def estimate_speed(self) -> float:
        rpm = self.Rated_Load_RPM * self.PWM
        cir = 2 * math.pi * self.Radius
        speed_cm_s = (rpm / 60) * cir

        return speed_cm_s

    def set_pwm(self, PWM: int):
        if PWM != self.PWM:
            self.PWM = PWM
            self.motor.ChangeDutyCycle(PWM)

    def forward(self) -> None:
        self.current_direction = GPIO.HIGH
        GPIO.output(self.DIR_pin, self.current_direction)

    def backward(self) -> None:
        self.current_direction = GPIO.LOW
        GPIO.output(self.DIR_pin, self.current_direction)

    def reverse(self) -> None:
        self.current_direction = GPIO.LOW if self.current_direction == GPIO.HIGH else GPIO.HIGH
        GPIO.output(self.DIR_pin, self.current_direction)


class Servo:
    
    def __init__(self, PWM_pin: int, frequency: int = 50, min_pwm: float = 1, max_pwm: float = 13) -> None:
        self.PWM_pin = PWM_pin
        self.frequency = frequency
        self.min_pwm = min_pwm
        self.max_pwm = max_pwm
        self.angle_range = 180.0  # Assuming a standard servo with 180 degrees of motion

        GPIO.setup(self.PWM_pin, GPIO.OUT)
        self.servo = GPIO.PWM(self.PWM_pin, self.frequency)
        self.servo.start(self.min_pwm)  # Start with the servo at its minimum position
        self.current_pwm = self.min_pwm  # Track the current PWM value

        # Create a lock to ensure thread safety when changing PWM
        self.lock = threading.Lock()

        # Rate limiting variables
        self.last_update_time = time.time()
        self.update_interval = 0.1  # Adjust this value as needed for your application

    def calculate_pwm(self, angle: int) -> float:
        # Calculate the PWM value based on the specified angle and range
        pwm_range = self.max_pwm - self.min_pwm
        pwm_value = (angle / self.angle_range) * pwm_range + self.min_pwm
        return pwm_value

    def set_angle(self, angle: int) -> None:
        pwm = self.calculate_pwm(angle)

        if pwm != self.current_pwm:
            # Acquire the lock to ensure thread safety
            with self.lock:
                # Rate limit the servo updates
                current_time = time.time()
                if current_time - self.last_update_time >= self.update_interval:
                    self.servo.ChangeDutyCycle(pwm)
                    self.current_pwm = pwm
                    self.last_update_time = current_time

    def get_angle(self) -> int:
        # Return the current position of the servo as an angle
        with self.lock:
            result = (self.current_pwm - self.min_pwm) / (self.max_pwm - self.min_pwm) * self.angle_range
            return int(result)
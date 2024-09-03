import time
import math
import RPi.GPIO as GPIO

import cv2 as cv
import numpy as np

def euclidean_distance(color1, color2):
    """Calculates the Euclidean distance between two colour vectors.

    Args:
        color1 (Tuple[int, int, int]): The first colour vector in RGB order.
        color2 (Tuple[int, int, int]): The second colour vector in RGB order.

    Returns:
        float: The Euclidean distance between the two colour vectors.
    """
    
    return math.sqrt(sum((c1 - c2) ** 2 for c1, c2 in zip(color1, color2)))

def pulseIn(pin: int, value: GPIO.LOW or GPIO.HIGH) -> float:
    GPIO.setup(pin, GPIO.IN)

    while GPIO.input(pin) == value:
        pass

    while GPIO.input(pin) != value:
        pass

    start_time = time.time()

    while GPIO.input(pin) == value:
        pass

    stop_time = time.time()

    elapsed_time = stop_time - start_time

    return elapsed_time


def getRedBounds():
    # Define the RGB color you want to detect as red
    red_rgb = (68, 214, 44)

    # Convert RGB to HSV
    red_hsv = cv.cvtColor(np.uint8([[red_rgb]]), cv.COLOR_BGR2HSV)

    # Extract the hue value
    red_hue = red_hsv[0][0][0]

    # Define your desired range around the detected hue
    hue_tolerance = 10  # Adjust this as needed

    # Calculate the lower and upper bounds for red (1)
    lower_bound1 = (red_hue - hue_tolerance, 100, 100)
    upper_bound1 = (red_hue + hue_tolerance, 255, 255)

    # Calculate the lower and upper bounds for red (2) near hue 180
    # Ensure bounds are within the valid range of 0-179 for hue
    lower_bound2 = (0, 100, 100)
    upper_bound2 = ((red_hue + hue_tolerance - 180) % 180, 255, 255)

    print("Lower Bound 1:", lower_bound1)
    print("Upper Bound 1:", upper_bound1)
    print("Lower Bound 2:", lower_bound2)
    print("Upper Bound 2:", upper_bound2)


def soft_start(motor, start=0.0, end=1.0, step=0.1, delay=0.1):
    pwm_value = start
    while pwm_value <= end:
        motor.forward(pwm_value)
        time.sleep(delay)
        pwm_value += step

def soft_stop(motor, start=1.0, end=0.0, step=0.1, delay=0.1):
    pwm_value = start
    while pwm_value >= end:
        motor.forward(pwm_value)
        time.sleep(delay)
        pwm_value -= step
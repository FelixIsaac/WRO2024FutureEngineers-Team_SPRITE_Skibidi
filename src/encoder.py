import time
import RPi.GPIO as GPIO


# 18 degrees per on/off of smaller wheel (11.5mm).
# Turns the bigger wheel (differential gear) of 30mm
# The gear ratio is 18deg * (30/11.5) = 6.9 degrees
# which is 0.120428 rads. The radius of the actual
# car wheel is 42.750, using the arc length formula
# 0.120428 * 42.750 (arc length = rads * radius)
# it is 5.148297 mm per pulse
VEHICLE_CIRC_LENGTH_MM = 5.148297 # mm
class Encoder:
    def __init__(self, out_pin: int) -> None:
        self.out_pin = out_pin
        self.count = 0
        self.total_distance = 0.0
        
        GPIO.setup(self.out_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(self.out_pin, GPIO.BOTH, callback=self.update_count)

    def update_count(self, channel) -> None:
        self.count += 1
        self.total_distance = self.count * VEHICLE_CIRC_LENGTH_MM

    def distance(self) -> float:
        return self.total_distance

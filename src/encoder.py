import time
import threading
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
        self.last_state = None
        self.running = True

        # Set up the GPIO pin as input
        GPIO.setup(self.out_pin, GPIO.IN)# Start the thread to continuously update the count
        self.thread = threading.Thread(target=self._run)
        self.thread.start()

    def _run(self) -> None:
        print("running")

        GPIO.setmode(GPIO.BOARD)

        while self.running:
            self.update_count()
            time.sleep(0.01)  # Adjust the sleep time as needed

    def update_count(self) -> None:
        current_state = GPIO.input(self.out_pin)
        # print(current_state)

        if self.last_state is not None and self.last_state != current_state:
            # Edge detected (either rising or falling)
            self.count += 1
            self.total_distance = self.count * VEHICLE_CIRC_LENGTH_MM

        self.last_state = current_state

    def distance(self) -> float:
        return self.total_distance

    def stop(self) -> None:
        # Stop the thread
        self.running = False
        self.thread.join()

    def cleanup(self) -> None:
        # Ensure GPIO cleanup is done properly
        self.stop()
        GPIO.cleanup()
import time
import numpy as np
from smbus2 import SMBus
from numpy import eye, zeros
from filterpy.kalman import KalmanFilter

# Compass registers
CONFIG_A = 0x00
CONFIG_B = 0x01
MODE = 0x02
X_MSB = 0x03
Z_MSB = 0x05
Y_MSB = 0x07

PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H = 0x43
GYRO_YOUT_H = 0x45
GYRO_ZOUT_H = 0x47

class IMU:
    def __init__(self, SCL_pin: int, SDA_pin: int, address: hex = 0x68) -> None:
        self.SCL_pin = SCL_pin
        self.SDA_pin = SDA_pin
        self.address = address
        self.bus = 1

        # Bias correction variables
        self.gyro_bias = np.zeros(3)
        # self.calibrate_gyroscope()

        # Initialize Kalman Filter
        self.kf = KalmanFilter(dim_x=6, dim_z=6)

        # Set initial state and covariance
        self.kf.x = zeros(6)
        self.kf.P = eye(6)

        # Set state transition matrix F, observation matrix H, process noise covariance Q, and measurement noise covariance R
        self.kf.F = eye(6)
        self.kf.H = eye(6)
        self.kf.Q *= 0.01
        self.kf.R *= 0.1

        with SMBus(self.bus) as bus:
            bus.write_byte_data(self.address, PWR_MGMT_1, 0)

            bus.write_byte_data(self.address, CONFIG_A, 0x70)
            bus.write_byte_data(self.address, CONFIG_B, 0x20)
            bus.write_byte_data(self.address, MODE, 0x00)

    def calibrate_gyroscope(self, num_samples=1000):
        print("Calibrating gyroscope...")
        gyro_sum = np.zeros(3)

        for _ in range(num_samples):
            gyro_sum += np.array([
                self.read_word(GYRO_XOUT_H),
                self.read_word(GYRO_YOUT_H),
                self.read_word(GYRO_ZOUT_H)
            ])
            time.sleep(0.01)  # Adjust based on your sample rate

        self.gyro_bias = gyro_sum / num_samples
        print(f"Gyro bias: {self.gyro_bias}")

    def read_word(self, reg: hex) -> int:
        with SMBus(self.bus) as bus:
            high = bus.read_byte_data(self.address, reg)
            low = bus.read_byte_data(self.address, reg + 1)

            value = (high << 8) + low

            if value >= 0x8000:
                value = -((65535 - value) + 1)

            return value

    def read_data(self, reg: hex = 0) -> dict:
        accel_conversion = 9.80665 / 16384.0
        gyro_conversion = 1.0 / 131.0

        accel_x, accel_y, accel_z = (
            self.read_word(reg + ACCEL_XOUT_H) * accel_conversion,
            self.read_word(reg + ACCEL_YOUT_H) * accel_conversion,
            self.read_word(reg + ACCEL_ZOUT_H) * accel_conversion
        )

        gyro_x, gyro_y, gyro_z = (
            self.read_word(reg + GYRO_XOUT_H) * gyro_conversion - self.gyro_bias[0],
            self.read_word(reg + GYRO_YOUT_H) * gyro_conversion - self.gyro_bias[1],
            self.read_word(reg + GYRO_ZOUT_H) * gyro_conversion - self.gyro_bias[2]
        )

        self.kf.predict()
        self.kf.update([accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z])
        filtered_data = self.kf.x

        filtered_accel = filtered_data[:3]
        filtered_gyro = filtered_data[3:]

        return {
            'accel': filtered_accel,
            'gyro': filtered_gyro
        }

    def current_heading(self) -> np.array:
        """Reads magnetometer values and calculates the heading."""
        x, y, z =  self.read_data()['gyro']
        print(x, y ,z)

        # x = self.read_word(X_MSB)
        # y = self.read_word(Y_MSB)
        # z = self.read_word(Z_MSB)

        heading = np.arctan2(y, x) * (180 / np.pi)  # Convert to degrees

        if heading < 0:
            heading += 360

        return heading

# def adjust_course(self, target_heading: float, motor, servo) -> None:
#     """ Adjust the car's course to maintain the target heading """
#     current_heading = self.read_magnetometer()
#     print(f"Current Heading: {current_heading}°")

#     error = target_heading - current_heading

#     # Adjust the steering angle based on the error
#     if error > 180:
#         error -= 360
#     elif error < -180:
#         error += 360

#     # Simple proportional controller (P-Control)
#     steering_adjustment = error * 0.1  # Proportional gain (adjust as necessary)

#     if steering_adjustment > 20:
#         steering_adjustment = 20
#     elif steering_adjustment < -20:
#         steering_adjustment = -20

#     servo.angle = steering_adjustment

#     # Set motor speed forward (adjust as necessary)
#     motor.forward(0.5)  # 50% PWM
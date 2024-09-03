import time
import numpy as np
from smbus2 import SMBus
from numpy import eye, zeros
from filterpy.kalman import KalmanFilter
import math

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

        # Yaw angle to maintain
        self.current_yaw = 0.0
        self.target_yaw = 0.0

        # Bias correction variables
        self.gyro_bias = np.zeros(3)
        self.accel_bias = np.zeros(3)

        # Initialize Kalman Filter
        self.kf = KalmanFilter(dim_x=4, dim_z=6)

        # State: [roll, pitch, yaw, yaw_rate]
        self.kf.x = zeros(4)
        self.kf.P = eye(4) * 1000  # Large initial uncertainty

        # State transition matrix
        self.kf.F = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        # Measurement function
        self.kf.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
            [0, 0, 0, 0],
            [0, 0, 0, 0]
        ])

        # Measurement noise covariance
        self.kf.R = np.eye(6) * 0.1

        # Process noise covariance
        self.kf.Q = np.eye(4) * 0.01

        with SMBus(self.bus) as bus:
            # Wake up the MPU6050
            bus.write_byte_data(self.address, PWR_MGMT_1, 0)

        # self.calibrate()

    def calibrate(self, num_samples=1000):
        print("Calibrating IMU...")
        gyro_sum = np.zeros(3)
        accel_sum = np.zeros(3)

        for _ in range(num_samples):
            data = self.read_raw_data()
            gyro_sum += np.array(data['gyro'])
            accel_sum += np.array(data['accel'])
            time.sleep(0.01)

        self.gyro_bias = gyro_sum / num_samples
        self.accel_bias = accel_sum / num_samples - np.array([0, 0, 1])  # Assuming z-axis is vertical
        print(f"Gyro bias: {self.gyro_bias}")
        print(f"Accel bias: {self.accel_bias}")

        self.target_yaw = 0  # Set initial target yaw to 0
        self.current_yaw = 0
        print(f"Calibration complete. Target yaw set to {self.target_yaw}")

    def read_word(self, reg: hex = 0) -> int:
        with SMBus(self.bus) as bus:
            high = bus.read_byte_data(self.address, reg)
            low = bus.read_byte_data(self.address, reg + 1)

            value = (high << 8) + low

            if value >= 0x8000:
                value = -((65535 - value) + 1)

            return value

    def read_raw_data(self) -> dict:
        accel_conversion = 9.80665 / 16384.0
        gyro_conversion = np.pi / (180 * 131.0)  # Convert to radians/s

        accel = np.array([
            self.read_word(ACCEL_XOUT_H) * accel_conversion,
            self.read_word(ACCEL_YOUT_H) * accel_conversion,
            self.read_word(ACCEL_ZOUT_H) * accel_conversion
        ]) - self.accel_bias

        gyro = np.array([
            self.read_word(GYRO_XOUT_H) * gyro_conversion,
            self.read_word(GYRO_YOUT_H) * gyro_conversion,
            self.read_word(GYRO_ZOUT_H) * gyro_conversion
        ]) - self.gyro_bias

        return {
            'accel': accel,
            'gyro': gyro
        }

    def update(self, dt):
        data = self.read_raw_data()
        accel, gyro = data['accel'], data['gyro']

        # Calculate roll and pitch from accelerometer
        roll = math.atan2(accel[1], accel[2])
        pitch = math.atan2(-accel[0], math.sqrt(accel[1]**2 + accel[2]**2))

        # Predict
        self.kf.F[0, 3] = -dt * math.sin(self.kf.x[1])
        self.kf.F[1, 3] = dt * math.cos(self.kf.x[1]) * math.sin(self.kf.x[0])
        self.kf.F[2, 3] = dt * math.cos(self.kf.x[1]) * math.cos(self.kf.x[0])
        self.kf.predict()

        # Update
        z = np.array([roll, pitch, self.kf.x[2] + gyro[2]*dt, gyro[2], accel[0], accel[1]])
        self.kf.update(z)

        self.current_yaw = self.kf.x[2]
        return self.current_yaw

    def set_target_yaw(self, target):
        self.target_yaw = target

    def get_steering_adjustment(self):
        error = self.target_yaw - self.current_yaw
        # Normalize error to [-pi, pi]
        error = (error + np.pi) % (2 * np.pi) - np.pi
        return error * 5  # Proportional control, adjust gain as needed
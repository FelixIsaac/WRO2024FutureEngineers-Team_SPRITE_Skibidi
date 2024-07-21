from typing import TypedDict, List

from smbus2 import SMBus
from numpy import eye, zeros
from filterpy.kalman import KalmanFilter

"""
If you encounter the following error, FileNotFoundError: [Errno 2] No such file or directory: '/dev/i2c-1'

Update /boot/config.txt and reboot with the lines:
I2C
Add dtparam=i2c1=on

https://forums.raspberrypi.com/viewtopic.php?t=115080
"""

PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H = 0x43
GYRO_YOUT_H = 0x45
GYRO_ZOUT_H = 0x47

class AccelGyroData(TypedDict):
    accel: List[float]
    gyro: List[float]

class IMU:
    def __init__(self, SCL_pin: int, SDA_pin: int, address: hex = 0x68) -> None:
        self.SCL_pin = SCL_pin
        self.SDA_pin = SDA_pin
        self.address = address
        self.bus = 1

        # Initialize Kalman Filter
        self.kf = KalmanFilter(dim_x=6, dim_z=6)

        # Set initial state and covariance
        self.kf.x = zeros(6)   # Initial state: [accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z]
        self.kf.P = eye(6)     # Initial state covariance

        # Set state transition matrix F, observation matrix H, process noise covariance Q, and measurement noise covariance R
        self.kf.F = eye(6)     # Identity matrix for state transition (simple integration)
        self.kf.H = eye(6)     # Identity matrix for observation (direct measurements)
        self.kf.Q *= 0.01      # Adjust process noise covariance based on your system
        self.kf.R *= 0.1       # Adjust measurement noise covariance based on your system

        with SMBus(self.bus) as bus:
            # Wake up the MPU6050
            bus.write_byte_data(self.address, PWR_MGMT_1, 0)

    def read_word(self, reg: hex) -> int:
        with SMBus(self.bus) as bus:
            high = bus.read_byte_data(self.address, reg)
            low = bus.read_byte_data(self.address, reg + 1)

            # combine higher-order 8-bit and lower-order 8-bits into 16-bits
            value = (high << 8) + low

            # Convert to signed 16-bit integer
            if value >= 0x8000:
                value = -((65535 - value) + 1)

            return value

    def read_data(self, reg: hex = 0) -> AccelGyroData:
        accel_conversion = 9.80665 / 16384.0
        gyro_conversion = 1.0 / 131.0

        accel_x, accel_y, accel_z = (
            self.read_word(reg + ACCEL_XOUT_H) * accel_conversion,
            self.read_word(reg + ACCEL_YOUT_H) * accel_conversion,
            self.read_word(reg + ACCEL_ZOUT_H) * accel_conversion
        )

        gyro_x, gyro_y, gyro_z = (
            self.read_word(reg + GYRO_XOUT_H) * gyro_conversion,
            self.read_word(reg + GYRO_YOUT_H) * gyro_conversion,
            self.read_word(reg + GYRO_ZOUT_H) * gyro_conversion
        )

        # Use Kalman Filter for prediction and update
        self.kf.predict()
        self.kf.update([accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z])
        filtered_data = self.kf.x

        # Return the filtered data
        filtered_accel = filtered_data[:3]
        filtered_gyro = filtered_data[3:]
        
        """
        acceleration is in g but then converted to m/s^2
        gyroscope is angular velocity (degrees per second)
        """

        data: AccelGyroData = {
            'accel': filtered_accel,
            'gyro': filtered_gyro
        }

        return data
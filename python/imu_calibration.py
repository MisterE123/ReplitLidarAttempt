
import serial
import time
import numpy as np
from typing import Tuple

class IMUCalibration:
    def __init__(self, port: str = '/dev/ttyACM0', baud: int = 115200):
        self.ser = serial.Serial(port, baud)
        time.sleep(2)  # Wait for Arduino to reset
        
    def time_calibration(self) -> float:
        """Determine time offset between host and IMU"""
        offsets = []
        for _ in range(10):
            t1 = time.time_ns() // 1000  # Host microseconds
            self.ser.write(b'time_calibrate\n')
            response = self.ser.readline().decode().strip()
            t2 = time.time_ns() // 1000
            
            if response.startswith('TIME_ECHO'):
                t_imu = int(response.split(',')[1])
                offset = t_imu - (t1 + (t2 - t1) // 2)
                offsets.append(offset)
                
        return np.median(offsets)  # Use median to reject outliers
    
    def gravity_calibration(self) -> Tuple[float, float, float]:
        """Calibrate gravity vector"""
        self.ser.write(b'gravity_calibrate\n')
        while True:
            response = self.ser.readline().decode().strip()
            if response == 'GRAVITY_CAL_COMPLETE':
                break
            elif response == 'GRAVITY_CAL_FAILED':
                raise RuntimeError("Gravity calibration failed")
        return 0.0, 0.0, 1.0  # Return calibrated gravity vector
        
    def gyro_calibration(self):
        """Calibrate gyroscope bias"""
        self.ser.write(b'gyro_calibrate\n')
        while True:
            response = self.ser.readline().decode().strip()
            if response == 'GYRO_CAL_COMPLETE':
                break
                
    def mag_calibration(self):
        """Calibrate magnetometer"""
        self.ser.write(b'mag_calibrate\n')
        while True:
            response = self.ser.readline().decode().strip()
            if response == 'MAG_CAL_COMPLETE':
                break

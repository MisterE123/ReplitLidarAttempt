
import serial
import time
from typing import Tuple

class IMUAPI:
    def __init__(self, port: str = '/dev/ttyACM0', baud: int = 115200):
        """Initialize IMU API with serial connection parameters"""
        self.ser = serial.Serial(port, baud)
        time.sleep(2)  # Wait for Arduino reset
        
    def calibrate_time(self) -> float:
        """Calibrate time offset between host and IMU"""
        self.ser.write(b'time_calibrate\n')
        response = self.ser.readline().decode().strip()
        if response.startswith('TIME_ECHO'):
            return float(response.split(',')[1])
        raise RuntimeError("Time calibration failed")
        
    def calibrate_gravity(self) -> Tuple[float, float, float]:
        """Calibrate gravity vector with IMU held still"""
        self.ser.write(b'gravity_calibrate\n')
        response = self.ser.readline().decode().strip()
        if response != 'GRAVITY_CAL_START':
            raise RuntimeError("Gravity calibration failed to start")
            
        while True:
            response = self.ser.readline().decode().strip()
            if response == 'GRAVITY_CAL_COMPLETE':
                return (0.0, 0.0, 1.0)  # Default gravity vector
            elif response == 'GRAVITY_CAL_FAILED':
                raise RuntimeError("Gravity calibration failed")
        
    def calibrate_gyro(self) -> None:
        """Calibrate gyroscope bias with IMU held still"""
        self.ser.write(b'gyro_calibrate\n')
        response = self.ser.readline().decode().strip()
        if response != 'GYRO_CAL_START':
            raise RuntimeError("Gyro calibration failed to start")
            
        while True:
            response = self.ser.readline().decode().strip()
            if response == 'GYRO_CAL_COMPLETE':
                return
            elif response == 'GYRO_CAL_FAILED':
                raise RuntimeError("Gyro calibration failed")
            
    def calibrate_mag(self) -> None:
        """Calibrate magnetometer by rotating IMU through all orientations"""
        self.ser.write(b'mag_calibrate\n')
        response = self.ser.readline().decode().strip()
        if response != 'MAG_CAL_START':
            raise RuntimeError("Mag calibration failed to start")
            
        while True:
            response = self.ser.readline().decode().strip()
            if response == 'MAG_CAL_COMPLETE':
                return
            elif response == 'MAG_CAL_FAILED':
                raise RuntimeError("Mag calibration failed")

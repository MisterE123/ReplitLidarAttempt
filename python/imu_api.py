
import serial
import time
from typing import Tuple

class IMUAPI:
    def __init__(self, config_path: str = 'python/config.ini'):
        """Initialize IMU API with config file"""
        import configparser
        config = configparser.ConfigParser()
        config.read(config_path)
        
        port = config['IMU']['port']
        baud = int(config['IMU'].get('baud_rate', '115200'))
        self.still_delay = float(config['Calibration']['still_delay_seconds'])
        self.rotation_delay = float(config['Calibration']['rotation_delay_seconds'])
        
        self.ser = serial.Serial(port, baud)
        time.sleep(2)  # Wait for Arduino reset
        
    def calibrate_time(self) -> float:
        """Calibrate time offset between host and IMU"""
        self.ser.write(b'time_calibrate\n')
        response = self.ser.readline().decode().strip()
        if response.startswith('TIME_ECHO'):
            return float(response.split(',')[1])
        raise RuntimeError("Time calibration failed")
        
    def calibrate_still_sensors(self) -> None:
        """Calibrate gravity and gyro with IMU held still"""
        self.ser.write(b'still_calibrate\n')
        response = self.ser.readline().decode().strip()
        if response != 'STILL_CAL_START':
            raise RuntimeError("Still calibration failed to start")
            
        while True:
            response = self.ser.readline().decode().strip()
            if response == 'STILL_CAL_COMPLETE':
                return
            elif response == 'STILL_CAL_FAILED':
                raise RuntimeError("Still calibration failed")
            
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


from imu_calibration import IMUCalibration
from typing import Tuple

class IMUAPI:
    def __init__(self, port: str = '/dev/ttyACM0', baud: int = 115200):
        """Initialize IMU API with serial connection parameters"""
        self.imu = IMUCalibration(port, baud)
        
    def calibrate_time(self) -> float:
        """Calibrate time offset between host and IMU"""
        return self.imu.time_calibration()
        
    def calibrate_gravity(self) -> Tuple[float, float, float]:
        """Calibrate gravity vector with IMU held still"""
        return self.imu.gravity_calibration()
        
    def calibrate_gyro(self) -> None:
        """Calibrate gyroscope bias with IMU held still"""
        self.imu.gyro_calibration()
        
    def calibrate_mag(self) -> None:
        """Calibrate magnetometer by rotating IMU through all orientations"""
        self.imu.mag_calibration()

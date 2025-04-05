
from imu_calibration import IMUCalibration
from typing import Tuple

class IMUAPI:
    def __init__(self, port: str = '/dev/ttyACM0', baud: int = 115200):
        self.imu = IMUCalibration(port, baud)
        
    def calibrate_time(self) -> float:
        """Run time synchronization calibration"""
        return self.imu.time_calibration()
        
    def calibrate_gravity(self) -> Tuple[float, float, float]:
        """Run gravity vector calibration"""
        return self.imu.gravity_calibration()
        
    def calibrate_gyro(self) -> None:
        """Run gyroscope calibration"""
        self.imu.gyro_calibration()
        
    def calibrate_mag(self) -> None:
        """Run magnetometer calibration"""
        self.imu.mag_calibration()

# Example usage:
if __name__ == '__main__':
    imu_api = IMUAPI()
    time_offset = imu_api.calibrate_time()
    print(f"Time offset: {time_offset} microseconds")

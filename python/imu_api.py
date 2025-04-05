# Assuming a file structure like this:
# my_project/
# ├── python/
# │   └── imu_calibration.py
# └── main.py

# python/imu_calibration.py
class IMUCalibration:
    def __init__(self):
        pass

    def calibrate(self):
        print("IMU Calibration in progress...")

# main.py
from .python.imu_calibration import IMUCalibration # Updated import path


calibrator = IMUCalibration()
calibrator.calibrate()
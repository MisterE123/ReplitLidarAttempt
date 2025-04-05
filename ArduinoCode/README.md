
# IMU Data Collection System

This Arduino program collects calibrated data from the MKR IMU at high sample rates.

## Calibration Procedures
The system requires several calibration steps:

1. Time Synchronization
   - Establishes timestamp offset between host and IMU 
   - Run before each scanning session
   - Measures communication latency
   - Does not require the IMU to be stationary

2. Still Calibration
   - Combines gravity vector and gyro bias calibration
   - Place IMU flat and stationary
   - Takes ~1 second to complete

3. Magnetometer Calibration
   - Compensates for hard/soft iron effects
   - Rotate IMU through all orientations
   - Takes 10 seconds

## Data Protocol
Data blocks sent every 100ms:
```
BEGIN_DATA_BLOCK,magCount,accelCount,eulerCount,gyroCount
M,timestamp,x,y,z    // Calibrated magnetometer 
A,timestamp,x,y,z    // Gravity-compensated accelerometer
E,timestamp,x,y,z    // Euler angles
G,timestamp,x,y,z    // Bias-compensated gyroscope
END_DATA_BLOCK
```

## Commands
- time_calibrate: Start time sync
- still_calibrate: Calibrate gravity and gyroscope
- mag_calibrate: Calibrate magnetometer
- start_collection: Begin data streaming
- stop_collection: Stop data streaming

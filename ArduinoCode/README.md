
# IMU Data Collection System

This Arduino program collects data from the MKR IMU (Inertial Measurement Unit) and sends it through serial communication at 115200 baud rate.

## Sensor Data Types
- Accelerometer (100Hz): Linear acceleration with gravity compensation
- Gyroscope (100Hz): Angular velocity
- Magnetometer (20Hz): Magnetic field strength
- Euler angles (100Hz): Device orientation

## Buffer Sizes
- Accelerometer: 20 readings
- Gyroscope: 20 readings
- Magnetometer: 5 readings
- Euler angles: 20 readings

## Protocol

### Initialization
1. Send "calibrate" over serial to begin gravity calibration
2. Keep device still during calibration (~1 second)
3. System will output calibrated gravity vector

### Data Format
Data is sent in blocks every 100ms with the following format:

```
BEGIN_DATA_BLOCK,magCount,accelCount,eulerCount,gyroCount
M,timestamp,x,y,z    // Magnetometer readings
A,timestamp,x,y,z    // Accelerometer readings (gravity compensated)
E,timestamp,x,y,z    // Euler angles
G,timestamp,x,y,z    // Gyroscope readings
END_DATA_BLOCK
```

### Timestamps
- All timestamps are in microseconds (µs)
- Use timestamps for precise sensor data synchronization

## Key Functions
- `setup()`: Initializes IMU and waits for calibration command
- `calibrateGravity()`: Calibrates gravity vector for compensation
- `collectSensorData()`: Continuously reads sensor data into buffers
- `sendAllData()`: Transmits buffered data over serial
- `resetBuffers()`: Clears all data buffers after transmission

## Error Handling
- IMU initialization failure: Continuous loop with error message
- Buffer overflow protection: Forces data transmission if buffers fill
- Failed gravity calibration: Falls back to default values [0,0,1]

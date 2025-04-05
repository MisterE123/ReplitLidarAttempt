Configuration File (config.ini)

This file contains configuration settings for the IMU and LiDAR calibration system.Settings

[IMU] Section

port: Serial port where the IMU (Arduino) is connected (e.g., /dev/ttyACM0, COM3).
baud_rate: Communication speed with the IMU (default: 115200).

[LiDAR] Section
port: Serial port where the LiDAR sensor is connected (e.g., /dev/ttyUSB0, COM4).
baud_rate: Communication speed with the LiDAR (e.g., 230400).
enabled: Set to true to enable LiDAR connection and synchronization, false to disable. If disabled, the system will operate in IMU-only mode.

[Calibration] Section

still_delay_seconds: Time (seconds) given to the user to place the device down and let it settle before still calibration (gravity/gyro) begins (default: 5.0).
rotation_delay_seconds: Time (seconds) given to the user to prepare for motion calibration (magnetometer) instructions (default: 10.0).
lidar_sync_duration_s: (Optional) Overrides the default time window (seconds) used in the API to look for LiDAR packets during clock synchronization.

Example
```
[IMU]
port = /dev/ttyACM0
baud_rate = 115200

[LiDAR]
port = /dev/ttyUSB0
baud_rate = 230400
enabled = true

[Calibration]
still_delay_seconds = 3
rotation_delay_seconds = 8
# lidar_sync_duration_s = 0.3
```
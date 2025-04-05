
# Configuration File (config.ini)

This file contains configuration settings for the IMU calibration system.

## Settings

### IMU Section
- `port`: Serial port where the IMU is connected (e.g., /dev/ttyACM0)
- `baud_rate`: Communication speed with IMU (default: 115200)

### Calibration Section
- `still_delay_seconds`: Time given to user to place device down and let it settle before still calibration (default: 5)
- `rotation_delay_seconds`: Time given to user to prepare for rotation calibration instructions (default: 10)

## Example
```ini
[IMU]
port = /dev/ttyACM0
baud_rate = 115200

[Calibration]
still_delay_seconds = 3
rotation_delay_seconds = 3
```

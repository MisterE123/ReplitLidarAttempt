
# Configuration File (config.ini)

This file contains configuration settings for the IMU calibration system.

## Settings

### IMU Section
- `port`: Serial port where the IMU is connected (e.g., /dev/ttyACM0)
- `baud_rate`: Communication speed with IMU (default: 115200)

### Calibration Section
- `still_delay_seconds`: Time to wait before starting still calibration (default: 3)
- `rotation_delay_seconds`: Time to wait before starting rotation calibration (default: 3)

## Example
```ini
[IMU]
port = /dev/ttyACM0
baud_rate = 115200

[Calibration]
still_delay_seconds = 3
rotation_delay_seconds = 3
```

# Configuration File (`config.ini`)

This file contains configuration settings for the IMU, LiDAR, Stereo Camera, and data storage system.

## Settings

### `[IMU]` Section

* **`port`**: Serial port where the IMU (Arduino) is connected (e.g., `/dev/ttyACM0`, `COM3`).
* **`baud_rate`**: Communication speed with the IMU (default: `115200`).

### `[LiDAR]` Section

* **`port`**: Serial port where the LiDAR sensor is connected (e.g., `/dev/ttyUSB0`, `COM4`).
* **`baud_rate`**: Communication speed with the LiDAR (e.g., `230400`).
* **`enabled`**: Set to `true` to enable LiDAR connection and synchronization, `false` to disable. If disabled, the system will operate without LiDAR.

### `[Calibration]` Section

* **`still_delay_seconds`**: Time (seconds) given to the user to place the device down and let it settle before still calibration (gravity/gyro) begins (default: `5.0`).
* **`rotation_delay_seconds`**: Time (seconds) given to the user to prepare for motion calibration (magnetometer) instructions (default: `10.0`).
* **`lidar_sync_duration_s`**: (Optional) Overrides the default time window (seconds) used in the API to look for LiDAR packets during clock synchronization.

### `[StereoCamera]` Section

* **`left_image`**: Device path for the left camera of the stereo pair (e.g., `/dev/video0`).
* **`right_image`**: Device path for the right camera of the stereo pair (e.g., `/dev/video2`).
* **`upside_down`**: Set to `true` if the camera images are physically captured upside down and require software flipping during processing. Set to `false` otherwise.

### `[Storage]` Section

* **`scan_folder_path`**: The base directory path where all data for scan sessions (like the SQLite database and captured images) will be saved. Session-specific subfolders will likely be created under this path.

## Example

```ini
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

[StereoCamera]
left_image = /dev/video0
right_image = /dev/video2
upside_down = false

[Storage]
scan_folder_path = /home/pi/slam_data/scans

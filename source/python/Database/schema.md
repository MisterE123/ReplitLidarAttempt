## SLAM Data Database Schema (SQLite)

This document outlines a proposed SQLite database schema for storing synchronized 2D LiDAR, IMU, and Stereo Camera data collected on a Raspberry Pi 5, intended for 3D SLAM applications.

**Database System:** SQLite (recommended for its simplicity and performance in this embedded context). Consider enabling Write-Ahead Logging (`PRAGMA journal_mode=WAL;`) for better read/write concurrency.

**Timestamping Strategy:**
* **`pi_timestamp`**: Recorded using the Raspberry Pi's system clock (`time.time()` or `time.time_ns()`) when data arrives at the host or an event (like image capture) occurs. This provides a common timeline.
* **`sensor_timestamp`**: The internal timestamp provided by the sensor (IMU, LiDAR, potentially Camera) within its data packet/block or associated with its event. Stored alongside `pi_timestamp`.
* **IMU Time Calibration:** The result of the IMU time calibration (`calibrate_time()`) is stored per session to accurately relate the IMU's `sensor_timestamp` to the `pi_timestamp`.
* **LiDAR Time Synchronization:** Reference timestamps captured during IMU time calibration are stored per session to help relate the LiDAR's `sensor_timestamp` to the `pi_timestamp`.

**Image Storage Strategy:**
* Image data (e.g., from the stereo camera) is **not stored directly** in the database as BLOBs.
* Instead, images are saved as files on the filesystem (e.g., in session-specific folders).
* The database stores the **file paths** to these images (preferably relative paths from a defined session root directory).

**Table Definitions:**

**1. `ScanSession` Table**
* Stores metadata about each data collection session.

| Column                          | Type             | Description                                                                          |
| :------------------------------ | :--------------- | :----------------------------------------------------------------------------------- |
| `session_id`                    | INTEGER          | **Primary Key.** Unique identifier for the scan session.                               |
| `start_time`                    | REAL or INTEGER  | Pi system timestamp when the session began (e.g., seconds or ns since epoch).          |
| `end_time`                      | REAL or INTEGER  | Pi system timestamp when the session ended (NULLABLE).                               |
| `description`                   | TEXT             | Optional user-provided notes about the scan (NULLABLE).                              |
| `config_details`                | TEXT             | Relevant configuration parameters used (e.g., JSON dump of `config.ini`) (NULLABLE).  |
| `imu_time_calibration_result`   | INTEGER or TEXT  | Result from IMU `calibrate_time()` (e.g., IMU epoch timestamp) (NULLABLE).            |
| `imu_gravity_calibrated_status` | TEXT or INTEGER  | Status ('Success', 'Failed', 'Not Performed') of gravity calibration (NULLABLE).      |
| `imu_gyro_calibrated_status`    | TEXT or INTEGER  | Status ('Success', 'Failed', 'Not Performed') of gyro calibration (NULLABLE).         |
| `imu_mag_calibrated_status`     | TEXT or INTEGER  | Status ('Success', 'Failed', 'Not Performed') of magnetometer calibration (NULLABLE). |
| `lidar_time_calibration_info`   | TEXT             | Reference timestamps for LiDAR sync (e.g., JSON with Pi/LiDAR/IMU pairs) (NULLABLE). |

**2. `IMUMeasurement` Table**
* Stores individual sensor readings parsed from IMU data blocks.

| Column             | Type            | Description                                                                 |
| :----------------- | :-------------- | :-------------------------------------------------------------------------- |
| `measurement_id`   | INTEGER         | **Primary Key.** Unique ID for this specific measurement.                   |
| `session_id`       | INTEGER         | **Foreign Key** -> `ScanSession.session_id`.                                |
| `pi_timestamp`     | REAL or INTEGER | Pi system timestamp when the data *block* containing this was processed.      |
| `sensor_timestamp` | INTEGER         | Timestamp from the IMU data stream (e.g., microseconds since IMU epoch).    |
| `sensor_type`      | TEXT            | Type of reading ('M', 'A', 'E', 'G').                                       |
| `x`                | REAL            | Value for the first axis (e.g., Mag X, Accel X, Euler Roll, Gyro X).        |
| `y`                | REAL            | Value for the second axis (e.g., Mag Y, Accel Y, Euler Pitch, Gyro Y).      |
| `z`                | REAL            | Value for the third axis (e.g., Mag Z, Accel Z, Euler Yaw, Gyro Z).         |

**3. `LidarPacket` Table**
* Stores metadata for each received LiDAR data packet.

| Column             | Type            | Description                                                                 |
| :----------------- | :-------------- | :-------------------------------------------------------------------------- |
| `packet_id`        | INTEGER         | **Primary Key.** Unique ID for this LiDAR packet.                           |
| `session_id`       | INTEGER         | **Foreign Key** -> `ScanSession.session_id`.                                |
| `pi_timestamp`     | REAL or INTEGER | Pi system timestamp when this packet was processed.                         |
| `sensor_timestamp` | REAL            | Timestamp provided in the LiDAR packet header.                              |
| `speed`            | REAL            | Rotational speed reported in the packet (degrees/sec).                      |
| `start_angle`      | REAL            | Start angle reported in the packet (degrees).                               |
| `end_angle`        | REAL            | End angle reported in the packet (degrees).                                 |
| `raw_packet_data`  | BLOB            | Optional: Store the raw packet bytes for debugging/reprocessing (NULLABLE). |

**4. `LidarPoint` Table**
* Stores individual, processed points extracted from LiDAR packets.

| Column      | Type    | Description                                         |
| :---------- | :------ | :-------------------------------------------------- |
| `point_id`  | INTEGER | **Primary Key.** Unique ID for this specific point. |
| `packet_id` | INTEGER | **Foreign Key** -> `LidarPacket.packet_id`.         |
| `angle`     | REAL    | Calculated angle for this point (degrees).          |
| `distance`  | REAL    | Measured distance for this point (meters).          |
| `intensity` | INTEGER | Measured signal intensity for this point.           |

**5. `StereoImagePair` Table** (New Table)
* Stores references to captured stereo image pairs.

| Column             | Type            | Description                                                                   |
| :----------------- | :-------------- | :---------------------------------------------------------------------------- |
| `image_pair_id`    | INTEGER         | **Primary Key.** Unique ID for this stereo image pair record.                 |
| `session_id`       | INTEGER         | **Foreign Key** -> `ScanSession.session_id`.                                  |
| `pi_timestamp`     | REAL or INTEGER | Pi system timestamp when the image pair was captured/saved.                   |
| `sensor_timestamp` | REAL or INTEGER | Timestamp from the camera sensor itself, if available (NULLABLE).             |
| `left_image_path`  | TEXT            | Filesystem path (preferably relative) to the left image file.                 |
| `right_image_path` | TEXT            | Filesystem path (preferably relative) to the right image file.                |

**Concurrency Notes:**
* Each thread (IMU writer, LiDAR writer, Camera writer, UI reader) should use its own database connection.
* Writer threads must wrap inserts in transactions (`connection.commit()`).
* Enable WAL mode (`PRAGMA journal_mode=WAL;`) for better read/write concurrency.

**Indexing:**
* Create indices on foreign keys (`session_id`, `packet_id`).
* Create indices on timestamp columns (`pi_timestamp`) in `IMUMeasurement`, `LidarPacket`, and `StereoImagePair` tables to optimize time-based queries for the UI/SLAM process.
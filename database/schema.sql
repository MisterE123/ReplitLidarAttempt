
-- Core tables for raw sensor data
CREATE TABLE scanning_sessions (
    session_id INTEGER PRIMARY KEY,
    start_time TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    end_time TIMESTAMP,
    imu_start_micro BIGINT,    -- IMU's internal microsecond counter at session start
    lidar_start_micro BIGINT,  -- LIDAR's internal microsecond counter at session start
    notes TEXT
);

CREATE TABLE imu_readings (
    reading_id INTEGER PRIMARY KEY,
    session_id INTEGER,
    timestamp_micro BIGINT,  -- microseconds since session start
    accel_x REAL,
    accel_y REAL,
    accel_z REAL,
    gyro_x REAL,
    gyro_y REAL,
    gyro_z REAL,
    mag_x REAL,
    mag_y REAL,
    mag_z REAL,
    euler_x REAL,
    euler_y REAL,
    euler_z REAL,
    FOREIGN KEY (session_id) REFERENCES scanning_sessions(session_id)
);

CREATE TABLE lidar_scans (
    scan_id INTEGER PRIMARY KEY,
    session_id INTEGER,
    timestamp_micro BIGINT,  -- microseconds since session start
    angle_start REAL,  -- in radians
    angle_increment REAL,  -- in radians
    scan_data BLOB,  -- serialized array of distances
    FOREIGN KEY (session_id) REFERENCES scanning_sessions(session_id)
);

CREATE TABLE stereo_frames (
    frame_id INTEGER PRIMARY KEY,
    session_id INTEGER,
    timestamp_micro BIGINT,  -- microseconds since session start
    left_image BLOB,   -- compressed jpg
    right_image BLOB,  -- compressed jpg
    FOREIGN KEY (session_id) REFERENCES scanning_sessions(session_id)
);

-- Tables for SLAM results
CREATE TABLE slam_poses (
    pose_id INTEGER PRIMARY KEY,
    session_id INTEGER,
    timestamp_micro BIGINT,
    x REAL,
    y REAL,
    z REAL,
    quat_w REAL,
    quat_x REAL,
    quat_y REAL,
    quat_z REAL,
    confidence REAL,
    is_keyframe BOOLEAN,
    FOREIGN KEY (session_id) REFERENCES scanning_sessions(session_id)
);

-- Indices for faster querying
CREATE INDEX idx_imu_session_time ON imu_readings(session_id, timestamp_micro);
CREATE INDEX idx_lidar_session_time ON lidar_scans(session_id, timestamp_micro);
CREATE INDEX idx_stereo_session_time ON stereo_frames(session_id, timestamp_micro);
CREATE INDEX idx_poses_session_time ON slam_poses(session_id, timestamp_micro);

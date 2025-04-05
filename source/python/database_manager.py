import sqlite3
import configparser
import os
import time
import datetime
import re # For sanitizing filenames
from typing import Optional, Dict, Any, List, Tuple

class DataManager:
    """
    Manages the SQLite database for storing SLAM data (IMU, LiDAR, Stereo Images).
    Handles database creation, table setup, and data insertion based on config and schema.
    Organizes data into session-specific database files within named directories.
    """

    def __init__(self, config_path: str = 'config.ini'):
        """
        Initializes the DataManager by reading the storage path from the config.

        Args:
            config_path: Path to the configuration (.ini) file.

        Raises:
            FileNotFoundError: If the config file is not found.
            KeyError: If required keys are missing in the config file.
            ValueError: If config values are invalid.
        """
        print(f"Initializing DataManager from: {config_path}")
        self.config = configparser.ConfigParser()
        if not os.path.exists(config_path):
            raise FileNotFoundError(f"Configuration file not found: {config_path}")

        self.config.read(config_path)

        try:
            self.base_scan_folder_path = self.config['Storage']['scan_folder_path']
            if not self.base_scan_folder_path:
                raise ValueError("scan_folder_path cannot be empty in [Storage] section.")
            print(f"  Base Scan Folder Path: {self.base_scan_folder_path}")
        except KeyError as e:
            raise KeyError(f"Missing key in [Storage] section of '{config_path}': {e}")

        self.conn: Optional[sqlite3.Connection] = None
        self.cursor: Optional[sqlite3.Cursor] = None
        self.current_session_id: Optional[int] = None
        self.current_db_path: Optional[str] = None
        self.current_session_path: Optional[str] = None # Store the full session path
        self.current_session_name: Optional[str] = None # Store the actual folder name used

    @staticmethod
    def _sanitize_foldername(name: str) -> str:
        """Removes invalid characters for directory names and replaces spaces."""
        if not name:
            name = "unnamed_scan"
        # Remove characters that are typically invalid in directory names
        name = re.sub(r'[<>:"/\\|?*\^\+%\$#@!=`~\.]+', '', name)
        # Replace spaces and consecutive underscores/hyphens with a single underscore
        name = re.sub(r'\s+', '_', name)
        name = re.sub(r'[_]+', '_', name)
        name = re.sub(r'[-]+', '-', name)
        # Remove leading/trailing underscores/hyphens/spaces
        name = name.strip('_- ')
        # Handle empty string after sanitization
        if not name:
            name = "sanitized_scan"
        return name

    def start_new_session(self, session_name_base: str, config_details: str = "") -> Optional[int]:
        """
        Starts a new data collection session with a user-provided base name.
        - Creates a new session-specific directory based on the sanitized name (appends timestamp if needed).
        - Creates/Connects to a session-specific SQLite database file.
        - Creates tables if they don't exist.
        - Adds a record to the ScanSession table using the base name as description.
        - Stores the connection, cursor, session ID, and paths.

        Args:
            session_name_base: The user-provided base name/description for the session.
            config_details: Optional string (e.g., JSON) of config used.

        Returns:
            The session_id of the newly created session, or None on failure.
        """
        if self.conn:
            print("WARN: A session is already active. Please close it before starting a new one.")
            return None

        try:
            start_timestamp_ns = time.time_ns()
            start_time_dt = datetime.datetime.now()

            # --- Determine Session Folder Name ---
            sanitized_name = self._sanitize_foldername(session_name_base)
            session_path_attempt = os.path.join(self.base_scan_folder_path, sanitized_name)

            # Check if directory exists, append timestamp if it does
            folder_suffix = ""
            counter = 0
            while os.path.exists(session_path_attempt + folder_suffix):
                 # Append timestamp or counter to make unique
                 timestamp_suffix = start_time_dt.strftime("_%Y%m%d_%H%M%S")
                 counter_suffix = f"_{counter}" if counter > 0 else ""
                 folder_suffix = timestamp_suffix + counter_suffix
                 counter += 1
                 if counter > 10: # Safety break
                      print(f"WARN: Could not find unique folder name for {sanitized_name} after {counter} tries.")
                      # Fallback to purely timestamp based?
                      sanitized_name = start_time_dt.strftime("session_%Y%m%d_%H%M%S_%f")[:-3]
                      folder_suffix = ""
                      session_path_attempt = os.path.join(self.base_scan_folder_path, sanitized_name)
                      break # Use this timestamp name

            self.current_session_name = sanitized_name + folder_suffix
            self.current_session_path = os.path.join(self.base_scan_folder_path, self.current_session_name)
            self.current_db_path = os.path.join(self.current_session_path, "slam_data.db")
            # ---

            print(f"Starting new session. Path: {self.current_session_path}")
            os.makedirs(self.current_session_path, exist_ok=True) # Create session directory

            print(f"Connecting to database: {self.current_db_path}")
            self.conn = sqlite3.connect(self.current_db_path, isolation_level=None)
            self.cursor = self.conn.cursor()

            self.cursor.execute("PRAGMA journal_mode=WAL;")
            print("Database WAL mode enabled.")

            self._create_tables()

            # Insert session record, using the original base name as description
            sql = """
                INSERT INTO ScanSession (start_time, description, config_details)
                VALUES (?, ?, ?)
            """
            # Use session_name_base for description
            self.cursor.execute(sql, (start_timestamp_ns, session_name_base, config_details))
            self.current_session_id = self.cursor.lastrowid
            print(f"New session started with ID: {self.current_session_id} (Folder: {self.current_session_name})")
            return self.current_session_id

        except sqlite3.Error as e:
            print(f"ERROR: SQLite error during session start: {e}")
            self.close_session()
            return None
        except OSError as e:
            print(f"ERROR: OS error creating session directory {self.current_session_path}: {e}")
            self.close_session()
            return None
        except Exception as e:
            print(f"ERROR: Unexpected error during session start: {e}")
            traceback.print_exc()
            self.close_session()
            return None

    # _create_tables remains the same
    def _create_tables(self):
        """Creates database tables based on the schema if they don't exist."""
        if not self.cursor:
            raise ConnectionError("Database connection not established.")
        print("Creating database tables if they don't exist...")
        try:
            self.cursor.execute("""
                CREATE TABLE IF NOT EXISTS ScanSession (
                    session_id INTEGER PRIMARY KEY AUTOINCREMENT,
                    start_time INTEGER NOT NULL, -- Nanoseconds since epoch
                    end_time INTEGER,           -- Nanoseconds since epoch
                    description TEXT,
                    config_details TEXT,
                    imu_time_calibration_result INTEGER,
                    imu_gravity_calibrated_status TEXT,
                    imu_gyro_calibrated_status TEXT,
                    imu_mag_calibrated_status TEXT,
                    lidar_time_calibration_info TEXT
                )
            """)
            self.cursor.execute("""
                CREATE TABLE IF NOT EXISTS IMUMeasurement (
                    measurement_id INTEGER PRIMARY KEY AUTOINCREMENT,
                    session_id INTEGER NOT NULL,
                    pi_timestamp INTEGER NOT NULL, -- Nanoseconds since epoch
                    sensor_timestamp INTEGER NOT NULL, -- Microseconds from IMU
                    sensor_type TEXT NOT NULL CHECK(sensor_type IN ('M', 'A', 'E', 'G')),
                    x REAL NOT NULL, y REAL NOT NULL, z REAL NOT NULL,
                    FOREIGN KEY (session_id) REFERENCES ScanSession (session_id)
                )
            """)
            self.cursor.execute("CREATE INDEX IF NOT EXISTS idx_imu_pi_timestamp ON IMUMeasurement (pi_timestamp)")
            self.cursor.execute("""
                CREATE TABLE IF NOT EXISTS LidarPacket (
                    packet_id INTEGER PRIMARY KEY AUTOINCREMENT,
                    session_id INTEGER NOT NULL,
                    pi_timestamp INTEGER NOT NULL, -- Nanoseconds since epoch
                    sensor_timestamp REAL, speed REAL, start_angle REAL, end_angle REAL,
                    raw_packet_data BLOB,
                    FOREIGN KEY (session_id) REFERENCES ScanSession (session_id)
                )
            """)
            self.cursor.execute("CREATE INDEX IF NOT EXISTS idx_lidar_pi_timestamp ON LidarPacket (pi_timestamp)")
            self.cursor.execute("""
                CREATE TABLE IF NOT EXISTS LidarPoint (
                    point_id INTEGER PRIMARY KEY AUTOINCREMENT,
                    packet_id INTEGER NOT NULL, angle REAL NOT NULL, distance REAL NOT NULL, intensity INTEGER NOT NULL,
                    FOREIGN KEY (packet_id) REFERENCES LidarPacket (packet_id)
                )
            """)
            self.cursor.execute("CREATE INDEX IF NOT EXISTS idx_lidarpoint_packet_id ON LidarPoint (packet_id)")
            self.cursor.execute("""
                CREATE TABLE IF NOT EXISTS StereoImagePair (
                    image_pair_id INTEGER PRIMARY KEY AUTOINCREMENT,
                    session_id INTEGER NOT NULL,
                    pi_timestamp INTEGER NOT NULL, -- Nanoseconds since epoch
                    sensor_timestamp INTEGER,
                    left_image_path TEXT NOT NULL, right_image_path TEXT NOT NULL,
                    FOREIGN KEY (session_id) REFERENCES ScanSession (session_id)
                )
            """)
            self.cursor.execute("CREATE INDEX IF NOT EXISTS idx_stereo_pi_timestamp ON StereoImagePair (pi_timestamp)")
            print("Table creation check complete.")
        except sqlite3.Error as e:
            print(f"ERROR: SQLite error creating tables: {e}")
            raise

    # add_stereo_image_pair remains the same
    def add_stereo_image_pair(self, image_info: Dict[str, Any]) -> Optional[int]:
        """Adds a record for a captured stereo image pair to the database."""
        if not self.conn or not self.cursor or not self.current_session_id:
            print("ERROR: No active session. Cannot add stereo image pair.")
            return None
        required_keys = ['session_id', 'pi_timestamp', 'sensor_timestamp', 'left_image_path', 'right_image_path']
        if not all(key in image_info for key in required_keys):
             print(f"ERROR: Missing required keys in image_info dictionary. Expected: {required_keys}")
             return None
        if image_info['session_id'] != self.current_session_id:
             print(f"ERROR: Session ID mismatch. Expected {self.current_session_id}, got {image_info['session_id']}.")
             return None
        sql = """
            INSERT INTO StereoImagePair (session_id, pi_timestamp, sensor_timestamp, left_image_path, right_image_path)
            VALUES (?, ?, ?, ?, ?)
        """
        try:
            self.cursor.execute(sql, (
                image_info['session_id'], image_info['pi_timestamp'], image_info['sensor_timestamp'],
                image_info['left_image_path'], image_info['right_image_path']
            ))
            return self.cursor.lastrowid
        except sqlite3.Error as e:
            print(f"ERROR: SQLite error adding stereo image pair: {e}")
            return None
        except Exception as e:
            print(f"ERROR: Unexpected error adding stereo image pair: {e}")
            return None

    # add_imu_measurement remains the same
    def add_imu_measurement(self, pi_ts: int, sensor_ts: int, type: str, x: float, y: float, z: float) -> Optional[int]:
        """Adds a single IMU measurement."""
        if not self.conn or not self.cursor or not self.current_session_id: return None
        sql = """INSERT INTO IMUMeasurement (session_id, pi_timestamp, sensor_timestamp, sensor_type, x, y, z)
                 VALUES (?, ?, ?, ?, ?, ?, ?)"""
        try:
            self.cursor.execute(sql, (self.current_session_id, pi_ts, sensor_ts, type, x, y, z))
            return self.cursor.lastrowid
        except sqlite3.Error as e: print(f"ERROR: SQLite error adding IMU measurement: {e}"); return None

    # add_lidar_packet_and_points remains the same
    def add_lidar_packet_and_points(self, packet_info: Dict[str, Any], points: List[Dict[str, Any]]) -> Optional[int]:
        """Adds LiDAR packet info and associated points."""
        if not self.conn or not self.cursor or not self.current_session_id: return None
        packet_sql = """INSERT INTO LidarPacket (session_id, pi_timestamp, sensor_timestamp, speed, start_angle, end_angle)
                        VALUES (?, ?, ?, ?, ?, ?)"""
        point_sql = """INSERT INTO LidarPoint (packet_id, angle, distance, intensity)
                       VALUES (?, ?, ?, ?)"""
        try:
            self.cursor.execute(packet_sql, (
                self.current_session_id, packet_info['pi_timestamp'], packet_info['sensor_timestamp'],
                packet_info['speed'], packet_info['start_angle'], packet_info['end_angle']
            ))
            packet_id = self.cursor.lastrowid
            if packet_id and points: # Check if points list is not empty
                point_data = [(packet_id, p['angle'], p['distance'], p['intensity']) for p in points]
                self.cursor.executemany(point_sql, point_data)
            return packet_id
        except sqlite3.Error as e: print(f"ERROR: SQLite error adding LiDAR data: {e}"); return None

    # update_session_calibration_status remains the same
    def update_session_calibration_status(self, imu_cal_result=None, grav_status=None, gyro_status=None, mag_status=None, lidar_cal_info=None):
         """Updates calibration status fields for the current session."""
         if not self.conn or not self.cursor or not self.current_session_id: return False
         updates = []
         params = []
         if imu_cal_result is not None: updates.append("imu_time_calibration_result = ?"); params.append(imu_cal_result)
         if grav_status is not None: updates.append("imu_gravity_calibrated_status = ?"); params.append(grav_status)
         if gyro_status is not None: updates.append("imu_gyro_calibrated_status = ?"); params.append(gyro_status)
         if mag_status is not None: updates.append("imu_mag_calibrated_status = ?"); params.append(mag_status)
         if lidar_cal_info is not None: updates.append("lidar_time_calibration_info = ?"); params.append(lidar_cal_info)
         if not updates: return True
         sql = f"UPDATE ScanSession SET {', '.join(updates)} WHERE session_id = ?"
         params.append(self.current_session_id)
         try:
             self.cursor.execute(sql, tuple(params))
             return True
         except sqlite3.Error as e:
             print(f"ERROR: SQLite error updating session calibration status: {e}")
             return False

    # end_current_session remains the same
    def end_current_session(self) -> bool:
        """Ends the current session by recording the end time and closing."""
        if not self.conn or not self.cursor or not self.current_session_id:
            # print("WARN: No active session to end.") # Reduce noise
            return False
        print(f"Ending session ID: {self.current_session_id}")
        try:
            end_timestamp_ns = time.time_ns()
            sql = "UPDATE ScanSession SET end_time = ? WHERE session_id = ?"
            self.cursor.execute(sql, (end_timestamp_ns, self.current_session_id))
            print("Session end time recorded.")
            return self.close_session()
        except sqlite3.Error as e:
            print(f"ERROR: SQLite error updating session end time: {e}")
            self.close_session()
            return False

    # close_session remains the same
    def close_session(self) -> bool:
        """Closes the database connection."""
        closed = False
        if self.cursor:
            try: self.cursor.close()
            except Exception as e: print(f"WARN: Error closing DB cursor: {e}")
            self.cursor = None
        if self.conn:
            try:
                self.conn.close()
                print(f"Database connection closed: {self.current_db_path}")
                closed = True
            except Exception as e: print(f"WARN: Error closing DB connection: {e}")
            self.conn = None
        self.current_session_id = None
        self.current_db_path = None
        self.current_session_path = None
        self.current_session_name = None
        return closed


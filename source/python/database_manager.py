import sqlite3
import configparser
import os
import time
import datetime
from typing import Optional, Dict, Any, List, Tuple

class DataManager:
    """
    Manages the SQLite database for storing SLAM data (IMU, LiDAR, Stereo Images).
    Handles database creation, table setup, and data insertion based on config and schema.
    Organizes data into session-specific database files.
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

        # --- Read Storage Config ---
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

    def start_new_session(self, description: str = "", config_details: str = "") -> Optional[int]:
        """
        Starts a new data collection session.
        - Creates a new session-specific directory.
        - Creates/Connects to a session-specific SQLite database file.
        - Creates tables if they don't exist.
        - Adds a record to the ScanSession table.
        - Stores the connection, cursor, and session ID for future inserts.

        Args:
            description: Optional description for the session.
            config_details: Optional string (e.g., JSON) of config used.

        Returns:
            The session_id of the newly created session, or None on failure.
        """
        if self.conn:
            print("WARN: A session is already active. Please close it before starting a new one.")
            # Or automatically close the previous one? For now, require explicit close.
            return None

        try:
            start_timestamp_ns = time.time_ns() # Use nanoseconds
            start_time_dt = datetime.datetime.now()
            session_folder_name = start_time_dt.strftime("session_%Y%m%d_%H%M%S")
            session_path = os.path.join(self.base_scan_folder_path, session_folder_name)
            self.current_db_path = os.path.join(session_path, "slam_data.db")

            print(f"Starting new session. Path: {session_path}")
            os.makedirs(session_path, exist_ok=True) # Create session directory

            print(f"Connecting to database: {self.current_db_path}")
            # isolation_level=None -> Autocommit mode (can be changed later if needed)
            # Or handle transactions manually with conn.commit()
            self.conn = sqlite3.connect(self.current_db_path, isolation_level=None)
            self.cursor = self.conn.cursor()

            # Enable WAL mode for better concurrency
            self.cursor.execute("PRAGMA journal_mode=WAL;")
            print("Database WAL mode enabled.")

            # Create tables
            self._create_tables()

            # Insert session record
            sql = """
                INSERT INTO ScanSession (start_time, description, config_details)
                VALUES (?, ?, ?)
            """
            self.cursor.execute(sql, (start_timestamp_ns, description, config_details))
            # self.conn.commit() # Needed if isolation_level is not None
            self.current_session_id = self.cursor.lastrowid
            print(f"New session started with ID: {self.current_session_id}")
            return self.current_session_id

        except sqlite3.Error as e:
            print(f"ERROR: SQLite error during session start: {e}")
            self.close_session() # Attempt cleanup
            return None
        except OSError as e:
            print(f"ERROR: OS error creating session directory {session_path}: {e}")
            self.close_session() # Attempt cleanup
            return None
        except Exception as e:
            print(f"ERROR: Unexpected error during session start: {e}")
            import traceback
            traceback.print_exc()
            self.close_session() # Attempt cleanup
            return None

    def _create_tables(self):
        """Creates database tables based on the schema if they don't exist."""
        if not self.cursor:
            raise ConnectionError("Database connection not established.")

        print("Creating database tables if they don't exist...")
        try:
            # ScanSession Table
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

            # IMUMeasurement Table
            self.cursor.execute("""
                CREATE TABLE IF NOT EXISTS IMUMeasurement (
                    measurement_id INTEGER PRIMARY KEY AUTOINCREMENT,
                    session_id INTEGER NOT NULL,
                    pi_timestamp INTEGER NOT NULL, -- Nanoseconds since epoch
                    sensor_timestamp INTEGER NOT NULL, -- Microseconds from IMU
                    sensor_type TEXT NOT NULL CHECK(sensor_type IN ('M', 'A', 'E', 'G')),
                    x REAL NOT NULL,
                    y REAL NOT NULL,
                    z REAL NOT NULL,
                    FOREIGN KEY (session_id) REFERENCES ScanSession (session_id)
                )
            """)
            # Index for faster time queries
            self.cursor.execute("CREATE INDEX IF NOT EXISTS idx_imu_pi_timestamp ON IMUMeasurement (pi_timestamp)")


            # LidarPacket Table
            self.cursor.execute("""
                CREATE TABLE IF NOT EXISTS LidarPacket (
                    packet_id INTEGER PRIMARY KEY AUTOINCREMENT,
                    session_id INTEGER NOT NULL,
                    pi_timestamp INTEGER NOT NULL, -- Nanoseconds since epoch
                    sensor_timestamp REAL,         -- Seconds from LiDAR
                    speed REAL,
                    start_angle REAL,
                    end_angle REAL,
                    raw_packet_data BLOB,
                    FOREIGN KEY (session_id) REFERENCES ScanSession (session_id)
                )
            """)
            # Index for faster time queries
            self.cursor.execute("CREATE INDEX IF NOT EXISTS idx_lidar_pi_timestamp ON LidarPacket (pi_timestamp)")


            # LidarPoint Table
            self.cursor.execute("""
                CREATE TABLE IF NOT EXISTS LidarPoint (
                    point_id INTEGER PRIMARY KEY AUTOINCREMENT,
                    packet_id INTEGER NOT NULL,
                    angle REAL NOT NULL,
                    distance REAL NOT NULL,
                    intensity INTEGER NOT NULL,
                    FOREIGN KEY (packet_id) REFERENCES LidarPacket (packet_id)
                )
            """)
            # Index for faster packet lookups
            self.cursor.execute("CREATE INDEX IF NOT EXISTS idx_lidarpoint_packet_id ON LidarPoint (packet_id)")


            # StereoImagePair Table
            self.cursor.execute("""
                CREATE TABLE IF NOT EXISTS StereoImagePair (
                    image_pair_id INTEGER PRIMARY KEY AUTOINCREMENT,
                    session_id INTEGER NOT NULL,
                    pi_timestamp INTEGER NOT NULL, -- Nanoseconds since epoch
                    sensor_timestamp INTEGER,      -- Optional, from camera sensor
                    left_image_path TEXT NOT NULL, -- Relative path
                    right_image_path TEXT NOT NULL, -- Relative path
                    FOREIGN KEY (session_id) REFERENCES ScanSession (session_id)
                )
            """)
            # Index for faster time queries
            self.cursor.execute("CREATE INDEX IF NOT EXISTS idx_stereo_pi_timestamp ON StereoImagePair (pi_timestamp)")

            print("Table creation check complete.")
            # self.conn.commit() # Needed if isolation_level is not None

        except sqlite3.Error as e:
            print(f"ERROR: SQLite error creating tables: {e}")
            raise # Re-raise the error to be handled by the caller

    def add_stereo_image_pair(self, image_info: Dict[str, Any]) -> Optional[int]:
        """
        Adds a record for a captured stereo image pair to the database.

        Args:
            image_info: A dictionary matching the output of
                        StereoCameraSystem.capture_and_save_pair.
                        Expected keys: 'session_id', 'pi_timestamp', 'sensor_timestamp',
                                       'left_image_path', 'right_image_path'.

        Returns:
            The row ID of the inserted record, or None on failure.
        """
        if not self.conn or not self.cursor or not self.current_session_id:
            print("ERROR: No active session. Cannot add stereo image pair.")
            return None

        # Validate input dictionary keys
        required_keys = ['session_id', 'pi_timestamp', 'sensor_timestamp', 'left_image_path', 'right_image_path']
        if not all(key in image_info for key in required_keys):
             print(f"ERROR: Missing required keys in image_info dictionary. Expected: {required_keys}")
             return None

        # Ensure the session_id matches the current active session
        if image_info['session_id'] != self.current_session_id:
             print(f"ERROR: Session ID mismatch. Expected {self.current_session_id}, got {image_info['session_id']}.")
             return None

        sql = """
            INSERT INTO StereoImagePair (session_id, pi_timestamp, sensor_timestamp, left_image_path, right_image_path)
            VALUES (?, ?, ?, ?, ?)
        """
        try:
            self.cursor.execute(sql, (
                image_info['session_id'],
                image_info['pi_timestamp'],
                image_info['sensor_timestamp'],
                image_info['left_image_path'],
                image_info['right_image_path']
            ))
            # self.conn.commit() # Needed if isolation_level is not None
            inserted_id = self.cursor.lastrowid
            # print(f"DEBUG: Added StereoImagePair record with ID: {inserted_id}") # Optional debug
            return inserted_id
        except sqlite3.Error as e:
            print(f"ERROR: SQLite error adding stereo image pair: {e}")
            return None
        except Exception as e:
            print(f"ERROR: Unexpected error adding stereo image pair: {e}")
            return None

    # --- Placeholder methods for other data types ---
    def add_imu_measurement(self, pi_ts: int, sensor_ts: int, type: str, x: float, y: float, z: float) -> Optional[int]:
        """Placeholder: Adds a single IMU measurement."""
        if not self.conn or not self.cursor or not self.current_session_id: return None
        sql = """INSERT INTO IMUMeasurement (session_id, pi_timestamp, sensor_timestamp, sensor_type, x, y, z)
                 VALUES (?, ?, ?, ?, ?, ?, ?)"""
        try:
            self.cursor.execute(sql, (self.current_session_id, pi_ts, sensor_ts, type, x, y, z))
            return self.cursor.lastrowid
        except sqlite3.Error as e: print(f"ERROR: SQLite error adding IMU measurement: {e}"); return None

    def add_lidar_packet_and_points(self, packet_info: Dict[str, Any], points: List[Dict[str, Any]]) -> Optional[int]:
        """Placeholder: Adds LiDAR packet info and associated points."""
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
            if packet_id:
                point_data = [(packet_id, p['angle'], p['distance'], p['intensity']) for p in points]
                self.cursor.executemany(point_sql, point_data)
            return packet_id
        except sqlite3.Error as e: print(f"ERROR: SQLite error adding LiDAR data: {e}"); return None # Consider rolling back transaction if commit is manual

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

         if not updates: return True # Nothing to update

         sql = f"UPDATE ScanSession SET {', '.join(updates)} WHERE session_id = ?"
         params.append(self.current_session_id)
         try:
             self.cursor.execute(sql, tuple(params))
             # print(f"DEBUG: Updated calibration status for session {self.current_session_id}")
             return True
         except sqlite3.Error as e:
             print(f"ERROR: SQLite error updating session calibration status: {e}")
             return False


    def end_current_session(self) -> bool:
        """
        Ends the current session by recording the end time in the database
        and closing the connection.
        """
        if not self.conn or not self.cursor or not self.current_session_id:
            print("WARN: No active session to end.")
            return False

        print(f"Ending session ID: {self.current_session_id}")
        try:
            end_timestamp_ns = time.time_ns()
            sql = "UPDATE ScanSession SET end_time = ? WHERE session_id = ?"
            self.cursor.execute(sql, (end_timestamp_ns, self.current_session_id))
            # self.conn.commit() # Needed if isolation_level is not None
            print("Session end time recorded.")
            return self.close_session() # Close connection after updating
        except sqlite3.Error as e:
            print(f"ERROR: SQLite error updating session end time: {e}")
            # Still try to close the connection
            self.close_session()
            return False

    def close_session(self) -> bool:
        """Closes the database connection."""
        closed = False
        if self.cursor:
            try:
                self.cursor.close()
                # print("DEBUG: DB cursor closed.")
            except Exception as e:
                print(f"WARN: Error closing DB cursor: {e}")
            self.cursor = None
        if self.conn:
            try:
                # self.conn.commit() # Commit any final changes if not in autocommit
                self.conn.close()
                print(f"Database connection closed: {self.current_db_path}")
                closed = True
            except Exception as e:
                print(f"WARN: Error closing DB connection: {e}")
            self.conn = None
        self.current_session_id = None
        self.current_db_path = None
        return closed


# --- Example Usage ---
if __name__ == "__main__":
    CONFIG_FILE = 'config.ini' # Make sure this file exists and is configured

    # Ensure a dummy config exists for basic testing
    if not os.path.exists(CONFIG_FILE):
        print(f"WARNING: {CONFIG_FILE} not found. Creating a dummy one for testing.")
        print("         >>> Please edit it with your actual camera devices and paths! <<<")
        dummy_config = configparser.ConfigParser()
        dummy_config['IMU'] = {'port': '/dev/ttyACM0', 'baud_rate': '115200'}
        dummy_config['LiDAR'] = {'port': '/dev/ttyUSB0', 'baud_rate': '230400', 'enabled': 'true'}
        dummy_config['Calibration'] = {'still_delay_seconds': '5', 'rotation_delay_seconds': '10'}
        dummy_config['StereoCamera'] = {'left_image': '0', 'right_image': '2', 'upside_down': 'false'}
        dummy_config['Storage'] = {'scan_folder_path': './scan_data_output'}
        with open(CONFIG_FILE, 'w') as configfile:
            dummy_config.write(configfile)
        if not os.path.exists('./scan_data_output'):
             os.makedirs('./scan_data_output')

    db_manager: Optional[DataManager] = None
    session_id: Optional[int] = None

    try:
        # 1. Initialize DataManager
        db_manager = DataManager(config_path=CONFIG_FILE)

        # 2. Start a new session
        session_id = db_manager.start_new_session(
            description="Test scan session",
            config_details="Sample config details here (e.g., JSON string)"
        )

        if session_id is not None:
            print(f"\n--- Session {session_id} Active ---")

            # 3. Simulate getting data from StereoCameraSystem
            #    (In a real application, you would call stereo_system.capture_and_save_pair)
            print("\nSimulating stereo image capture...")
            simulated_pi_time_ns = time.time_ns()
            simulated_timestamp_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
            simulated_session_folder = f"session_{session_id:04d}"
            simulated_relative_base = os.path.join(simulated_session_folder, "stereo_images", simulated_timestamp_str)

            # NOTE: These paths MUST match the structure created by StereoCameraSystem
            simulated_db_info = {
                'session_id': session_id,
                'pi_timestamp': simulated_pi_time_ns,
                'sensor_timestamp': None, # Simulate no sensor timestamp
                'left_image_path': os.path.join(simulated_relative_base, f"left_{simulated_timestamp_str}.png"),
                'right_image_path': os.path.join(simulated_relative_base, f"right_{simulated_timestamp_str}.png")
            }
            print(f"Simulated DB Info: {simulated_db_info}")

            # 4. Add the simulated stereo image pair data to the DB
            insert_id = db_manager.add_stereo_image_pair(simulated_db_info)
            if insert_id:
                print(f"Successfully added stereo image pair record with ID: {insert_id}")
            else:
                print("Failed to add stereo image pair record.")

            # 5. (Optional) Add simulated IMU/LiDAR data using placeholder methods
            print("\nSimulating adding other data types...")
            imu_id = db_manager.add_imu_measurement(time.time_ns(), 12345678, 'A', 0.1, 0.0, 9.8)
            if imu_id: print(f"Added IMU record ID: {imu_id}")

            # Simulate LiDAR packet and points
            lidar_packet = {
                'pi_timestamp': time.time_ns(), 'sensor_timestamp': 50.5,
                'speed': 5.0, 'start_angle': 0.0, 'end_angle': 359.0
            }
            lidar_points = [
                {'angle': 0.0, 'distance': 2.5, 'intensity': 100},
                {'angle': 1.0, 'distance': 2.6, 'intensity': 110}
            ]
            lidar_packet_id = db_manager.add_lidar_packet_and_points(lidar_packet, lidar_points)
            if lidar_packet_id: print(f"Added LiDAR packet ID: {lidar_packet_id} with {len(lidar_points)} points")

            # 6. Update calibration status (example)
            print("\nUpdating session calibration status...")
            cal_success = db_manager.update_session_calibration_status(
                 imu_cal_result=1678886400000000, # Example timestamp
                 grav_status='Success',
                 gyro_status='Success',
                 mag_status='Failed',
                 lidar_cal_info='{"sync_offset_ns": 12345, "quality": "good"}' # Example JSON
            )
            if cal_success: print("Calibration status updated.")
            else: print("Failed to update calibration status.")


        else:
            print("\n--- Failed to start session ---")

    except (FileNotFoundError, KeyError, ValueError) as e:
        print(f"\nERROR: Configuration Error - {e}")
    except Exception as e:
        print(f"\nERROR: An unexpected error occurred: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 7. Ensure the session is ended and connection closed
        if db_manager and db_manager.current_session_id is not None:
            print("\n--- Ending Session ---")
            db_manager.end_current_session()
        elif db_manager:
             # Ensure connection is closed even if session didn't start properly
             db_manager.close_session()


    print("\nDatabase Manager Script Finished.")

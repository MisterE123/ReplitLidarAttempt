import threading
import queue
import time
import configparser
import os
import sys
import traceback
import sqlite3

# --- Import Custom Modules ---
try:
    from imu_api import IMUAPI, IMUCommunicationError, IMUTimeoutError, IMUCalibrationError
    from lidar_api import LidarAPI, LidarCommunicationError
    # Assumes updated script saved as stereo_camera_api.py
    from stereo_camera_api import StereoCameraSystem
    # Assumes updated script saved as database_manager.py
    from database_manager import DataManager
except ImportError as e:
    print(f"ERROR: Failed to import necessary API modules: {e}")
    sys.exit(1)
except FileNotFoundError as e:
     print(f"ERROR: Config file issue during module import: {e}")
     sys.exit(1)
except (KeyError, ValueError) as e:
     print(f"ERROR: Config file value issue during module import: {e}")
     sys.exit(1)

# --- Worker Functions ---

# imu_worker remains the same as in main_collector_refactored_v1
def imu_worker(config_path: str, imu_q: queue.Queue, stop: threading.Event, db_manager: DataManager):
    """IMU data collection thread."""
    print("[IMU Worker] Thread started.")
    imu_api: Optional[IMUAPI] = None
    sync_result = None
    try:
        imu_api = IMUAPI(config_path=config_path)
        print("[IMU Worker] IMU API Initialized.")
        print("[IMU Worker] Performing clock sync...")
        sync_result = imu_api.calibrate_clocks()
        if db_manager and db_manager.current_session_id is not None:
             lidar_info_str = f"Status: {sync_result.get('status')}, Msg: {sync_result.get('message')}"
             db_manager.update_session_calibration_status(
                  imu_cal_result=sync_result.get('imu_sensor_timestamp_at_cal_us'),
                  lidar_cal_info=lidar_info_str )
             print(f"[IMU Worker] Updated DB with clock sync status for session {db_manager.current_session_id}")
        else: print("[IMU Worker] WARN: DB Manager not ready, cannot update clock sync status.")
        if sync_result['status'] == 'Failed':
             print(f"[IMU Worker] CRITICAL: Clock sync failed: {sync_result['message']}. Stopping."); return
        print(f"[IMU Worker] Clock sync status: {sync_result['status']}")
        if not imu_api.start_collection():
            print("[IMU Worker] Failed to start IMU data collection. Stopping thread."); return
        print("[IMU Worker] IMU data collection started.")
        while not stop.is_set():
            data_block = imu_api.read_data_block(read_timeout_seconds=0.5)
            block_pi_timestamp_ns = time.time_ns()
            if data_block:
                for sensor_type, readings in data_block.items():
                    for reading in readings:
                        try:
                            item = (block_pi_timestamp_ns, reading['timestamp'], sensor_type,
                                    reading['x'], reading['y'], reading['z'])
                            imu_q.put(item, timeout=0.1)
                        except queue.Full: print("[IMU Worker] WARN: IMU queue full. Discarding measurement.")
                        except KeyError: print(f"[IMU Worker] WARN: Malformed reading dict: {reading}")
            elif stop.is_set(): break
            else: pass
    except (IMUCommunicationError, IMUTimeoutError, IMUCalibrationError) as e: print(f"[IMU Worker] ERROR: IMU API Error: {e}"); stop.set()
    except FileNotFoundError as e: print(f"[IMU Worker] ERROR: Config file not found: {e}"); stop.set()
    except (KeyError, ValueError) as e: print(f"[IMU Worker] ERROR: Config file invalid: {e}"); stop.set()
    except Exception as e: print(f"[IMU Worker] ERROR: Unexpected error: {e}"); traceback.print_exc(); stop.set()
    finally:
        print("[IMU Worker] Stopping...")
        if imu_api:
            try: imu_api.stop_collection()
            except Exception as e: print(f"[IMU Worker] WARN: Error stopping IMU collection: {e}")
            try: imu_api.close()
            except Exception as e: print(f"[IMU Worker] WARN: Error closing IMU API: {e}")
        print("[IMU Worker] Thread finished.")


# camera_worker now gets session_folder_path from db_manager
def camera_worker(config_path: str, cam_q: queue.Queue, db_manager: DataManager, stop: threading.Event):
    """Camera data collection thread."""
    print("[Camera Worker] Thread started.")
    stereo_system: Optional[StereoCameraSystem] = None
    capture_interval = 0.5
    session_id = None
    session_folder_path = None

    try:
        temp_config = configparser.ConfigParser()
        temp_config.read(config_path)
        capture_interval = temp_config.getfloat('Collection', 'camera_capture_interval_seconds', fallback=0.5)
        print(f"[Camera Worker] Capture interval set to {capture_interval} seconds.")

        stereo_system = StereoCameraSystem(config_path=config_path)
        print("[Camera Worker] Stereo Camera System Initialized.")

        print("[Camera Worker] Waiting for database session to be active...")
        while db_manager.current_session_id is None and not stop.is_set():
            time.sleep(0.2)
        if stop.is_set(): print("[Camera Worker] Stop signaled before DB session active. Exiting."); return

        session_id = db_manager.current_session_id
        session_folder_path = db_manager.current_session_path # Get the actual session path
        if not session_folder_path or not session_id:
             print("[Camera Worker] ERROR: Failed to get valid session info from DB Manager. Stopping.")
             stop.set(); return

        print(f"[Camera Worker] Database session {session_id} detected (Path: {session_folder_path}). Starting capture loop.")

        last_capture_time = time.monotonic()
        while not stop.is_set():
            current_time = time.monotonic()
            if current_time - last_capture_time >= capture_interval:
                last_capture_time = current_time
                # Pass the actual session folder path
                db_info = stereo_system.capture_and_save_pair(session_id=session_id, session_folder_path=session_folder_path)
                if db_info:
                    try: cam_q.put(db_info, timeout=capture_interval * 0.8)
                    except queue.Full: print("[Camera Worker] WARN: Camera queue full. Discarding capture.")
                else: print("[Camera Worker] WARN: Failed to capture/save stereo pair.")
            wait_time = max(0, capture_interval - (time.monotonic() - last_capture_time))
            stop.wait(timeout=wait_time)

    except FileNotFoundError as e: print(f"[Camera Worker] ERROR: Config file not found: {e}"); stop.set()
    except (KeyError, ValueError) as e: print(f"[Camera Worker] ERROR: Config file invalid: {e}"); stop.set()
    except Exception as e: print(f"[Camera Worker] ERROR: Unexpected error: {e}"); traceback.print_exc(); stop.set()
    finally: print("[Camera Worker] Stopping..."); print("[Camera Worker] Thread finished.")


# database_worker remains the same as in main_collector_refactored_v1
def database_worker(config_path: str, imu_q: queue.Queue, cam_q: queue.Queue, lidar_api: Optional[LidarAPI], stop: threading.Event, db_manager: DataManager):
    """Database writing thread."""
    print("[DB Worker] Thread started.")
    lidar_poll_interval = 0.05
    last_processed_lidar_ts = -1
    try:
        temp_config = configparser.ConfigParser(); temp_config.read(config_path)
        lidar_poll_interval = temp_config.getfloat('Collection', 'lidar_poll_interval_seconds', fallback=0.05)
        if db_manager.current_session_id is None:
             print("[DB Worker] ERROR: Database session not started by caller. Stopping."); stop.set(); return
        print(f"[DB Worker] Using database session {db_manager.current_session_id}.")
        while not stop.is_set():
            processed_item = False
            try: # IMU
                while True:
                    (block_pi_ts, sensor_ts, type, x, y, z) = imu_q.get(block=True, timeout=0.01)
                    db_manager.add_imu_measurement(block_pi_ts, sensor_ts, type, x, y, z); processed_item = True
            except queue.Empty: pass
            try: # Camera
                while True:
                    cam_info_dict = cam_q.get(block=True, timeout=0.01)
                    db_manager.add_stereo_image_pair(cam_info_dict); processed_item = True
            except queue.Empty: pass
            if lidar_api: # LiDAR
                latest_lidar_packet = lidar_api.get_latest_packet()
                if latest_lidar_packet:
                    pi_ts_ns, packet_data = latest_lidar_packet
                    if pi_ts_ns > last_processed_lidar_ts:
                        last_processed_lidar_ts = pi_ts_ns
                        lidar_db_packet_info = { 'pi_timestamp': pi_ts_ns, 'sensor_timestamp': packet_data.get('sensor_timestamp'),
                             'speed': packet_data.get('speed'), 'start_angle': packet_data.get('start_angle'), 'end_angle': packet_data.get('end_angle'), }
                        points = packet_data.get('points', [])
                        db_manager.add_lidar_packet_and_points(lidar_db_packet_info, points); processed_item = True
            if not processed_item and not stop.is_set(): time.sleep(lidar_poll_interval)
            if stop.is_set(): break
    except FileNotFoundError as e: print(f"[DB Worker] ERROR: Config file not found: {e}"); stop.set()
    except (KeyError, ValueError) as e: print(f"[DB Worker] ERROR: Config file invalid: {e}"); stop.set()
    except sqlite3.Error as e: print(f"[DB Worker] ERROR: Database error: {e}"); stop.set()
    except Exception as e: print(f"[DB Worker] ERROR: Unexpected error: {e}"); traceback.print_exc(); stop.set()
    finally: print("[DB Worker] Stopping..."); print("[DB Worker] Thread finished.")


class DataCollector:
    """Manages the concurrent data collection process."""
    def __init__(self, config_path: str = 'config.ini'):
        self.config_path = config_path
        self.config = configparser.ConfigParser()
        if not os.path.exists(config_path):
            raise FileNotFoundError(f"Configuration file not found: {config_path}")
        self.config.read(config_path)
        if not self.config.has_section('Collection'):
             raise ValueError(f"Missing [Collection] section in '{config_path}'.")

        self.imu_queue = queue.Queue(maxsize=100)
        self.camera_queue = queue.Queue(maxsize=10)
        self.threads = []
        self.lidar_api_instance: Optional[LidarAPI] = None
        self.db_manager: Optional[DataManager] = None
        self._is_running = False
        self._session_prepared = False

    def prepare_session(self, session_name: str) -> bool:
        """
        Initializes LiDAR (if enabled) and DataManager, starts a new session.
        Must be called before run().

        Args:
            session_name: The base name for the session provided by the user.

        Returns:
            True if session preparation was successful, False otherwise.
        """
        if self._is_running or self._session_prepared:
             print("[DataCollector] WARN: Session already prepared or collection running.")
             return self._session_prepared

        print(f"[DataCollector] Preparing session '{session_name}'...")
        try:
            # --- Initialize LiDAR API ---
            lidar_enabled = self.config.getboolean('LiDAR', 'enabled', fallback=False)
            if lidar_enabled:
                print("[DataCollector] LiDAR enabled. Initializing LidarAPI...")
                try:
                    self.lidar_api_instance = LidarAPI(
                        port=self.config['LiDAR']['port'],
                        baudrate=self.config.getint('LiDAR', 'baud_rate', fallback=230400)
                    )
                    self.lidar_api_instance.connect()
                    print("[DataCollector] LidarAPI connected.")
                    time.sleep(0.5) # Allow connection setup
                except LidarCommunicationError as e:
                     print(f"[DataCollector] WARN: Failed to initialize LiDAR: {e}. Continuing without LiDAR.")
                     self.lidar_api_instance = None
                except (KeyError, ValueError) as e:
                     print(f"[DataCollector] WARN: Invalid LiDAR config: {e}. Continuing without LiDAR.")
                     self.lidar_api_instance = None
            else:
                print("[DataCollector] LiDAR disabled in configuration.")

            # --- Initialize DataManager & Start Session ---
            print("[DataCollector] Initializing DataManager...")
            self.db_manager = DataManager(config_path=self.config_path)
            session_id = self.db_manager.start_new_session(session_name_base=session_name) # Use provided name

            if session_id is None:
                 print("[DataCollector] CRITICAL: Failed to start database session.")
                 # Clean up LiDAR if it was started
                 if self.lidar_api_instance: self.lidar_api_instance.disconnect()
                 self.db_manager = None # Ensure db_manager is None on failure
                 return False

            print(f"[DataCollector] Database session {session_id} started successfully.")
            self._session_prepared = True
            return True

        except Exception as e:
            print(f"[DataCollector] ERROR: Unexpected error during session preparation: {e}")
            traceback.print_exc()
            # Attempt cleanup
            if self.lidar_api_instance:
                 try: self.lidar_api_instance.disconnect()
                 except: pass
            if self.db_manager:
                 try: self.db_manager.close_session()
                 except: pass
            self._session_prepared = False
            return False


    def run(self, stop_event: threading.Event):
        """
        Starts and manages the data collection worker threads. Assumes prepare_session() was called successfully.
        Waits until stop_event is set, then performs cleanup.
        """
        if not self._session_prepared or not self.db_manager:
             print("[DataCollector] ERROR: Session not prepared. Call prepare_session() first.")
             return
        if self._is_running:
             print("[DataCollector] Collection is already running.")
             return

        print("[DataCollector] Starting data collection run...")
        self._is_running = True
        start_time = time.time()
        self.threads = [] # Reset thread list

        try:
            # --- Create and Start Worker Threads ---
            print("[DataCollector] Starting worker threads...")
            # Pass the already initialized db_manager and lidar_api_instance
            db_thread = threading.Thread(target=database_worker,
                                         args=(self.config_path, self.imu_queue, self.camera_queue,
                                               self.lidar_api_instance, stop_event, self.db_manager),
                                         daemon=True, name="DBWorkerThread")
            self.threads.append(db_thread)

            imu_thread = threading.Thread(target=imu_worker,
                                          args=(self.config_path, self.imu_queue, stop_event, self.db_manager),
                                          daemon=True, name="IMUWorkerThread")
            self.threads.append(imu_thread)

            cam_thread = threading.Thread(target=camera_worker,
                                          args=(self.config_path, self.camera_queue, self.db_manager, stop_event),
                                          daemon=True, name="CameraWorkerThread")
            self.threads.append(cam_thread)

            for t in self.threads:
                t.start()

            print("\n--- [DataCollector] Data collection running ---")

            # --- Wait for Stop Signal ---
            while not stop_event.is_set():
                all_alive = True
                for i, t in enumerate(self.threads):
                    if not t.is_alive():
                         print(f"[DataCollector] WARN: Thread '{t.name}' (index {i}) is not alive. Signaling stop.")
                         stop_event.set(); all_alive = False; break
                if not all_alive: break
                stop_event.wait(timeout=0.5)

            print("\n[DataCollector] Stop signal received.")

        except Exception as e:
            print(f"[DataCollector] ERROR: An unexpected error occurred during run: {e}")
            traceback.print_exc(); stop_event.set()
        finally:
            # --- Cleanup ---
            print("[DataCollector] Starting cleanup...")
            print("[DataCollector] Waiting for worker threads to finish...")
            for t in self.threads:
                 if t.is_alive():
                      t.join(timeout=10.0)
                      if t.is_alive(): print(f"[DataCollector] WARN: Thread '{t.name}' did not join cleanly.")

            # --- Final DB Operations ---
            if self.db_manager:
                 print("[DataCollector] Draining final queue items...")
                 final_imu_count = 0; final_cam_count = 0
                 try:
                     while True:
                          (block_pi_ts, sensor_ts, type, x, y, z) = self.imu_queue.get_nowait()
                          self.db_manager.add_imu_measurement(block_pi_ts, sensor_ts, type, x, y, z); final_imu_count += 1
                 except queue.Empty: pass
                 try:
                     while True:
                          cam_info_dict = self.camera_queue.get_nowait()
                          self.db_manager.add_stereo_image_pair(cam_info_dict); final_cam_count += 1
                 except queue.Empty: pass
                 print(f"[DataCollector] Drained {final_imu_count} IMU items, {final_cam_count} camera items.")

                 print("[DataCollector] Ending database session...")
                 self.db_manager.end_current_session() # This closes the connection

            # Disconnect LiDAR
            if self.lidar_api_instance:
                try:
                    print("[DataCollector] Disconnecting LidarAPI...")
                    self.lidar_api_instance.disconnect()
                except Exception as e: print(f"[DataCollector] WARN: Error disconnecting LidarAPI: {e}")

            self._is_running = False
            self._session_prepared = False # Reset prepared state
            end_time = time.time()
            print(f"--- [DataCollector] Data collection run finished --- (Duration: {end_time - start_time:.2f} seconds)")


# --- Standalone Test (Optional) ---
if __name__ == "__main__":
     # Keep the standalone test logic if needed for direct testing
     pass # Remove or comment out if not testing standalone

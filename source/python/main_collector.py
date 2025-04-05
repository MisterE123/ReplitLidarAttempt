import threading
import queue
import time
import configparser
import os
import sys
import traceback
import sqlite3 # Added for specific error handling

# --- Import Custom Modules ---
try:
    from imu_api import IMUAPI, IMUCommunicationError, IMUTimeoutError, IMUCalibrationError
    from lidar_api import LidarAPI, LidarCommunicationError
    from stereo_camera_api import StereoCameraSystem
    from database_manager import DataManager
except ImportError as e:
    print(f"ERROR: Failed to import necessary API modules: {e}")
    print("Ensure imu_api.py, lidar_api.py, stereo_camera_api.py, and database_manager.py are accessible.")
    sys.exit(1)
except FileNotFoundError as e:
     print(f"ERROR: Config file issue during module import: {e}")
     sys.exit(1)
except (KeyError, ValueError) as e:
     print(f"ERROR: Config file value issue during module import: {e}")
     sys.exit(1)

# --- Worker Functions (remain largely unchanged, could be class methods too) ---
# Note: These functions now rely on the DataCollector instance for shared resources if needed,
# but currently primarily use passed arguments (config_path, queues, stop_event, instances).

def imu_worker(config_path: str, imu_q: queue.Queue, stop: threading.Event, db_manager: DataManager):
    """
    Thread function to connect to IMU, read data blocks, parse, and queue measurements.
    Also attempts to update calibration status in the database.
    """
    print("[IMU Worker] Thread started.")
    imu_api: Optional[IMUAPI] = None
    sync_result = None
    try:
        imu_api = IMUAPI(config_path=config_path)
        print("[IMU Worker] IMU API Initialized.")

        print("[IMU Worker] Performing clock sync...")
        sync_result = imu_api.calibrate_clocks()

        # --- Update DB with Sync Status ---
        # Requires db_manager instance
        if db_manager and db_manager.current_session_id is not None:
             lidar_info_str = f"Status: {sync_result.get('status')}, Msg: {sync_result.get('message')}"
             # Consider JSON dumping the full sync_result if needed
             db_manager.update_session_calibration_status(
                  imu_cal_result=sync_result.get('imu_sensor_timestamp_at_cal_us'),
                  lidar_cal_info=lidar_info_str
             )
             print(f"[IMU Worker] Updated DB with clock sync status for session {db_manager.current_session_id}")
        else:
             print("[IMU Worker] WARN: DB Manager not ready, cannot update clock sync status.")
        # ---

        if sync_result['status'] == 'Failed':
             print(f"[IMU Worker] CRITICAL: Clock sync failed: {sync_result['message']}. Stopping.")
             return

        print(f"[IMU Worker] Clock sync status: {sync_result['status']}")

        # --- Start IMU Data Collection ---
        if not imu_api.start_collection():
            print("[IMU Worker] Failed to start IMU data collection. Stopping thread.")
            return
        print("[IMU Worker] IMU data collection started.")

        # --- Main Loop ---
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
                        except queue.Full:
                            print("[IMU Worker] WARN: IMU queue full. Discarding measurement.")
                        except KeyError:
                            print(f"[IMU Worker] WARN: Malformed reading dict: {reading}")
            elif stop.is_set():
                 break
            else:
                pass # Timeout reading block

    except (IMUCommunicationError, IMUTimeoutError, IMUCalibrationError) as e:
        print(f"[IMU Worker] ERROR: IMU API Error: {e}")
        stop.set()
    except FileNotFoundError as e:
        print(f"[IMU Worker] ERROR: Config file not found: {e}")
        stop.set()
    except (KeyError, ValueError) as e:
         print(f"[IMU Worker] ERROR: Config file invalid: {e}")
         stop.set()
    except Exception as e:
        print(f"[IMU Worker] ERROR: Unexpected error: {e}")
        traceback.print_exc()
        stop.set()
    finally:
        print("[IMU Worker] Stopping...")
        if imu_api:
            try: imu_api.stop_collection()
            except Exception as e: print(f"[IMU Worker] WARN: Error stopping IMU collection: {e}")
            try: imu_api.close()
            except Exception as e: print(f"[IMU Worker] WARN: Error closing IMU API: {e}")
        print("[IMU Worker] Thread finished.")


def camera_worker(config_path: str, cam_q: queue.Queue, db_manager: DataManager, stop: threading.Event):
    """
    Thread function to initialize stereo camera, capture pairs at intervals,
    and queue the results (paths, timestamps) for the database worker.
    """
    print("[Camera Worker] Thread started.")
    stereo_system: Optional[StereoCameraSystem] = None
    capture_interval = 0.5

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
        if stop.is_set():
             print("[Camera Worker] Stop signaled before DB session active. Exiting.")
             return
        session_id = db_manager.current_session_id
        print(f"[Camera Worker] Database session {session_id} detected. Starting capture loop.")

        last_capture_time = time.monotonic()
        while not stop.is_set():
            current_time = time.monotonic()
            if current_time - last_capture_time >= capture_interval:
                last_capture_time = current_time
                # print("[Camera Worker] Triggering stereo capture...") # Reduce verbosity
                db_info = stereo_system.capture_and_save_pair(session_id=session_id)

                if db_info:
                    try:
                        cam_q.put(db_info, timeout=capture_interval * 0.8)
                        # print("[Camera Worker] Queued capture info for database.") # Reduce verbosity
                    except queue.Full:
                        print("[Camera Worker] WARN: Camera queue full. Discarding capture.")
                else:
                    print("[Camera Worker] WARN: Failed to capture/save stereo pair.")

            wait_time = max(0, capture_interval - (time.monotonic() - last_capture_time))
            stop.wait(timeout=wait_time)

    except FileNotFoundError as e:
        print(f"[Camera Worker] ERROR: Config file not found: {e}")
        stop.set()
    except (KeyError, ValueError) as e:
         print(f"[Camera Worker] ERROR: Config file invalid: {e}")
         stop.set()
    except Exception as e:
        print(f"[Camera Worker] ERROR: Unexpected error: {e}")
        traceback.print_exc()
        stop.set()
    finally:
        print("[Camera Worker] Stopping...")
        print("[Camera Worker] Thread finished.")


def database_worker(config_path: str, imu_q: queue.Queue, cam_q: queue.Queue, lidar_api: Optional[LidarAPI], stop: threading.Event, db_manager: DataManager):
    """
    Thread function to manage the database session, read from data queues,
    poll LiDAR, and insert records into the database using the provided DataManager instance.
    """
    print("[DB Worker] Thread started.")
    lidar_poll_interval = 0.05
    last_processed_lidar_ts = -1

    try:
        temp_config = configparser.ConfigParser()
        temp_config.read(config_path)
        lidar_poll_interval = temp_config.getfloat('Collection', 'lidar_poll_interval_seconds', fallback=0.05)

        # --- Start Session using the provided DataManager instance ---
        # Session should already be started by the DataCollector.run method
        if db_manager.current_session_id is None:
             print("[DB Worker] ERROR: Database session not started by caller. Stopping.")
             stop.set()
             return

        print(f"[DB Worker] Using database session {db_manager.current_session_id}.")

        # --- Main Loop ---
        while not stop.is_set():
            processed_item = False

            # 1. Process IMU Queue
            try:
                while True:
                    (block_pi_ts, sensor_ts, type, x, y, z) = imu_q.get(block=True, timeout=0.01)
                    db_manager.add_imu_measurement(block_pi_ts, sensor_ts, type, x, y, z)
                    processed_item = True
            except queue.Empty: pass

            # 2. Process Camera Queue
            try:
                while True:
                    cam_info_dict = cam_q.get(block=True, timeout=0.01)
                    db_manager.add_stereo_image_pair(cam_info_dict)
                    processed_item = True
            except queue.Empty: pass

            # 3. Process LiDAR Data
            if lidar_api:
                latest_lidar_packet = lidar_api.get_latest_packet()
                if latest_lidar_packet:
                    pi_ts_ns, packet_data = latest_lidar_packet
                    if pi_ts_ns > last_processed_lidar_ts:
                        last_processed_lidar_ts = pi_ts_ns
                        lidar_db_packet_info = {
                             'pi_timestamp': pi_ts_ns, 'sensor_timestamp': packet_data.get('sensor_timestamp'),
                             'speed': packet_data.get('speed'), 'start_angle': packet_data.get('start_angle'),
                             'end_angle': packet_data.get('end_angle'),
                        }
                        points = packet_data.get('points', [])
                        db_manager.add_lidar_packet_and_points(lidar_db_packet_info, points)
                        processed_item = True

            if not processed_item and not stop.is_set():
                time.sleep(lidar_poll_interval)

            if stop.is_set(): break

    except FileNotFoundError as e:
        print(f"[DB Worker] ERROR: Config file not found: {e}")
        stop.set()
    except (KeyError, ValueError) as e:
         print(f"[DB Worker] ERROR: Config file invalid: {e}")
         stop.set()
    except sqlite3.Error as e:
         print(f"[DB Worker] ERROR: Database error: {e}")
         stop.set()
    except Exception as e:
        print(f"[DB Worker] ERROR: Unexpected error: {e}")
        traceback.print_exc()
        stop.set()
    finally:
        print("[DB Worker] Stopping...")
        # Draining queues and ending session is handled by DataCollector.run
        print("[DB Worker] Thread finished.")


class DataCollector:
    """
    Manages the concurrent data collection process from IMU, LiDAR, and Stereo Camera.
    """
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

    def run(self, stop_event: threading.Event):
        """
        Starts and manages the data collection threads. Waits until stop_event is set.
        Performs cleanup.

        Args:
            stop_event: The threading.Event object used to signal stopping.
        """
        if self._is_running:
             print("[DataCollector] Collection is already running.")
             return

        print("[DataCollector] Starting data collection run...")
        self._is_running = True
        start_time = time.time()
        self.threads = [] # Reset thread list

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
                    time.sleep(0.5)
                except LidarCommunicationError as e:
                     print(f"[DataCollector] WARN: Failed to initialize LiDAR: {e}. Continuing without LiDAR.")
                     self.lidar_api_instance = None
                except (KeyError, ValueError) as e:
                     print(f"[DataCollector] WARN: Invalid LiDAR config: {e}. Continuing without LiDAR.")
                     self.lidar_api_instance = None
            else:
                print("[DataCollector] LiDAR disabled in configuration.")

            # --- Initialize DataManager ---
            print("[DataCollector] Initializing DataManager...")
            self.db_manager = DataManager(config_path=self.config_path)
            session_id = self.db_manager.start_new_session(description="GUI collection session")
            if session_id is None:
                 print("[DataCollector] CRITICAL: Failed to start database session. Aborting run.")
                 stop_event.set() # Ensure stop is signaled if DB fails
                 return # Abort run

            print(f"[DataCollector] Database session {session_id} started.")

            # --- Create and Start Worker Threads ---
            print("[DataCollector] Starting worker threads...")

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
                # Check if any worker thread died unexpectedly
                all_alive = True
                for i, t in enumerate(self.threads):
                    if not t.is_alive():
                         print(f"[DataCollector] WARN: Thread '{t.name}' (index {i}) is not alive. Signaling stop.")
                         stop_event.set()
                         all_alive = False
                         break
                if not all_alive: break
                # Efficiently wait for stop signal without busy-looping
                stop_event.wait(timeout=0.5) # Check every 0.5 seconds

            print("\n[DataCollector] Stop signal received.")

        except Exception as e:
            print(f"[DataCollector] ERROR: An unexpected error occurred during setup or run: {e}")
            traceback.print_exc()
            print("[DataCollector] Signaling stop due to error...")
            stop_event.set()
        finally:
            # --- Cleanup ---
            print("[DataCollector] Starting cleanup...")

            # Wait for threads to finish
            print("[DataCollector] Waiting for worker threads to finish...")
            for t in self.threads:
                 # Check if thread was actually started before joining
                 if t.is_alive():
                      t.join(timeout=10.0) # Wait with timeout
                      if t.is_alive():
                           print(f"[DataCollector] WARN: Thread '{t.name}' did not join cleanly after timeout.")
                 # else: # Useful for debugging if threads fail to start
                 #      print(f"[DataCollector] DEBUG: Thread '{t.name}' was not alive before join.")


            # Drain queues and close DB session (handled by DB worker final block, but call end_session here for safety)
            if self.db_manager:
                 print("[DataCollector] Ensuring database session is ended...")
                 # Drain queues again here? DB worker should handle it.
                 self.db_manager.end_current_session() # This closes the connection

            # Disconnect LiDAR
            if self.lidar_api_instance:
                try:
                    print("[DataCollector] Disconnecting LidarAPI...")
                    self.lidar_api_instance.disconnect()
                except Exception as e:
                    print(f"[DataCollector] WARN: Error disconnecting LidarAPI: {e}")

            self._is_running = False
            end_time = time.time()
            print(f"--- [DataCollector] Data collection run finished --- (Duration: {end_time - start_time:.2f} seconds)")


# --- Standalone Test ---
if __name__ == "__main__":
    print("--- Running DataCollector Standalone Test ---")
    TEST_CONFIG_FILE = 'config.ini'

    # Ensure config exists
    if not os.path.exists(TEST_CONFIG_FILE):
        print(f"ERROR: Configuration file '{TEST_CONFIG_FILE}' not found.")
        sys.exit(1)
    temp_config_check = configparser.ConfigParser()
    temp_config_check.read(TEST_CONFIG_FILE)
    if not temp_config_check.has_section('Collection'):
         print(f"ERROR: Missing [Collection] section in '{TEST_CONFIG_FILE}'.")
         sys.exit(1)

    collector_stop_event = threading.Event()
    try:
        collector = DataCollector(config_path=TEST_CONFIG_FILE)

        # Run collection in a separate thread for standalone test
        collector_thread = threading.Thread(target=collector.run, args=(collector_stop_event,))
        collector_thread.start()

        input("--- Data collection running. Press Enter to stop... --- \n")
        print("[Standalone Test] Enter pressed. Signaling stop...")
        collector_stop_event.set()

        # Wait for collector thread to finish cleanup
        collector_thread.join(timeout=20.0) # Generous timeout for cleanup
        if collector_thread.is_alive():
             print("[Standalone Test] WARN: Collector thread did not finish cleanly.")

    except KeyboardInterrupt:
        print("\n[Standalone Test] KeyboardInterrupt received. Signaling stop...")
        collector_stop_event.set()
        # Allow time for cleanup
        time.sleep(5)
    except Exception as e:
         print(f"[Standalone Test] ERROR: {e}")
         traceback.print_exc()
         collector_stop_event.set()
         time.sleep(5)

    print("--- Standalone Test Finished ---")

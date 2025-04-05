import serial
import time
import configparser
import threading # Keep threading for potential future use if needed
from typing import Tuple, Dict, List, Optional, Any

# Attempt to import the new LidarAPI
try:
    from lidar_api import LidarAPI, LidarCommunicationError
except ImportError:
    print("WARNING: Could not import LidarAPI. LiDAR synchronization features will be unavailable.")
    LidarAPI = None # Define as None if import fails
    LidarCommunicationError = Exception # Define dummy exception

# Define custom exceptions for specific IMU API errors
class IMUCommunicationError(Exception):
    """Custom exception for general serial communication problems."""
    pass

class IMUTimeoutError(IMUCommunicationError):
    """Custom exception for timeouts during communication."""
    pass

class IMUCalibrationError(IMUCommunicationError):
    """Custom exception for calibration failures reported by the IMU."""
    pass


class IMUAPI:
    # Assuming config.ini is in the same directory as this script
    def __init__(self,
                 config_path: str = 'config.ini',
                 serial_timeout: float = 1.0,
                 lidar_instance: Optional[LidarAPI] = None):
        """
        Initialize IMU API: Read config, open serial port, wait for device.

        Args:
            config_path: Path to the configuration file.
            serial_timeout: Default timeout in seconds for serial read operations.
            lidar_instance: An optional pre-initialized instance of LidarAPI.
                            If provided, it will be used for clock sync.
                            If None, LiDAR sync features are disabled.
        """
        print("DEBUG: IMUAPI.__init__ started.")
        config = configparser.ConfigParser()
        print(f"DEBUG: Reading config file: {config_path}")
        read_files = config.read(config_path)
        if not read_files:
             raise FileNotFoundError(f"Configuration file not found or empty: {config_path}")
        print("DEBUG: Config file read.")

        # --- Read IMU Connection Settings ---
        try:
            imu_port = config['IMU']['port']
            imu_baud = int(config['IMU'].get('baud_rate', '115200'))
            print(f"DEBUG: Config IMU - Port={imu_port}, Baud={imu_baud}")
        except KeyError as e:
            raise KeyError(f"Missing IMU key in config file '{config_path}': {e}")
        except ValueError as e:
             raise ValueError(f"Invalid numerical value for IMU in config file '{config_path}': {e}")

        # --- Read Calibration Delay Settings ---
        try:
            self.still_delay = float(config['Calibration'].get('still_delay_seconds', 5.0))
            self.rotation_delay = float(config['Calibration'].get('rotation_delay_seconds', 10.0))
            print(f"DEBUG: Config Calib - Still Delay={self.still_delay}, Rotation Delay={self.rotation_delay}")
        except KeyError as e:
            print(f"IMUAPI: Warning - [Calibration] section or keys missing in config: {e}. Using defaults.")
            self.still_delay = 5.0
            self.rotation_delay = 10.0
        except ValueError as e:
            raise ValueError(f"Invalid numerical value for delays in config file '{config_path}': {e}")

        # --- Store LiDAR instance ---
        self.lidar: Optional[LidarAPI] = lidar_instance
        if self.lidar:
             print("DEBUG: IMUAPI initialized with provided LidarAPI instance.")
        elif LidarAPI is None:
             print("DEBUG: IMUAPI: LidarAPI not imported, LiDAR sync disabled.")
        else:
             print("DEBUG: IMUAPI: No LidarAPI instance provided, LiDAR sync disabled.")


        # --- Establish IMU Serial Connection ---
        self.ser: Optional[serial.Serial] = None # Initialize ser to None
        try:
            print(f"DEBUG: Attempting to open IMU serial port {imu_port} at {imu_baud} baud...")
            self.ser = serial.Serial(imu_port, imu_baud, timeout=serial_timeout)
            print(f"DEBUG: IMU serial port {imu_port} opened successfully.")
            print("DEBUG: Waiting 2 seconds for Arduino (IMU) to initialize...")
            time.sleep(2)
            print("DEBUG: Finished 2-second wait.")
            print("DEBUG: Attempting to flush initial Arduino output...")
            self._flush_initial_output()
            print("DEBUG: Finished flushing initial output.")
            print("IMUAPI: Ready.")

        except serial.SerialException as e:
            print(f"DEBUG: Failed to open IMU serial port {imu_port}: {e}")
            if self.ser and self.ser.is_open: self.ser.close()
            self.ser = None
            raise IMUCommunicationError(f"Failed to open IMU serial port {imu_port}: {e}")
        except Exception as e:
            print(f"DEBUG: Unexpected error during IMU serial setup or flush: {e}")
            if self.ser and self.ser.is_open: self.ser.close()
            self.ser = None
            raise # Re-raise

    def _flush_initial_output(self, timeout_seconds: float = 2.0):
        """Reads and discards any initial output from the Arduino after reset."""
        if not self.ser or not self.ser.is_open:
            print("DEBUG: _flush_initial_output: IMU Serial port not open, skipping flush.")
            return

        print("DEBUG: _flush_initial_output started.")
        start_time = time.time()
        while time.time() - start_time < timeout_seconds:
            try:
                if not self.ser or not self.ser.is_open: break
                line_bytes = self.ser.readline()
                if not line_bytes:
                    time.sleep(0.1) # Give it a moment
                    if not self.ser or not self.ser.is_open: break
                    if not self.ser.readline(): # Check again
                        print("DEBUG: _flush_initial_output timed out. Breaking flush loop.")
                        break
                    else:
                         line = line_bytes.decode(errors='ignore').strip()
                         print(f"DEBUG: Discarding startup line (after 2nd try): {line}")
                else:
                    line = line_bytes.decode(errors='ignore').strip()
                    print(f"DEBUG: Discarding startup line: {line}")
            except serial.SerialTimeoutException:
                print("DEBUG: _flush_initial_output caught SerialTimeoutException. Breaking flush loop.")
                break
            except Exception as e:
                 print(f"DEBUG: Warning - Error flushing initial output: {e}")
                 break
        print("DEBUG: _flush_initial_output finished.")


    def _send_command_and_wait_for_status(self, command: bytes, start_token: str, complete_token: str, fail_token: str, timeout_seconds: float) -> bool:
        """
        Sends a command, waits for start token, prints intermediate lines,
        and waits for a completion or failure token. (Error handling improved)
        """
        if not self.ser or not self.ser.is_open:
             raise IMUCommunicationError("IMU Serial port is not open.")

        print(f"IMUAPI: Sending command: {command.decode().strip()}")
        self.ser.write(command)
        self.ser.flush()

        # --- Wait for START token ---
        start_wait_start_time = time.time()
        start_token_received = False
        print(f"DEBUG: Waiting for start token '{start_token}'...")
        start_timeout = (self.ser.timeout if self.ser.timeout else 0) + 1.0 # Use configured timeout + buffer
        while time.time() - start_wait_start_time < start_timeout:
             try:
                if not self.ser or not self.ser.is_open: raise IMUCommunicationError("IMU Serial port closed unexpectedly.")
                line_bytes = self.ser.readline()
                if not line_bytes: continue # Read timed out, try again

                line = line_bytes.decode(errors='ignore').strip()
                print(f"IMUAPI: Received: {line}")

                if line.startswith(start_token):
                    print(f"DEBUG: Start token '{start_token}' received.")
                    start_token_received = True
                    break
             except serial.SerialException as e:
                 raise IMUCommunicationError(f"Serial error waiting for start token: {e}")
             except Exception as e:
                 raise IMUCommunicationError(f"Error reading start token: {e}")

        if not start_token_received:
             print(f"DEBUG: Timeout waiting for start token '{start_token}'.")
             raise IMUTimeoutError(f"Timeout waiting for start token '{start_token}' after sending command '{command.decode().strip()}'")

        # --- Wait for COMPLETE or FAIL token ---
        wait_start_time = time.time()
        print(f"DEBUG: Waiting for complete ('{complete_token}') or fail ('{fail_token}') token (timeout={timeout_seconds}s)...")
        while time.time() - wait_start_time < timeout_seconds:
            try:
                if not self.ser or not self.ser.is_open: raise IMUCommunicationError("IMU Serial port closed unexpectedly.")
                line_bytes = self.ser.readline()
                if not line_bytes: continue # Read timed out, check overall timeout

                line = line_bytes.decode(errors='ignore').strip()
                print(f"IMUAPI: Received: {line}")

                if line.startswith(complete_token):
                    print(f"DEBUG: Complete token '{complete_token}' received.")
                    return True # Success
                if line.startswith(fail_token):
                    print(f"DEBUG: Fail token '{fail_token}' received.")
                    raise IMUCalibrationError(f"Calibration failed. IMU reported: '{line}'")

            except serial.SerialException as e:
                 raise IMUCommunicationError(f"Serial error waiting for status token: {e}")
            except Exception as e:
                raise IMUCommunicationError(f"Unexpected error while waiting for status: {e}")

        print(f"DEBUG: Timeout waiting for complete/fail token.")
        raise IMUTimeoutError(f"Timeout ({timeout_seconds}s) waiting for '{complete_token}' or '{fail_token}' after receiving '{start_token}'")


    def calibrate_time(self) -> int:
        """
        Performs ONLY the IMU time calibration. Sends 'time_calibrate' command.
        Returns the IMU's internal timestamp (microseconds) at the moment of calibration.
        Use `calibrate_clocks` for combined IMU and LiDAR synchronization.
        """
        if not self.ser or not self.ser.is_open:
             raise IMUCommunicationError("IMU Serial port is not open.")

        command = b'time_calibrate\n'
        print(f"IMUAPI: Sending command: {command.decode().strip()}")
        self.ser.write(command)
        self.ser.flush()

        print("DEBUG: Waiting for TIME_ECHO response...")
        try:
            read_timeout = (self.ser.timeout if self.ser.timeout else 0) + 1.0
            start_wait = time.time()
            response_bytes = b''
            while time.time() - start_wait < read_timeout:
                 if not self.ser or not self.ser.is_open: raise IMUCommunicationError("IMU Serial port closed unexpectedly.")
                 response_bytes = self.ser.readline()
                 if response_bytes: break
            else: # Loop finished without break -> timeout
                 print("DEBUG: Timeout waiting for TIME_ECHO.")
                 raise IMUTimeoutError("Timeout waiting for TIME_ECHO response.")

            response = response_bytes.decode(errors='ignore').strip()
            print(f"IMUAPI: Received: {response}")

            if response.startswith('TIME_ECHO'):
                try:
                    imu_sensor_timestamp_us = int(response.split(',')[1])
                    print(f"IMUAPI: Time calibrated. IMU Epoch (us): {imu_sensor_timestamp_us}")
                    return imu_sensor_timestamp_us
                except (IndexError, ValueError):
                     print(f"DEBUG: Failed to parse TIME_ECHO: {response}")
                     raise IMUCommunicationError(f"Failed to parse timestamp from TIME_ECHO response: {response}")
            else:
                print(f"DEBUG: Unexpected response to time_calibrate: {response}. Reading extra lines...")
                extra_line = self.ser.read_until(b'\n', timeout=0.5).decode(errors='ignore').strip()
                print(f"IMUAPI: Received extra line after time_calibrate: {extra_line}")
                raise IMUCommunicationError(f"Time calibration failed. Expected 'TIME_ECHO', got '{response}'")

        except serial.SerialException as e:
             raise IMUCommunicationError(f"Serial error during time calibration: {e}")
        except Exception as e:
             print(f"DEBUG: Unexpected error during time calibration: {e}")
             # Ensure specific exceptions are raised where possible
             if isinstance(e, (IMUCommunicationError, IMUTimeoutError)):
                  raise e
             else:
                  raise IMUCommunicationError(f"Error during time calibration: {e}")


    def calibrate_clocks(self, lidar_sync_duration_s: float = 0.2) -> Dict[str, Any]:
        """
        Performs IMU time calibration and attempts to synchronize with LiDAR clock.

        This method sends the 'time_calibrate' command to the IMU and, while
        waiting for the response, captures the latest available LiDAR packet's
        timestamps.

        Args:
            lidar_sync_duration_s: Approximate duration (in seconds) around the IMU
                                   calibration command/response to look for LiDAR packets.

        Returns:
            A dictionary containing the synchronization results:
            {
                'pi_sys_time_for_imu_cal_ns': int, # Approx Pi time (ns) of IMU cal
                'imu_sensor_timestamp_at_cal_us': int, # IMU sensor time (us) at cal
                'pi_sys_time_anchor_lidar_ns': Optional[int], # Pi time (ns) of sync LiDAR packet
                'lidar_sensor_timestamp_anchor_s': Optional[float] # LiDAR sensor time (s) of sync packet
                'status': str # 'Success' or 'Partial (No LiDAR Sync)' or 'Failed'
                'message': str # Descriptive message
            }
            Returns None for LiDAR fields if LiDAR API is unavailable or no packet
            was captured during the sync window.

        Raises:
            IMUCommunicationError, IMUTimeoutError: If IMU communication fails.
            LidarCommunicationError: If the provided LiDAR instance is not connected.
        """
        if not self.ser or not self.ser.is_open:
            raise IMUCommunicationError("IMU Serial port is not open.")

        # Check LiDAR status if available
        if self.lidar and not self.lidar._is_connected:
             # We could try connecting here, but safer to require it pre-connected
             raise LidarCommunicationError("Provided LidarAPI instance is not connected.")

        command = b'time_calibrate\n'
        imu_response = None
        imu_sensor_timestamp_us = None
        lidar_packets_in_window: List[Tuple[int, Dict[str, Any]]] = []

        print("IMUAPI: Starting clock calibration (IMU + LiDAR)...")

        # --- Timing and Data Capture ---
        pi_time_start_ns = time.time_ns()
        pi_time_imu_response_ns = None
        pi_time_end_ns = None

        try:
            # Send command to IMU
            print(f"IMUAPI: Sending command: {command.decode().strip()}")
            self.ser.write(command)
            self.ser.flush()

            # --- Wait for IMU response while capturing LiDAR data ---
            print("DEBUG: Waiting for TIME_ECHO response from IMU...")
            read_timeout = (self.ser.timeout if self.ser.timeout else 0) + 1.5 # Slightly longer timeout
            wait_start_time = time.time()
            response_received = False

            while time.time() - wait_start_time < read_timeout:
                # 1. Check for IMU response
                if not response_received and self.ser.in_waiting > 0:
                    response_bytes = self.ser.readline()
                    pi_time_imu_response_ns = time.time_ns() # Capture time *after* read completes
                    if response_bytes:
                        imu_response = response_bytes.decode(errors='ignore').strip()
                        print(f"IMUAPI: Received IMU response: {imu_response}")
                        response_received = True
                        # Don't break yet, continue capturing LiDAR for a short window

                # 2. Capture latest LiDAR packet (if LiDAR is available)
                if self.lidar:
                    latest_lidar = self.lidar.get_latest_packet()
                    if latest_lidar:
                         # Check if this packet's Pi timestamp is newer than the last captured one
                         # Store if it falls within a reasonable window around the current time
                         current_pi_time = time.time()
                         lidar_pi_time_ns, _ = latest_lidar
                         lidar_pi_time_s = lidar_pi_time_ns / 1e9
                         # Only add if it's recent and not already added
                         # Check if pi_time_start_ns exists
                         time_lower_bound = (pi_time_start_ns / 1e9) - 0.1 # A bit before start
                         time_upper_bound = current_pi_time + lidar_sync_duration_s # Allow some time after loop might finish

                         is_new = True
                         if lidar_packets_in_window:
                              last_ts, _ = lidar_packets_in_window[-1]
                              if lidar_pi_time_ns == last_ts:
                                   is_new = False

                         if is_new and (time_lower_bound <= lidar_pi_time_s <= time_upper_bound):
                              lidar_packets_in_window.append(latest_lidar)
                              # Keep only the last few packets in the window to avoid memory issues
                              lidar_packets_in_window = lidar_packets_in_window[-10:] # Keep last 10

                # 3. Check if IMU response received and exit condition met
                if response_received:
                     # Optional: Wait a tiny bit longer after response to capture trailing LiDAR packets
                     if time.time() - (pi_time_imu_response_ns / 1e9) > (lidar_sync_duration_s / 2.0):
                          break # Exit loop after response + half sync duration

                # 4. Small sleep if nothing happened
                if not self.ser.in_waiting and not response_received:
                    time.sleep(0.002) # Short sleep to yield CPU

            pi_time_end_ns = time.time_ns() # Record end time of the whole process

            # --- Process Results ---
            if not response_received:
                print("DEBUG: Timeout waiting for TIME_ECHO from IMU.")
                raise IMUTimeoutError("Timeout waiting for TIME_ECHO response during clock calibration.")

            # Parse IMU response
            if imu_response and imu_response.startswith('TIME_ECHO'):
                try:
                    imu_sensor_timestamp_us = int(imu_response.split(',')[1])
                    print(f"IMUAPI: IMU Time calibrated. IMU Epoch (us): {imu_sensor_timestamp_us}")
                except (IndexError, ValueError):
                     print(f"DEBUG: Failed to parse TIME_ECHO: {imu_response}")
                     raise IMUCommunicationError(f"Failed to parse timestamp from TIME_ECHO response: {imu_response}")
            else:
                print(f"DEBUG: Unexpected IMU response: {imu_response}")
                raise IMUCommunicationError(f"Clock calibration failed. Expected 'TIME_ECHO', got '{imu_response}'")

            # Approximate Pi time for IMU calibration event
            # Use the time the response was received, or midpoint if preferred
            pi_sys_time_for_imu_cal_ns = pi_time_imu_response_ns if pi_time_imu_response_ns else (pi_time_start_ns + pi_time_end_ns) // 2

            # Find the best LiDAR anchor packet
            pi_sys_time_anchor_lidar_ns = None
            lidar_sensor_timestamp_anchor_s = None
            if self.lidar and lidar_packets_in_window:
                best_packet = min(lidar_packets_in_window,
                                  key=lambda p: abs(p[0] - pi_sys_time_for_imu_cal_ns))
                pi_sys_time_anchor_lidar_ns = best_packet[0]
                lidar_sensor_timestamp_anchor_s = best_packet[1].get('sensor_timestamp')
                print(f"DEBUG: Found LiDAR anchor packet. PiTime(ns): {pi_sys_time_anchor_lidar_ns}, SensorTime(s): {lidar_sensor_timestamp_anchor_s}")

            # --- Construct result dictionary ---
            status = 'Success' if lidar_sensor_timestamp_anchor_s is not None else 'Partial (No LiDAR Sync)'
            message = "IMU and LiDAR clocks synchronized." if status == 'Success' else "IMU clock calibrated, but no suitable LiDAR packet found for sync."
            if not self.lidar:
                 status = 'Partial (No LiDAR API)'
                 message = "IMU clock calibrated. LiDAR API not available for sync."


            result = {
                'pi_sys_time_for_imu_cal_ns': pi_sys_time_for_imu_cal_ns,
                'imu_sensor_timestamp_at_cal_us': imu_sensor_timestamp_us,
                'pi_sys_time_anchor_lidar_ns': pi_sys_time_anchor_lidar_ns,
                'lidar_sensor_timestamp_anchor_s': lidar_sensor_timestamp_anchor_s,
                'status': status,
                'message': message
            }
            print(f"IMUAPI: Clock calibration result: {result}")
            return result

        except (serial.SerialException, IMUCommunicationError, IMUTimeoutError, LidarCommunicationError) as e:
             # Catch specific errors from IMU or LiDAR comms
             print(f"ERROR: Clock calibration failed: {e}")
             return {
                'pi_sys_time_for_imu_cal_ns': None,
                'imu_sensor_timestamp_at_cal_us': None,
                'pi_sys_time_anchor_lidar_ns': None,
                'lidar_sensor_timestamp_anchor_s': None,
                'status': 'Failed',
                'message': f"Calibration failed due to communication error: {e}"
            }
        except Exception as e:
            # Catch any other unexpected errors
            print(f"ERROR: Unexpected error during clock calibration: {e}")
            return {
                'pi_sys_time_for_imu_cal_ns': None,
                'imu_sensor_timestamp_at_cal_us': None,
                'pi_sys_time_anchor_lidar_ns': None,
                'lidar_sensor_timestamp_anchor_s': None,
                'status': 'Failed',
                'message': f"Unexpected error during calibration: {e}"
            }


    # --- Other Calibration Methods (Unchanged from original, just added debug/error details) ---

    def calibrate_gravity(self, timeout_seconds: float = 7.0) -> None:
        """Calibrate gravity compensation. Sends 'gravity_calibrate'."""
        print("DEBUG: Starting gravity calibration...")
        self._send_command_and_wait_for_status(
            command=b'gravity_calibrate\n',
            start_token='GRAVITY_CAL_START',
            complete_token='GRAVITY_CAL_COMPLETE',
            fail_token='GRAVITY_CAL_FAILED',
            timeout_seconds=timeout_seconds
        )
        print("IMUAPI: Gravity calibration successful.")

    def calibrate_gyro(self, timeout_seconds: float = 7.0) -> None:
        """Calibrate gyroscope bias. Sends 'gyro_calibrate'."""
        print("DEBUG: Starting gyro calibration...")
        self._send_command_and_wait_for_status(
            command=b'gyro_calibrate\n',
            start_token='GYRO_CAL_START',
            complete_token='GYRO_CAL_COMPLETE',
            fail_token='GYRO_CAL_FAILED',
            timeout_seconds=timeout_seconds
        )
        print("IMUAPI: Gyro calibration successful.")

    def calibrate_mag(self, timeout_seconds: float = 15.0) -> None:
        """Calibrate magnetometer. Sends 'mag_calibrate'."""
        print("DEBUG: Starting magnetometer calibration...")
        if timeout_seconds <= 10.0:
             print("IMUAPI: Warning - Magnetometer calibration timeout should ideally be > 10 seconds.")
        self._send_command_and_wait_for_status(
            command=b'mag_calibrate\n',
            start_token='MAG_CAL_START',
            complete_token='MAG_CAL_COMPLETE',
            fail_token='MAG_CAL_FAILED',
            timeout_seconds=timeout_seconds
        )
        print("IMUAPI: Magnetometer calibration successful.")

    # --- Data Collection Methods (Unchanged) ---

    def start_collection(self) -> bool:
        """Sends 'start_collection' command."""
        # ... (Keep original implementation)
        if not self.ser or not self.ser.is_open: raise IMUCommunicationError("IMU Serial port is not open.")
        command = b'start_collection\n'; print(f"IMUAPI: Sending command: {command.decode().strip()}"); self.ser.write(command); self.ser.flush()
        print("DEBUG: Waiting for start_collection response..."); start_time = time.time(); response_found = False; success = False; read_timeout = (self.ser.timeout if self.ser.timeout else 0) + 1.0
        while time.time() - start_time < read_timeout:
            try:
                if not self.ser or not self.ser.is_open: raise IMUCommunicationError("IMU Serial port closed unexpectedly.")
                line_bytes = self.ser.readline();
                if not line_bytes: continue
                line = line_bytes.decode(errors='ignore').strip(); print(f"IMUAPI: Received: {line}")
                if line.startswith("STARTING_COLLECTION"): print("DEBUG: 'STARTING_COLLECTION' received."); response_found = True; success = True; break
                elif line.startswith("ERROR: Time must be calibrated first"): print(f"IMUAPI: Error from IMU: {line}"); response_found = True; success = False; break
                elif line.startswith("WARNING: Already collecting"): print(f"IMUAPI: Warning from IMU: {line}"); response_found = True; success = False; break
            except serial.SerialException as e: raise IMUCommunicationError(f"Serial error reading start_collection response: {e}")
            except Exception as e: raise IMUCommunicationError(f"Error reading start_collection response: {e}")
        if not response_found: print("DEBUG: Timeout waiting for start_collection response."); raise IMUTimeoutError("Timeout waiting for response after start_collection command.")
        return success


    def stop_collection(self) -> bool:
        """Sends 'stop_collection' command."""
        # ... (Keep original implementation)
        if not self.ser or not self.ser.is_open: raise IMUCommunicationError("IMU Serial port is not open.")
        command = b'stop_collection\n'; print(f"IMUAPI: Sending command: {command.decode().strip()}"); self.ser.write(command); self.ser.flush()
        print("DEBUG: Waiting for stop_collection response..."); start_time = time.time(); response_found = False; success = False; read_timeout = (self.ser.timeout if self.ser.timeout else 0) + 1.0
        while time.time() - start_time < read_timeout:
            try:
                if not self.ser or not self.ser.is_open: raise IMUCommunicationError("IMU Serial port closed unexpectedly.")
                line_bytes = self.ser.readline()
                if not line_bytes: continue
                line = line_bytes.decode(errors='ignore').strip(); print(f"IMUAPI: Received: {line}")
                if line.startswith("STOPPING_COLLECTION"): print("DEBUG: 'STOPPING_COLLECTION' received."); response_found = True; success = True # Don't break immediately
                elif line.startswith("INFO: Not currently collecting data"): print("DEBUG: 'INFO: Not currently collecting data' received."); response_found = True; success = True # Don't break immediately
                elif line.startswith("READY"):
                     if not response_found: print("IMUAPI: Received READY, assuming stop was successful or already stopped."); response_found = True; success = True
                     break # READY signifies end
            except serial.SerialException as e: raise IMUCommunicationError(f"Serial error reading stop_collection response: {e}")
            except Exception as e: raise IMUCommunicationError(f"Error reading stop_collection response: {e}")
        if not response_found: print("DEBUG: Timeout waiting for stop_collection response."); raise IMUTimeoutError("Timeout waiting for response after stop_collection command.")
        # self._flush_initial_output(timeout_seconds=0.5) # Optional flush after stop
        return success


    def read_data_block(self, read_timeout_seconds: float = 2.0) -> Optional[Dict[str, List[Dict]]]:
        """Reads one complete IMU data block."""
        # ... (Keep original implementation)
        if not self.ser or not self.ser.is_open: raise IMUCommunicationError("IMU Serial port is not open.")
        data: Dict[str, List[Dict]] = {'M': [], 'A': [], 'E': [], 'G': []}; expected_counts = {'M': 0, 'A': 0, 'E': 0, 'G': 0}; received_counts = {'M': 0, 'A': 0, 'E': 0, 'G': 0}
        in_block = False; block_found = False; start_time = time.time()
        while time.time() - start_time < read_timeout_seconds:
            try:
                if not self.ser or not self.ser.is_open: raise IMUCommunicationError("IMU Serial port closed unexpectedly.")
                line_bytes = self.ser.readline()
                if not line_bytes:
                    if in_block: print(f"DEBUG: Timeout occurred *inside* IMU data block after {time.time() - start_time:.2f}s."); raise IMUTimeoutError(f"Timeout occurred *inside* IMU data block after {time.time() - start_time:.2f}s.")
                    else:
                         if time.time() - start_time >= read_timeout_seconds: return None
                         continue
                line = line_bytes.decode(errors='ignore').strip()
                if not in_block:
                    if line.startswith("BEGIN_DATA_BLOCK"):
                        print(f"DEBUG: Received BEGIN_DATA_BLOCK: {line}"); parts = line.split(',')
                        if len(parts) != 5: print(f"IMUAPI: Warning - Malformed BEGIN_DATA_BLOCK: {line}"); continue
                        try:
                            expected_counts['M'] = int(parts[1]); expected_counts['A'] = int(parts[2]); expected_counts['E'] = int(parts[3]); expected_counts['G'] = int(parts[4])
                            in_block = True; block_found = True; data = {'M': [], 'A': [], 'E': [], 'G': []}; received_counts = {'M': 0, 'A': 0, 'E': 0, 'G': 0}
                        except ValueError: print(f"IMUAPI: Warning - Could not parse counts in BEGIN_DATA_BLOCK: {line}")
                else:
                    if line.startswith("END_DATA_BLOCK"):
                        print(f"DEBUG: Received END_DATA_BLOCK. Got M:{received_counts['M']}, A:{received_counts['A']}, E:{received_counts['E']}, G:{received_counts['G']}")
                        in_block = False
                        if (received_counts['M'] == expected_counts['M'] and received_counts['A'] == expected_counts['A'] and received_counts['E'] == expected_counts['E'] and received_counts['G'] == expected_counts['G']): return data
                        else: print(f"IMUAPI: Warning - Mismatched counts at END_DATA_BLOCK. Got M:{received_counts['M']}/{expected_counts['M']}, A:{received_counts['A']}/{expected_counts['A']}, E:{received_counts['E']}/{expected_counts['E']}, G:{received_counts['G']}/{expected_counts['G']}"); return data
                    elif len(line) > 2 and line[1] == ',':
                        sensor_type = line[0]
                        if sensor_type in data:
                            parts = line.split(',')
                            if len(parts) == 5:
                                try:
                                    reading = {'timestamp': int(parts[1]), 'x': float(parts[2]), 'y': float(parts[3]), 'z': float(parts[4])}
                                    if received_counts[sensor_type] < expected_counts[sensor_type]: data[sensor_type].append(reading); received_counts[sensor_type] += 1
                                    else: print(f"IMUAPI: Warning - Received more {sensor_type} readings than expected ({expected_counts[sensor_type]}) in block. Ignoring extra: {line}")
                                except (ValueError, IndexError): print(f"IMUAPI: Warning - Could not parse sensor data line: {line}")
                            else: print(f"IMUAPI: Warning - Malformed sensor data line: {line}")
                    else: print(f"IMUAPI: Warning - Unexpected line inside data block: {line}")
            except serial.SerialException as e: raise IMUCommunicationError(f"Serial error during read_data_block: {e}")
            except Exception as e: print(f"IMUAPI: Error during read_data_block: {e}"); raise IMUCommunicationError(f"Unexpected error processing line: {line if 'line' in locals() else 'N/A'}: {e}")
        if in_block: print("IMUAPI: Warning - Timeout occurred after block started but before END_DATA_BLOCK received."); return data
        elif not block_found: return None
        else: print("IMUAPI: Warning - Read loop exited unexpectedly after block completion."); return None # Should not happen


    def close(self):
        """Closes the IMU serial port if it's open."""
        print("DEBUG: IMUAPI.close() called.")
        # Note: Does NOT close the LiDAR instance, as it might be shared.
        # The owner of the LiDAR instance is responsible for closing it.
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
                print(f"IMUAPI: IMU Serial port {self.ser.port} closed.")
            except Exception as e:
                 print(f"IMUAPI: Error closing IMU serial port: {e}")
        else:
            print("DEBUG: IMU Serial port was already closed or not initialized.")
        self.ser = None


# --- Example Usage Block (Updated) ---
if __name__ == '__main__':
    # --- !!! CHANGE THESE TO YOUR DEVICE PORTS AND BAUD RATES !!! ---
    IMU_PORT = '/dev/ttyACM0' # Example IMU port
    IMU_BAUD = 115200
    LIDAR_PORT = '/dev/ttyUSB0' # Example LiDAR port
    LIDAR_BAUD = 230400

    # Create dummy config.ini if needed
    CONFIG_FILE = 'config.ini'
    try:
        with open(CONFIG_FILE, 'r') as f: pass
        print(f"Using existing config file: {CONFIG_FILE}")
        # Ensure it has the correct ports for the example
        # You might want to update the config file manually or programmatically here
    except FileNotFoundError:
        print(f"Creating dummy config file: {CONFIG_FILE}")
        with open(CONFIG_FILE, 'w') as f:
            f.write("[IMU]\n")
            f.write(f"port = {IMU_PORT}\n") # Use variable
            f.write(f"baud_rate = {IMU_BAUD}\n")
            f.write("\n[Calibration]\n")
            f.write("still_delay_seconds = 5\n")
            f.write("rotation_delay_seconds = 10\n")

    # Initialize APIs
    lidar_instance: Optional[LidarAPI] = None
    imu_instance: Optional[IMUAPI] = None

    try:
        # 1. Initialize LiDAR (if available)
        if LidarAPI:
             print("\nInitializing LiDAR API...")
             try:
                  lidar_instance = LidarAPI(port=LIDAR_PORT, baudrate=LIDAR_BAUD)
                  lidar_instance.connect()
                  print("LiDAR API Connected.")
                  # Give LiDAR time to start streaming packets
                  time.sleep(1.0)
             except LidarCommunicationError as e:
                  print(f"WARNING: Failed to initialize LiDAR: {e}. Proceeding without LiDAR sync.")
                  lidar_instance = None # Ensure it's None if connection failed
        else:
             print("\nLidarAPI not available. Skipping LiDAR initialization.")

        # 2. Initialize IMU (passing LiDAR instance if available)
        print("\nInitializing IMU API...")
        imu_instance = IMUAPI(config_path=CONFIG_FILE, serial_timeout=2.0, lidar_instance=lidar_instance)
        print("IMU API Initialized.")

        # --- Clock Synchronization ---
        print("\nAttempting Clock Synchronization...")
        sync_result = imu_instance.calibrate_clocks(lidar_sync_duration_s=0.3)
        print("\n--- Clock Sync Result ---")
        print(f"Status: {sync_result.get('status')}")
        print(f"Message: {sync_result.get('message')}")
        print(f"IMU Cal Pi Time (ns):   {sync_result.get('pi_sys_time_for_imu_cal_ns')}")
        print(f"IMU Sensor Time (us): {sync_result.get('imu_sensor_timestamp_at_cal_us')}")
        print(f"LiDAR Anchor Pi Time (ns): {sync_result.get('pi_sys_time_anchor_lidar_ns')}")
        print(f"LiDAR Sensor Time (s):   {sync_result.get('lidar_sensor_timestamp_anchor_s')}")
        print("------------------------")

        # Check if IMU time calibration part was successful
        time_calibrated = sync_result.get('imu_sensor_timestamp_at_cal_us') is not None

        # --- Sensor Calibration Sequence (Only if time calibration worked) ---
        if time_calibrated:
            print("\nStarting Sensor Calibration Example...")
            try:
                 # Gravity
                 input("Prepare for Gravity Calibration. Keep IMU still, then press Enter...")
                 print("Attempting Gravity Calibration..."); imu_instance.calibrate_gravity(); print("Gravity Cal Complete.")
                 # Gyro
                 input("Prepare for Gyro Calibration. Keep IMU still, then press Enter...")
                 print("Attempting Gyro Calibration..."); imu_instance.calibrate_gyro(); print("Gyro Cal Complete.")
                 # Mag
                 input("Prepare for Magnetometer Calibration. Press Enter, then rotate slowly for ~10s...")
                 print("Attempting Magnetometer Calibration..."); imu_instance.calibrate_mag(); print("Mag Cal Complete.")
                 print("\n--- SENSOR CALIBRATIONS COMPLETE ---")
            except (IMUCommunicationError, IMUCalibrationError, IMUTimeoutError) as e:
                print(f"\n--- SENSOR CALIBRATION FAILED --- \nError: {e}")
            except Exception as e:
                 print(f"\n--- UNEXPECTED ERROR DURING SENSOR CALIBRATION --- \nError: {e}")
        else:
             print("\nSkipping sensor calibration because clock calibration failed.")


        # --- Data Collection Example (Only if time calibration worked) ---
        if time_calibrated:
            print("\nStarting Data Collection Example...")
            try:
                if imu_instance.start_collection():
                    print("IMU Collection started.")
                    collection_start_time = time.time()
                    num_blocks_received = 0
                    max_blocks = 3 # Collect fewer blocks for example
                    print(f"Attempting to read {max_blocks} IMU data blocks...")

                    while num_blocks_received < max_blocks:
                         print(f"Waiting for IMU block {num_blocks_received + 1}/{max_blocks}...")
                         data_block = imu_instance.read_data_block(read_timeout_seconds=3.0)
                         if data_block:
                             num_blocks_received += 1
                             print(f"--- IMU Block {num_blocks_received} Received ---")
                             print(f"  M:{len(data_block['M'])} A:{len(data_block['A'])} E:{len(data_block['E'])} G:{len(data_block['G'])}")
                             if data_block['A']: print(f"  First Accel Timestamp (us): {data_block['A'][0]['timestamp']}")
                         else:
                             print("No IMU data block received within timeout.")
                             if time.time() - collection_start_time > 15: print("Overall collection time limit reached."); break
                    print("\nStopping IMU data collection..."); imu_instance.stop_collection(); print("IMU Collection stopped.")
                else: print("Failed to start IMU collection.")
            except (IMUCommunicationError, IMUTimeoutError) as e:
                 print(f"\n--- IMU DATA COLLECTION FAILED --- \nError: {e}")
                 try:
                     if imu_instance: print("Attempting to stop collection after error..."); imu_instance.stop_collection()
                 except Exception as stop_e: print(f"Could not stop collection after error: {stop_e}")
            except Exception as e:
                 print(f"\n--- UNEXPECTED ERROR DURING IMU COLLECTION --- \nError: {e}")
                 try:
                     if imu_instance: print("Attempting to stop collection after error..."); imu_instance.stop_collection()
                 except Exception as stop_e: print(f"Could not stop collection after error: {stop_e}")
        else:
             print("\nSkipping data collection because clock calibration failed.")

    except (FileNotFoundError, KeyError, ValueError, IMUCommunicationError, LidarCommunicationError) as e:
        print(f"\n--- FAILED TO INITIALIZE APIs ---")
        print(f"Error: {e}")
    except KeyboardInterrupt:
        print("\nKeyboard Interrupt detected. Exiting.")
    except Exception as e:
        print(f"\n--- UNEXPECTED ERROR DURING SETUP OR EXECUTION ---")
        print(f"Error: {e}")
        import traceback
        traceback.print_exc() # Print detailed traceback for unexpected errors

    finally:
        # Ensure resources are released
        print("\nClosing resources...")
        if imu_instance:
             print("Closing IMU API...")
             imu_instance.close()
        if lidar_instance:
             print("Closing LiDAR API...")
             lidar_instance.disconnect() # Use disconnect for LiDAR API
        print("\nIMU/LiDAR API Example Finished.")

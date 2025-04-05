import serial
import time
import configparser
from typing import Tuple, Dict, List, Optional # Added Dict, List, Optional

# Define custom exceptions
class IMUCommunicationError(Exception):
    """Custom exception for serial communication problems."""
    pass

class IMUTimeoutError(IMUCommunicationError):
    """Custom exception for timeouts during communication."""
    pass

class IMUCalibrationError(IMUCommunicationError):
    """Custom exception for calibration failures reported by the IMU."""
    pass


class IMUAPI:
    def __init__(self, config_path: str = 'python/config.ini', serial_timeout: float = 1.0):
        """
        Initialize IMU API with config file.

        Args:
            config_path: Path to the configuration file.
            serial_timeout: Default timeout in seconds for serial read operations.
                           Calibration methods might use longer internal timeouts.
        """
        config = configparser.ConfigParser()
        read_files = config.read(config_path)
        if not read_files:
             raise FileNotFoundError(f"Configuration file not found or empty: {config_path}")

        try:
            port = config['IMU']['port']
            baud = int(config['IMU'].get('baud_rate', '115200'))
        except KeyError as e:
            raise KeyError(f"Missing key in config file '{config_path}': {e}")
        except ValueError as e:
             raise ValueError(f"Invalid numerical value in config file '{config_path}': {e}")

        try:
            # Load delays from config and store them
            self.still_delay = float(config['Calibration'].get('still_delay_seconds', 5.0))
            self.rotation_delay = float(config['Calibration'].get('rotation_delay_seconds', 10.0)) # Use get for defaults
        except KeyError as e:
            # Handle missing [Calibration] section or keys if desired, or let it raise
            print(f"IMUAPI: Warning - [Calibration] section or keys missing in config: {e}. Using defaults.")
            self.still_delay = 5.0
            self.rotation_delay = 10.0
        except ValueError as e:
            raise ValueError(f"Invalid numerical value for delays in config file '{config_path}': {e}")

        # Remove or update the comment that previously said these delays were not used


        try:
            self.ser = serial.Serial(port, baud, timeout=serial_timeout)
            print(f"IMUAPI: Serial port {port} opened at {baud} baud.")
            print("IMUAPI: Waiting for Arduino to initialize...")
            time.sleep(2)  # Wait for Arduino reset
            # Read any startup messages from Arduino (like the READY message)
            self._flush_initial_output()
            print("IMUAPI: Ready.")
        except serial.SerialException as e:
            raise IMUCommunicationError(f"Failed to open serial port {port}: {e}")

    def _flush_initial_output(self, timeout_seconds: float = 2.0):
        """Reads and discards any initial output from the Arduino after reset."""
        start_time = time.time()
        while time.time() - start_time < timeout_seconds:
            try:
                line = self.ser.readline().decode(errors='ignore').strip()
                if line:
                    print(f"IMUAPI: Discarding startup line: {line}")
                else: # If readline times out (returns empty)
                    # Give it one more short try in case buffer wasn't full
                    time.sleep(0.1)
                    if not self.ser.readline():
                        break # Assume no more startup messages
            except serial.SerialTimeoutException:
                break # Port timeout occurred, assume no more messages
            except Exception as e:
                 print(f"IMUAPI: Warning - Error flushing initial output: {e}")
                 break # Stop flushing on other errors


    def _send_command_and_wait_for_status(self, command: bytes, start_token: str, complete_token: str, fail_token: str, timeout_seconds: float) -> bool:
        """
        Sends a command, waits for start token, prints intermediate lines,
        and waits for a completion or failure token.

        Args:
            command: The command bytes to send (e.g., b'gravity_calibrate\n').
            start_token: The expected first line indicating the command started.
            complete_token: The line indicating successful completion.
            fail_token: The line indicating failure.
            timeout_seconds: Total time to wait for the complete/fail token after start.

        Returns:
            True if complete_token is received.

        Raises:
            IMUTimeoutError: If timeout occurs at any stage.
            IMUCalibrationError: If fail_token is received.
            IMUCommunicationError: For other serial or decoding issues.
        """
        if not self.ser.is_open:
             raise IMUCommunicationError("Serial port is not open.")

        print(f"IMUAPI: Sending command: {command.decode().strip()}")
        self.ser.write(command)
        self.ser.flush() # Ensure command is sent

        # --- Wait for START token ---
        start_wait_start_time = time.time()
        start_token_received = False
        while time.time() - start_wait_start_time < self.ser.timeout + 1: # Wait slightly longer than default timeout
             try:
                line_bytes = self.ser.readline()
                if not line_bytes: continue # Read timed out, try again within window

                line = line_bytes.decode().strip()
                print(f"IMUAPI: Received: {line}") # Log received line

                if line.startswith(start_token):
                    start_token_received = True
                    break # Start token found
                # Optional: Handle unexpected lines before start token if needed
             except serial.SerialTimeoutException:
                 continue # Should not happen if timeout > 0, but handle defensively
             except UnicodeDecodeError:
                 print(f"IMUAPI: Warning - received non-UTF8 bytes waiting for start: {line_bytes}")
                 continue
             except Exception as e:
                 raise IMUCommunicationError(f"Error reading start token: {e}")

        if not start_token_received:
             raise IMUTimeoutError(f"Timeout waiting for start token '{start_token}' after sending command '{command.decode().strip()}'")


        # --- Wait for COMPLETE or FAIL token ---
        wait_start_time = time.time()
        while time.time() - wait_start_time < timeout_seconds:
            try:
                # Use a short timeout for readline within the loop
                # to allow checking the outer loop's time condition frequently
                line_bytes = self.ser.readline() # Use default timeout configured in __init__
                if not line_bytes:
                    # Readline timed out for this attempt, continue loop checking overall timeout
                    continue

                line = line_bytes.decode().strip()
                print(f"IMUAPI: Received: {line}") # Log intermediate lines

                if line.startswith(complete_token):
                    return True # Success
                if line.startswith(fail_token):
                    raise IMUCalibrationError(f"Calibration failed. IMU reported: '{line}'")

            except serial.SerialTimeoutException:
                # Readline timed out, continue loop checking overall timeout
                continue
            except UnicodeDecodeError:
                print(f"IMUAPI: Warning - received non-UTF8 bytes waiting for status: {line_bytes}")
                continue
            except Exception as e:
                raise IMUCommunicationError(f"Unexpected error while waiting for status: {e}")

        # If loop finishes without returning/raising, overall timeout occurred
        raise IMUTimeoutError(f"Timeout ({timeout_seconds}s) waiting for '{complete_token}' or '{fail_token}' after receiving '{start_token}'")


    def calibrate_time(self) -> int:
        """
        Calibrate time offset between host and IMU.

        Returns:
            The timestamp (microseconds) sent back by the IMU as the epoch reference.

        Raises:
            IMUTimeoutError, IMUCommunicationError
        """
        if not self.ser.is_open:
             raise IMUCommunicationError("Serial port is not open.")

        command = b'time_calibrate\n'
        print(f"IMUAPI: Sending command: {command.decode().strip()}")
        self.ser.write(command)
        self.ser.flush()

        try:
            # Use slightly longer timeout for this specific response
            response_bytes = self.ser.readline()
            if not response_bytes:
                 raise IMUTimeoutError("Timeout waiting for TIME_ECHO response.")

            response = response_bytes.decode().strip()
            print(f"IMUAPI: Received: {response}")

            if response.startswith('TIME_ECHO'):
                try:
                    # Parse as integer
                    timestamp = int(response.split(',')[1])
                    print(f"IMUAPI: Time calibrated. IMU Epoch (us): {timestamp}")
                    return timestamp
                except (IndexError, ValueError):
                     raise IMUCommunicationError(f"Failed to parse timestamp from TIME_ECHO response: {response}")
            else:
                # Read a few more lines maybe? Sometimes READY comes after.
                extra_line = self.ser.readline().decode().strip()
                print(f"IMUAPI: Received unexpected line after time_calibrate: {extra_line}")
                raise IMUCommunicationError(f"Time calibration failed. Expected 'TIME_ECHO', got '{response}'")

        except serial.SerialTimeoutException:
             raise IMUTimeoutError("Timeout waiting for TIME_ECHO response.")
        except UnicodeDecodeError as e:
             raise IMUCommunicationError(f"Failed to decode TIME_ECHO response: {e}")
        except Exception as e:
             raise IMUCommunicationError(f"Error during time calibration: {e}")

    def calibrate_gravity(self, timeout_seconds: float = 5.0) -> None:
        """
        Calibrate gravity compensation with IMU held still.

        Args:
            timeout_seconds: How long to wait for the calibration to complete.
        """
        self._send_command_and_wait_for_status(
            command=b'gravity_calibrate\n',
            start_token='GRAVITY_CAL_START',
            complete_token='GRAVITY_CAL_COMPLETE',
            fail_token='GRAVITY_CAL_FAILED',
            timeout_seconds=timeout_seconds
        )
        print("IMUAPI: Gravity calibration successful.")

    def calibrate_gyro(self, timeout_seconds: float = 5.0) -> None:
        """
        Calibrate gyroscope bias with IMU held still.

        Args:
            timeout_seconds: How long to wait for the calibration to complete.
        """
        self._send_command_and_wait_for_status(
            command=b'gyro_calibrate\n',
            start_token='GYRO_CAL_START',
            complete_token='GYRO_CAL_COMPLETE',
            fail_token='GYRO_CAL_FAILED',
            timeout_seconds=timeout_seconds
        )
        print("IMUAPI: Gyro calibration successful.")

    def calibrate_mag(self, timeout_seconds: float = 15.0) -> None:
        """
        Calibrate magnetometer by rotating IMU through all orientations.
        The Arduino sketch handles the 10-second rotation period internally.

        Args:
            timeout_seconds: How long to wait for the calibration to complete
                             (should be > 10s).
        """
        if timeout_seconds <= 10.0:
             print("IMUAPI: Warning - Magnetometer calibration timeout should be greater than 10 seconds.")

        self._send_command_and_wait_for_status(
            command=b'mag_calibrate\n',
            start_token='MAG_CAL_START',
            complete_token='MAG_CAL_COMPLETE',
            fail_token='MAG_CAL_FAILED',
            timeout_seconds=timeout_seconds
        )
        print("IMUAPI: Magnetometer calibration successful.")

    def start_collection(self) -> bool:
        """
        Sends the command to start data collection on the IMU.

        Returns:
             True if collection started successfully, False otherwise (e.g., time not calibrated).

        Raises:
             IMUTimeoutError, IMUCommunicationError if serial communication fails.
        """
        if not self.ser.is_open:
             raise IMUCommunicationError("Serial port is not open.")

        command = b'start_collection\n'
        print(f"IMUAPI: Sending command: {command.decode().strip()}")
        self.ser.write(command)
        self.ser.flush()

        # Read response lines
        start_time = time.time()
        response_found = False
        success = False
        while time.time() - start_time < self.ser.timeout + 1: # Use short timeout window
             try:
                line_bytes = self.ser.readline()
                if not line_bytes: continue

                line = line_bytes.decode().strip()
                print(f"IMUAPI: Received: {line}")
                if line.startswith("STARTING_COLLECTION"):
                    response_found = True
                    success = True
                    break
                elif line.startswith("ERROR: Time must be calibrated first"):
                     print(f"IMUAPI: Error from IMU: {line}")
                     response_found = True
                     success = False
                     break
                elif line.startswith("WARNING: Already collecting"): # Handle warning
                     print(f"IMUAPI: Warning from IMU: {line}")
                     response_found = True
                     success = False # Or True depending on desired behavior? Let's say False.
                     break
                # Ignore other potential lines like READY if sent unexpectedly

             except serial.SerialTimeoutException:
                 continue
             except UnicodeDecodeError:
                 print(f"IMUAPI: Warning - received non-UTF8 bytes response: {line_bytes}")
                 continue
             except Exception as e:
                 raise IMUCommunicationError(f"Error reading start_collection response: {e}")

        if not response_found:
             raise IMUTimeoutError("Timeout waiting for response after start_collection command.")

        return success


    def stop_collection(self) -> bool:
        """
        Sends the command to stop data collection on the IMU.

        Returns:
             True if collection stopped successfully or was already stopped.

        Raises:
             IMUTimeoutError, IMUCommunicationError if serial communication fails.
        """
        if not self.ser.is_open:
             raise IMUCommunicationError("Serial port is not open.")

        command = b'stop_collection\n'
        print(f"IMUAPI: Sending command: {command.decode().strip()}")
        self.ser.write(command)
        self.ser.flush()

        # Read response lines
        start_time = time.time()
        response_found = False
        success = False
        while time.time() - start_time < self.ser.timeout + 1: # Use short timeout window
            try:
                line_bytes = self.ser.readline()
                if not line_bytes: continue

                line = line_bytes.decode().strip()
                print(f"IMUAPI: Received: {line}")
                # Arduino sends STOPPING_COLLECTION then READY and menu
                # We only care about the first confirmation.
                if line.startswith("STOPPING_COLLECTION"):
                    response_found = True
                    success = True
                    break
                elif line.startswith("INFO: Not currently collecting data"):
                     response_found = True
                     success = True # Treat as success
                     break
                elif line.startswith("READY"): # If we missed the stop message but got READY
                     if not response_found: # Check if we already found the target message
                          print("IMUAPI: Received READY, assuming stop was successful or already stopped.")
                          response_found = True
                          success = True
                     # Don't break yet, might still get the STOPPING message
                # Ignore menu lines etc.

            except serial.SerialTimeoutException:
                 continue
            except UnicodeDecodeError:
                print(f"IMUAPI: Warning - received non-UTF8 bytes response: {line_bytes}")
                continue
            except Exception as e:
                 raise IMUCommunicationError(f"Error reading stop_collection response: {e}")

        if not response_found:
             # If timeout, maybe it stopped silently? Hard to know. Let's raise.
             raise IMUTimeoutError("Timeout waiting for response after stop_collection command.")

        # Consume the rest of the READY message block if needed
        self._flush_initial_output(timeout_seconds=0.5) # Quick flush

        return success

    def read_data_block(self, read_timeout_seconds: float = 2.0) -> Optional[Dict[str, List[Dict]]]:
        """
        Reads one complete data block (BEGIN_DATA_BLOCK to END_DATA_BLOCK) from the IMU.
        This is a blocking call.

        Args:
            read_timeout_seconds: Max time to wait for a complete block to arrive.

        Returns:
            A dictionary containing lists of sensor readings ('M', 'A', 'E', 'G'),
            or None if a complete block is not received within the timeout.
            Each reading is a dict {'timestamp': int, 'x': float, 'y': float, 'z': float}.

        Raises:
            IMUTimeoutError if timeout occurs *while receiving data inside* a block.
            IMUCommunicationError for parsing or decoding errors.
        """
        if not self.ser.is_open:
             raise IMUCommunicationError("Serial port is not open.")

        data: Dict[str, List[Dict]] = {'M': [], 'A': [], 'E': [], 'G': []}
        expected_counts = {'M': 0, 'A': 0, 'E': 0, 'G': 0}
        received_counts = {'M': 0, 'A': 0, 'E': 0, 'G': 0}
        in_block = False
        block_found = False
        start_time = time.time()

        while time.time() - start_time < read_timeout_seconds:
            try:
                line_bytes = self.ser.readline() # Use default timeout from __init__
                if not line_bytes:
                    # Readline timeout occurred. If we weren't in a block, maybe no data sent yet.
                    # If we *were* in a block, it means data stopped mid-stream.
                    if in_block:
                         # This indicates a problem - data stopped unexpectedly
                         raise IMUTimeoutError(f"Timeout occurred *inside* data block after {time.time() - start_time:.2f}s.")
                    else:
                         # No data received in this read cycle, continue waiting if overall time allows
                         if time.time() - start_time >= read_timeout_seconds:
                             # Overall timeout exceeded without even starting a block
                             return None # Indicate no block found
                         continue # Continue waiting for block start

                line = line_bytes.decode().strip()
                # Optional: print(f"DEBUG Read: {line}")

                if not in_block:
                    if line.startswith("BEGIN_DATA_BLOCK"):
                        parts = line.split(',')
                        if len(parts) != 5:
                            print(f"IMUAPI: Warning - Malformed BEGIN_DATA_BLOCK: {line}")
                            continue
                        try:
                            expected_counts['M'] = int(parts[1])
                            expected_counts['A'] = int(parts[2])
                            expected_counts['E'] = int(parts[3])
                            expected_counts['G'] = int(parts[4])
                            in_block = True
                            block_found = True # Mark that we have at least started a block
                            # Reset data structure for the new block
                            data = {'M': [], 'A': [], 'E': [], 'G': []}
                            received_counts = {'M': 0, 'A': 0, 'E': 0, 'G': 0}
                            # print(f"DEBUG: Starting block. Expecting M:{expected_counts['M']}, A:{expected_counts['A']}, E:{expected_counts['E']}, G:{expected_counts['G']}")
                        except ValueError:
                            print(f"IMUAPI: Warning - Could not parse counts in BEGIN_DATA_BLOCK: {line}")
                            # Remain not in_block
                    # Ignore other lines when not in a block (like READY, command echoes, etc.)

                else: # We are inside a block
                    if line.startswith("END_DATA_BLOCK"):
                        # print(f"DEBUG: End Block Received. Got M:{received_counts['M']}, A:{received_counts['A']}, E:{received_counts['E']}, G:{received_counts['G']}")
                        # Verify counts (optional but recommended)
                        if (received_counts['M'] == expected_counts['M'] and
                            received_counts['A'] == expected_counts['A'] and
                            received_counts['E'] == expected_counts['E'] and
                            received_counts['G'] == expected_counts['G']):
                            # print("DEBUG: Counts match.")
                            return data # Successfully read the block
                        else:
                            print(f"IMUAPI: Warning - Mismatched counts at END_DATA_BLOCK. Got M:{received_counts['M']}/{expected_counts['M']}, A:{received_counts['A']}/{expected_counts['A']}, E:{received_counts['E']}/{expected_counts['E']}, G:{received_counts['G']}/{expected_counts['G']}")
                            return data # Return potentially incomplete data with a warning

                    elif len(line) > 2 and line[1] == ',':
                        sensor_type = line[0]
                        if sensor_type in data:
                            parts = line.split(',')
                            if len(parts) == 5:
                                try:
                                    reading = {
                                        'timestamp': int(parts[1]), # Arduino sends unsigned long
                                        'x': float(parts[2]),
                                        'y': float(parts[3]),
                                        'z': float(parts[4])
                                    }
                                    # Check if we are receiving more than expected?
                                    if received_counts[sensor_type] < expected_counts[sensor_type]:
                                        data[sensor_type].append(reading)
                                        received_counts[sensor_type] += 1
                                    else:
                                        # This case should ideally not happen if Arduino code is correct
                                        print(f"IMUAPI: Warning - Received more {sensor_type} readings than expected ({expected_counts[sensor_type]}) in block. Ignoring extra.")
                                except (ValueError, IndexError):
                                    print(f"IMUAPI: Warning - Could not parse sensor data line: {line}")
                            else:
                                print(f"IMUAPI: Warning - Malformed sensor data line: {line}")
                        # else: ignore lines that don't start with a known sensor type inside a block
                    else:
                         # Handle other potential lines inside a block if necessary
                         print(f"IMUAPI: Warning - Unexpected line inside data block: {line}")

            except serial.SerialTimeoutException:
                # Should be handled by checking `if not line_bytes:` above
                 if in_block:
                     raise IMUTimeoutError(f"Timeout occurred *inside* data block (SerialTimeoutException) after {time.time() - start_time:.2f}s.")
                 else:
                     # Check overall timeout before continuing
                      if time.time() - start_time >= read_timeout_seconds:
                          return None
                      continue
            except UnicodeDecodeError:
                print(f"IMUAPI: Warning - received non-UTF8 bytes: {line_bytes}")
                continue # Ignore undecodable lines
            except Exception as e:
                # Log unexpected errors
                print(f"IMUAPI: Error during read_data_block: {e}")
                # Decide: re-raise, return None, or return partial data?
                # Let's return None to indicate failure to read a complete block cleanly
                raise IMUCommunicationError(f"Unexpected error processing line: {line if 'line' in locals() else 'N/A'}: {e}")


        # If loop finishes, the overall timeout was exceeded
        if block_found and not in_block:
             # This should not happen if END_DATA_BLOCK logic is correct, but as fallback:
             print("IMUAPI: Warning - Timeout occurred after block started but before END_DATA_BLOCK received.")
             return data # Return partial data
        elif not block_found:
             # print("DEBUG: Overall timeout exceeded without finding BEGIN_DATA_BLOCK.")
             return None # Indicate timeout without finding a complete block
        else: # Should be covered by loop logic, but for completeness
             print("IMUAPI: Warning - Read loop exited unexpectedly.")
             return None


    def close(self):
        """Closes the serial port."""
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
                print(f"IMUAPI: Serial port {self.ser.port} closed.")
            except Exception as e:
                 print(f"IMUAPI: Error closing serial port: {e}")
        self.ser = None # Indicate port is closed


# Example Usage (requires a config.ini file)
if __name__ == '__main__':
    # Create a dummy config.ini for testing if it doesn't exist
    CONFIG_FILE = 'config.ini' # Assuming in same directory for example
    try:
        with open(CONFIG_FILE, 'r') as f:
             pass
        print(f"Using existing config file: {CONFIG_FILE}")
    except FileNotFoundError:
        print(f"Creating dummy config file: {CONFIG_FILE}")
        with open(CONFIG_FILE, 'w') as f:
            f.write("[IMU]\n")
            # --- !!! CHANGE THIS TO YOUR ARDUINO'S PORT !!! ---
            # Examples: 'COM3' on Windows, '/dev/ttyACM0' or '/dev/ttyUSB0' on Linux, '/dev/cu.usbmodemXXXX' on macOS
            f.write("port = COM3\n")
            f.write("baud_rate = 115200\n")
            # f.write("\n[Calibration]\n") # Not currently used
            # f.write("still_delay_seconds = 5\n")
            # f.write("rotation_delay_seconds = 15\n")

    try:
        print("Initializing IMUAPI...")
        # Use a slightly longer default timeout for potentially slower Arduinos
        imu = IMUAPI(config_path=CONFIG_FILE, serial_timeout=2.0)

        # --- Calibration Sequence ---
        print("\nStarting Calibration...")
        try:
             # 1. Time Calibration (Required before collection)
             print("Calibrating Time...")
             imu_epoch_us = imu.calibrate_time()
             print(f"Time Calibration Complete. IMU Epoch: {imu_epoch_us} us")
             time_calibrated = True

             # 2. Gravity Calibration (Hold Still)
             input("Prepare for Gravity Calibration. Keep the IMU perfectly still, then press Enter...")
             print("Calibrating Gravity...")
             imu.calibrate_gravity(timeout_seconds=5.0) # Wait up to 5s
             print("Gravity Calibration Complete.")

             # 3. Gyroscope Calibration (Hold Still)
             input("Prepare for Gyro Calibration. Keep the IMU perfectly still, then press Enter...")
             print("Calibrating Gyroscope...")
             imu.calibrate_gyro(timeout_seconds=5.0) # Wait up to 5s
             print("Gyro Calibration Complete.")

             # 4. Magnetometer Calibration (Rotate)
             input("Prepare for Magnetometer Calibration. Press Enter to start, then rotate slowly for 10 seconds...")
             print("Calibrating Magnetometer...")
             imu.calibrate_mag(timeout_seconds=15.0) # Wait up to 15s (covers 10s + buffer)
             print("Magnetometer Calibration Complete.")

        except (IMUCommunicationError, IMUCalibrationError, IMUTimeoutError) as e:
            print(f"\n--- CALIBRATION FAILED ---")
            print(f"Error: {e}")
            imu.close()
            exit()
        except Exception as e:
             print(f"\n--- UNEXPECTED ERROR DURING CALIBRATION ---")
             print(f"Error: {e}")
             imu.close()
             exit()


        # --- Data Collection Example ---
        if time_calibrated: # Only proceed if time was calibrated
            print("\nStarting Data Collection Example...")
            try:
                if imu.start_collection():
                    print("Collection started on IMU.")
                    collection_start_time = time.time()
                    num_blocks_received = 0
                    max_blocks = 10 # Collect 10 blocks for this example
                    print(f"Attempting to read {max_blocks} data blocks...")

                    while num_blocks_received < max_blocks:
                         # Wait for the next data block
                         print(f"Waiting for block {num_blocks_received + 1}/{max_blocks}...")
                         data_block = imu.read_data_block(read_timeout_seconds=3.0) # Wait up to 3s for a block

                         if data_block:
                             num_blocks_received += 1
                             print(f"--- Block {num_blocks_received} Received ---")
                             # Process the data (Example: print counts)
                             print(f"  Mag Readings: {len(data_block['M'])}")
                             print(f"  Acc Readings: {len(data_block['A'])}")
                             print(f"  Eul Readings: {len(data_block['E'])}")
                             print(f"  Gyr Readings: {len(data_block['G'])}")
                             # Example: Print first accel reading timestamp if available
                             if data_block['A']:
                                 print(f"  First Accel Timestamp (us since epoch): {data_block['A'][0]['timestamp']}")
                         else:
                             print("No data block received within timeout.")
                             # Optional: break or implement retry logic
                             # Check if collection duration exceeded
                             if time.time() - collection_start_time > 30: # Example overall limit
                                  print("Overall collection time limit reached.")
                                  break


                    print("\nStopping data collection...")
                    imu.stop_collection()
                    print("Collection stopped.")

                else:
                     print("Failed to start collection (check IMU state/calibration).")

            except (IMUCommunicationError, IMUTimeoutError) as e:
                 print(f"\n--- DATA COLLECTION FAILED ---")
                 print(f"Error: {e}")
                 # Try to stop collection even if error occurred
                 try:
                     print("Attempting to stop collection after error...")
                     imu.stop_collection()
                 except Exception as stop_e:
                     print(f"Could not stop collection after error: {stop_e}")

            except Exception as e:
                 print(f"\n--- UNEXPECTED ERROR DURING COLLECTION ---")
                 print(f"Error: {e}")
                 # Try to stop collection
                 try:
                     print("Attempting to stop collection after error...")
                     imu.stop_collection()
                 except Exception as stop_e:
                     print(f"Could not stop collection after error: {stop_e}")

        else:
             print("\nSkipping data collection because time was not calibrated.")


    finally:
        # Ensure the port is closed
        if 'imu' in locals() and imu.ser is not None:
             print("Closing serial port.")
             imu.close()

    print("\nIMU API Example Finished.")
import serial
import time
import configparser
from typing import Tuple, Dict, List, Optional # Added Dict, List, Optional

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
    def __init__(self, config_path: str = 'python/config.ini', serial_timeout: float = 1.0):
        """
        Initialize IMU API: Read config, open serial port, wait for device.

        Args:
            config_path: Path to the configuration file.
            serial_timeout: Default timeout in seconds for serial read operations.
                           Calibration methods might use longer internal timeouts.
        """
        print("DEBUG: IMUAPI.__init__ started.") # ADDED
        config = configparser.ConfigParser()
        print(f"DEBUG: Reading config file: {config_path}") # ADDED
        read_files = config.read(config_path)
        if not read_files:
             raise FileNotFoundError(f"Configuration file not found or empty: {config_path}")
        print("DEBUG: Config file read.") # ADDED

        # --- Read IMU Connection Settings ---
        try:
            port = config['IMU']['port']
            baud = int(config['IMU'].get('baud_rate', '115200'))
            print(f"DEBUG: Config - Port={port}, Baud={baud}") # ADDED
        except KeyError as e:
            raise KeyError(f"Missing key in config file '{config_path}': {e}")
        except ValueError as e:
             raise ValueError(f"Invalid numerical value in config file '{config_path}': {e}")

        # --- Read Calibration Delay Settings ---
        try:
            # Load delays from config and store them as instance attributes
            self.still_delay = float(config['Calibration'].get('still_delay_seconds', 5.0))
            self.rotation_delay = float(config['Calibration'].get('rotation_delay_seconds', 10.0)) # Use get for defaults
            print(f"DEBUG: Config - Still Delay={self.still_delay}, Rotation Delay={self.rotation_delay}") # ADDED
        except KeyError as e:
            # Handle missing [Calibration] section or keys if desired, or let it raise
            print(f"IMUAPI: Warning - [Calibration] section or keys missing in config: {e}. Using defaults.")
            self.still_delay = 5.0
            self.rotation_delay = 10.0
        except ValueError as e:
            raise ValueError(f"Invalid numerical value for delays in config file '{config_path}': {e}")

        # --- Establish Serial Connection ---
        try:
            print(f"DEBUG: Attempting to open serial port {port} at {baud} baud...") # ADDED
            # Create the serial port object - THIS CAN BLOCK/FAIL
            self.ser = serial.Serial(port, baud, timeout=serial_timeout)
            print(f"DEBUG: Serial port {port} opened successfully.") # ADDED

            # Wait for Arduino to reset/initialize after connection
            print("DEBUG: Waiting 2 seconds for Arduino to initialize...") # ADDED
            time.sleep(2)
            print("DEBUG: Finished 2-second wait.") # ADDED

            # Read and discard any initial messages from Arduino (e.g., "READY")
            print("DEBUG: Attempting to flush initial Arduino output...") # ADDED
            self._flush_initial_output()
            print("DEBUG: Finished flushing initial output.") # ADDED
            print("IMUAPI: Ready.") # Indicate successful initialization

        except serial.SerialException as e:
            # Handle errors specifically related to opening the serial port
            print(f"DEBUG: Failed to open serial port {port}: {e}") # ADDED
            raise IMUCommunicationError(f"Failed to open serial port {port}: {e}")
        except Exception as e: # ADDED generic exception catch here for debug
            # Catch any other unexpected errors during setup
            print(f"DEBUG: Unexpected error during serial setup or flush: {e}") # ADDED
            raise # Re-raise the exception after printing

    def _flush_initial_output(self, timeout_seconds: float = 2.0):
        """Reads and discards any initial output from the Arduino after reset."""
        print("DEBUG: _flush_initial_output started.") # ADDED
        start_time = time.time()
        # Loop for a maximum duration to avoid infinite wait
        while time.time() - start_time < timeout_seconds:
            try:
                print("DEBUG: _flush_initial_output trying readline()...") # ADDED
                # Attempt to read a line from the serial port
                # This will block until a newline is received or the timeout occurs
                line_bytes = self.ser.readline()
                line = line_bytes.decode(errors='ignore').strip()
                if line:
                    # If a line was received, print it for debugging purposes
                    print(f"DEBUG: Discarding startup line: {line}") # MODIFIED
                else:
                    # If readline timed out (returned empty bytes), maybe no more data
                    print("DEBUG: _flush_initial_output readline() timed out (empty).") # ADDED
                    # Give it one more short try in case buffer wasn't full yet
                    time.sleep(0.1)
                    if not self.ser.readline():
                        print("DEBUG: _flush_initial_output second readline() timed out. Breaking flush loop.") # ADDED
                        break # Assume no more startup messages are coming
            except serial.SerialTimeoutException:
                # This exception isn't typically raised by readline with timeout > 0
                # but handle defensively.
                print("DEBUG: _flush_initial_output caught SerialTimeoutException. Breaking flush loop.") # ADDED
                break
            except Exception as e:
                 # Handle other potential errors during reading/decoding
                 print(f"DEBUG: Warning - Error flushing initial output: {e}") # MODIFIED
                 break # Stop flushing on other errors
        print("DEBUG: _flush_initial_output finished.") # ADDED


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
        if not self.ser or not self.ser.is_open: # Added None check for self.ser
             raise IMUCommunicationError("Serial port is not open.")

        print(f"IMUAPI: Sending command: {command.decode().strip()}")
        self.ser.write(command)
        self.ser.flush() # Ensure command is sent immediately

        # --- Wait for START token ---
        start_wait_start_time = time.time()
        start_token_received = False
        print(f"DEBUG: Waiting for start token '{start_token}'...") # ADDED
        # Wait slightly longer than default serial timeout for the start token
        while time.time() - start_wait_start_time < self.ser.timeout + 1:
             try:
                line_bytes = self.ser.readline()
                if not line_bytes: continue # Read timed out, try again within window

                line = line_bytes.decode().strip()
                print(f"IMUAPI: Received: {line}") # Log received line

                if line.startswith(start_token):
                    print(f"DEBUG: Start token '{start_token}' received.") # ADDED
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
             print(f"DEBUG: Timeout waiting for start token '{start_token}'.") # ADDED
             raise IMUTimeoutError(f"Timeout waiting for start token '{start_token}' after sending command '{command.decode().strip()}'")


        # --- Wait for COMPLETE or FAIL token ---
        wait_start_time = time.time()
        print(f"DEBUG: Waiting for complete ('{complete_token}') or fail ('{fail_token}') token (timeout={timeout_seconds}s)...") # ADDED
        while time.time() - wait_start_time < timeout_seconds:
            try:
                # Use the default serial timeout for readline within the loop
                # to allow checking the outer loop's time condition frequently
                line_bytes = self.ser.readline()
                if not line_bytes:
                    # Readline timed out for this attempt, continue loop checking overall timeout
                    continue

                line = line_bytes.decode().strip()
                print(f"IMUAPI: Received: {line}") # Log intermediate lines

                if line.startswith(complete_token):
                    print(f"DEBUG: Complete token '{complete_token}' received.") # ADDED
                    return True # Success
                if line.startswith(fail_token):
                    print(f"DEBUG: Fail token '{fail_token}' received.") # ADDED
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
        print(f"DEBUG: Timeout waiting for complete/fail token.") # ADDED
        raise IMUTimeoutError(f"Timeout ({timeout_seconds}s) waiting for '{complete_token}' or '{fail_token}' after receiving '{start_token}'")


    def calibrate_time(self) -> int:
        """
        Calibrate time offset between host and IMU. Sends 'time_calibrate' command.

        Returns:
            The timestamp (microseconds) sent back by the IMU as the epoch reference.

        Raises:
            IMUTimeoutError, IMUCommunicationError
        """
        if not self.ser or not self.ser.is_open: # Added None check for self.ser
             raise IMUCommunicationError("Serial port is not open.")

        command = b'time_calibrate\n'
        print(f"IMUAPI: Sending command: {command.decode().strip()}")
        self.ser.write(command)
        self.ser.flush()

        print("DEBUG: Waiting for TIME_ECHO response...") # ADDED
        try:
            # Use slightly longer timeout for this specific response if needed
            # Consider making this configurable or increasing default self.ser.timeout
            response_bytes = self.ser.readline()
            if not response_bytes:
                 print("DEBUG: Timeout waiting for TIME_ECHO.") # ADDED
                 raise IMUTimeoutError("Timeout waiting for TIME_ECHO response.")

            response = response_bytes.decode().strip()
            print(f"IMUAPI: Received: {response}")

            if response.startswith('TIME_ECHO'):
                try:
                    # Parse the timestamp (sent as integer microseconds)
                    timestamp = int(response.split(',')[1])
                    print(f"IMUAPI: Time calibrated. IMU Epoch (us): {timestamp}")
                    return timestamp
                except (IndexError, ValueError):
                     print(f"DEBUG: Failed to parse TIME_ECHO: {response}") # ADDED
                     raise IMUCommunicationError(f"Failed to parse timestamp from TIME_ECHO response: {response}")
            else:
                # Read a few more lines maybe? Sometimes READY comes after.
                print(f"DEBUG: Unexpected response to time_calibrate: {response}. Reading extra lines...") # ADDED
                extra_line = self.ser.readline().decode().strip()
                print(f"IMUAPI: Received unexpected line after time_calibrate: {extra_line}")
                raise IMUCommunicationError(f"Time calibration failed. Expected 'TIME_ECHO', got '{response}'")

        except serial.SerialTimeoutException:
             print("DEBUG: SerialTimeoutException waiting for TIME_ECHO.") # ADDED
             raise IMUTimeoutError("Timeout waiting for TIME_ECHO response.")
        except UnicodeDecodeError as e:
             print(f"DEBUG: UnicodeDecodeError reading TIME_ECHO response: {e}") # ADDED
             raise IMUCommunicationError(f"Failed to decode TIME_ECHO response: {e}")
        except Exception as e:
             print(f"DEBUG: Unexpected error during time calibration: {e}") # ADDED
             raise IMUCommunicationError(f"Error during time calibration: {e}")

    def calibrate_gravity(self, timeout_seconds: float = 5.0) -> None:
        """
        Calibrate gravity compensation with IMU held still. Sends 'gravity_calibrate'.
        Uses the generic command sending function.

        Args:
            timeout_seconds: How long to wait for the calibration to complete.
        """
        print("DEBUG: Starting gravity calibration...") # ADDED
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
        Calibrate gyroscope bias with IMU held still. Sends 'gyro_calibrate'.
        Uses the generic command sending function.

        Args:
            timeout_seconds: How long to wait for the calibration to complete.
        """
        print("DEBUG: Starting gyro calibration...") # ADDED
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
        Calibrate magnetometer by rotating IMU through all orientations. Sends 'mag_calibrate'.
        The Arduino sketch handles the 10-second rotation period internally.
        Uses the generic command sending function.

        Args:
            timeout_seconds: How long to wait for the calibration to complete
                             (should be > 10s to allow for rotation).
        """
        print("DEBUG: Starting magnetometer calibration...") # ADDED
        if timeout_seconds <= 10.0:
             print("IMUAPI: Warning - Magnetometer calibration timeout should ideally be greater than 10 seconds.")

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
        Sends the command 'start_collection' to start data streaming on the IMU.

        Returns:
             True if collection started successfully, False otherwise (e.g., time not calibrated).

        Raises:
             IMUTimeoutError, IMUCommunicationError if serial communication fails.
        """
        if not self.ser or not self.ser.is_open: # Added None check for self.ser
             raise IMUCommunicationError("Serial port is not open.")

        command = b'start_collection\n'
        print(f"IMUAPI: Sending command: {command.decode().strip()}")
        self.ser.write(command)
        self.ser.flush()

        # Read response lines to confirm start or get error
        print("DEBUG: Waiting for start_collection response...") # ADDED
        start_time = time.time()
        response_found = False
        success = False
        # Wait slightly longer than default timeout for response
        while time.time() - start_time < self.ser.timeout + 1:
             try:
                line_bytes = self.ser.readline()
                if not line_bytes: continue # Timeout on this read, continue loop

                line = line_bytes.decode().strip()
                print(f"IMUAPI: Received: {line}")
                if line.startswith("STARTING_COLLECTION"):
                    print("DEBUG: 'STARTING_COLLECTION' received.") # ADDED
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
                     success = False # Treat as failure to start *new* collection
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
             print("DEBUG: Timeout waiting for start_collection response.") # ADDED
             raise IMUTimeoutError("Timeout waiting for response after start_collection command.")

        return success


    def stop_collection(self) -> bool:
        """
        Sends the command 'stop_collection' to stop data streaming on the IMU.

        Returns:
             True if collection stopped successfully or was already stopped.

        Raises:
             IMUTimeoutError, IMUCommunicationError if serial communication fails.
        """
        if not self.ser or not self.ser.is_open: # Added None check for self.ser
             raise IMUCommunicationError("Serial port is not open.")

        command = b'stop_collection\n'
        print(f"IMUAPI: Sending command: {command.decode().strip()}")
        self.ser.write(command)
        self.ser.flush()

        # Read response lines to confirm stop
        print("DEBUG: Waiting for stop_collection response...") # ADDED
        start_time = time.time()
        response_found = False
        success = False
        # Wait slightly longer than default timeout for response
        while time.time() - start_time < self.ser.timeout + 1:
            try:
                line_bytes = self.ser.readline()
                if not line_bytes: continue # Timeout on this read, continue loop

                line = line_bytes.decode().strip()
                print(f"IMUAPI: Received: {line}")
                # Arduino sends STOPPING_COLLECTION then READY and menu
                # We only care about the first confirmation.
                if line.startswith("STOPPING_COLLECTION"):
                    print("DEBUG: 'STOPPING_COLLECTION' received.") # ADDED
                    response_found = True
                    success = True
                    break # Found the primary success message
                elif line.startswith("INFO: Not currently collecting data"):
                     print("DEBUG: 'INFO: Not currently collecting data' received.") # ADDED
                     response_found = True
                     success = True # Treat as success (already stopped)
                     break
                elif line.startswith("READY"): # If we missed the stop message but got READY
                     if not response_found: # Check if we already found the target message
                          print("IMUAPI: Received READY, assuming stop was successful or already stopped.")
                          response_found = True
                          success = True
                     # Don't break yet, might still get the STOPPING message or menu lines
                # Ignore menu lines etc.

            except serial.SerialTimeoutException:
                 continue
            except UnicodeDecodeError:
                print(f"IMUAPI: Warning - received non-UTF8 bytes response: {line_bytes}")
                continue
            except Exception as e:
                 raise IMUCommunicationError(f"Error reading stop_collection response: {e}")

        if not response_found:
             # If timeout, maybe it stopped silently? Hard to know for sure. Let's raise.
             print("DEBUG: Timeout waiting for stop_collection response.") # ADDED
             raise IMUTimeoutError("Timeout waiting for response after stop_collection command.")

        # Consume the rest of the READY message block if Arduino sends more
        print("DEBUG: Flushing output after stop_collection...") # ADDED
        self._flush_initial_output(timeout_seconds=0.5) # Quick flush
        print("DEBUG: Finished flushing after stop_collection.") # ADDED

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
        if not self.ser or not self.ser.is_open: # Added None check for self.ser
             raise IMUCommunicationError("Serial port is not open.")

        # Initialize data structures for the block
        data: Dict[str, List[Dict]] = {'M': [], 'A': [], 'E': [], 'G': []}
        expected_counts = {'M': 0, 'A': 0, 'E': 0, 'G': 0}
        received_counts = {'M': 0, 'A': 0, 'E': 0, 'G': 0}
        in_block = False # Flag indicating if we are currently inside a data block
        block_found = False # Flag indicating if we have seen BEGIN_DATA_BLOCK
        start_time = time.time()
        # print(f"DEBUG: read_data_block waiting (timeout={read_timeout_seconds}s)...") # Optional: Can be noisy

        # Loop until overall timeout
        while time.time() - start_time < read_timeout_seconds:
            try:
                # Read a line using the default serial timeout
                line_bytes = self.ser.readline()
                if not line_bytes:
                    # Readline timed out for this attempt.
                    if in_block:
                         # If we were inside a block, data stopped unexpectedly mid-stream.
                         print(f"DEBUG: Timeout occurred *inside* data block after {time.time() - start_time:.2f}s.") # ADDED
                         raise IMUTimeoutError(f"Timeout occurred *inside* data block after {time.time() - start_time:.2f}s.")
                    else:
                         # Not in a block, just means no data arrived in this read cycle.
                         # Check if overall timeout is exceeded before continuing.
                         if time.time() - start_time >= read_timeout_seconds:
                             # print("DEBUG: Overall timeout exceeded without starting a block.") # Optional Debug
                             return None # Indicate no block found within overall timeout
                         continue # Continue waiting for block start

                # Decode the received line
                line = line_bytes.decode().strip()
                # Optional: print(f"DEBUG Read: {line}") # Very noisy debug

                if not in_block:
                    # --- Waiting for Block Start ---
                    if line.startswith("BEGIN_DATA_BLOCK"):
                        print(f"DEBUG: Received BEGIN_DATA_BLOCK: {line}") # ADDED
                        parts = line.split(',')
                        if len(parts) != 5:
                            print(f"IMUAPI: Warning - Malformed BEGIN_DATA_BLOCK: {line}")
                            continue # Ignore malformed line
                        try:
                            # Parse expected counts for each sensor type
                            expected_counts['M'] = int(parts[1])
                            expected_counts['A'] = int(parts[2])
                            expected_counts['E'] = int(parts[3])
                            expected_counts['G'] = int(parts[4])
                            in_block = True     # Set flag: now inside a block
                            block_found = True  # Set flag: block has started
                            # Reset data structure for the new block
                            data = {'M': [], 'A': [], 'E': [], 'G': []}
                            received_counts = {'M': 0, 'A': 0, 'E': 0, 'G': 0}
                            # print(f"DEBUG: Starting block. Expecting M:{expected_counts['M']}, A:{expected_counts['A']}, E:{expected_counts['E']}, G:{expected_counts['G']}")
                        except ValueError:
                            print(f"IMUAPI: Warning - Could not parse counts in BEGIN_DATA_BLOCK: {line}")
                            # Remain not in_block, wait for next BEGIN
                    # Ignore other lines when not in a block (like READY, command echoes, etc.)

                else:
                    # --- Inside a Block ---
                    if line.startswith("END_DATA_BLOCK"):
                        print(f"DEBUG: Received END_DATA_BLOCK. Got M:{received_counts['M']}, A:{received_counts['A']}, E:{received_counts['E']}, G:{received_counts['G']}") # ADDED
                        in_block = False # Exiting the block
                        # Verify counts (optional but recommended)
                        if (received_counts['M'] == expected_counts['M'] and
                            received_counts['A'] == expected_counts['A'] and
                            received_counts['E'] == expected_counts['E'] and
                            received_counts['G'] == expected_counts['G']):
                            # print("DEBUG: Counts match.")
                            return data # Successfully read the complete block
                        else:
                            # Counts mismatch - data might be corrupt or incomplete
                            print(f"IMUAPI: Warning - Mismatched counts at END_DATA_BLOCK. Got M:{received_counts['M']}/{expected_counts['M']}, A:{received_counts['A']}/{expected_counts['A']}, E:{received_counts['E']}/{expected_counts['E']}, G:{received_counts['G']}/{expected_counts['G']}")
                            return data # Return potentially incomplete data with a warning

                    elif len(line) > 2 and line[1] == ',':
                        # --- Processing Sensor Data Line ---
                        sensor_type = line[0]
                        if sensor_type in data: # Check if it's a known sensor type (M, A, E, G)
                            parts = line.split(',')
                            if len(parts) == 5: # Expect Type,Timestamp,X,Y,Z
                                try:
                                    # Parse the sensor reading
                                    reading = {
                                        'timestamp': int(parts[1]), # Arduino sends unsigned long
                                        'x': float(parts[2]),
                                        'y': float(parts[3]),
                                        'z': float(parts[4])
                                    }
                                    # Add reading to the list if we haven't received the expected count yet
                                    if received_counts[sensor_type] < expected_counts[sensor_type]:
                                        data[sensor_type].append(reading)
                                        received_counts[sensor_type] += 1
                                    else:
                                        # Received more data than announced in BEGIN_DATA_BLOCK
                                        # This indicates an issue with the Arduino code or transmission
                                        print(f"IMUAPI: Warning - Received more {sensor_type} readings than expected ({expected_counts[sensor_type]}) in block. Ignoring extra: {line}")
                                except (ValueError, IndexError):
                                    # Failed to parse numbers in the line
                                    print(f"IMUAPI: Warning - Could not parse sensor data line: {line}")
                            else:
                                # Line started with a sensor type but had wrong number of parts
                                print(f"IMUAPI: Warning - Malformed sensor data line: {line}")
                        # else: ignore lines that don't start with a known sensor type inside a block
                    else:
                         # Handle other unexpected lines inside a block if necessary
                         print(f"IMUAPI: Warning - Unexpected line inside data block: {line}")

            except serial.SerialTimeoutException:
                # Should be handled by checking `if not line_bytes:` above
                 if in_block:
                     print(f"DEBUG: SerialTimeoutException occurred *inside* data block after {time.time() - start_time:.2f}s.") # ADDED
                     raise IMUTimeoutError(f"Timeout occurred *inside* data block (SerialTimeoutException) after {time.time() - start_time:.2f}s.")
                 else:
                     # Check overall timeout before continuing
                      if time.time() - start_time >= read_timeout_seconds:
                          # print("DEBUG: Overall timeout exceeded while waiting for data (SerialTimeoutException).") # Optional Debug
                          return None
                      continue
            except UnicodeDecodeError:
                # Received bytes that couldn't be decoded as UTF-8
                print(f"IMUAPI: Warning - received non-UTF8 bytes: {line_bytes}")
                continue # Ignore undecodable lines
            except Exception as e:
                # Log other unexpected errors during block processing
                print(f"IMUAPI: Error during read_data_block: {e}")
                # Decide how to handle: re-raise, return None, or return partial data?
                # Let's re-raise to indicate a significant failure.
                raise IMUCommunicationError(f"Unexpected error processing line: {line if 'line' in locals() else 'N/A'}: {e}")


        # --- Overall Timeout Handling ---
        # If loop finishes, the overall read_timeout_seconds was exceeded
        if in_block:
             # Timeout occurred after BEGIN but before END was received
             print(f"DEBUG: Overall read_data_block timeout ({read_timeout_seconds}s) occurred *inside* a block.") # ADDED
             # Decide whether to return partial data or None/raise error
             # Returning partial data might be useful sometimes, but indicates an issue.
             print("IMUAPI: Warning - Timeout occurred after block started but before END_DATA_BLOCK received.")
             return data # Return partial data
        elif not block_found:
             # Timeout occurred without ever seeing a BEGIN_DATA_BLOCK
             # print(f"DEBUG: Overall read_data_block timeout ({read_timeout_seconds}s) without finding BEGIN_DATA_BLOCK.") # Optional Debug
             return None # Indicate timeout without finding any block
        else: # block_found is True, but in_block is False (should have returned successfully)
             # This state should ideally not be reached if END_DATA_BLOCK logic is correct
             print("IMUAPI: Warning - Read loop exited unexpectedly after block completion.")
             return None # Or return data if it was potentially valid


    def close(self):
        """Closes the serial port if it's open."""
        print("DEBUG: IMUAPI.close() called.") # ADDED
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
                print(f"IMUAPI: Serial port {self.ser.port} closed.")
            except Exception as e:
                 print(f"IMUAPI: Error closing serial port: {e}")
        else:
            print("DEBUG: Serial port was already closed or not initialized.") # ADDED
        self.ser = None # Indicate port is closed/unavailable


# Example Usage Block (only runs when script is executed directly)
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
            f.write("port = /dev/ttyACM0\n") # EDIT THIS LINE AS NEEDED
            f.write("baud_rate = 115200\n")
            f.write("\n[Calibration]\n") # Add calibration section
            f.write("still_delay_seconds = 5\n")
            f.write("rotation_delay_seconds = 10\n") # Reduced default for example

    imu_instance = None # Initialize variable to hold the IMU instance
    try:
        print("Initializing IMUAPI...")
        # Use a slightly longer default timeout for potentially slower Arduinos or initial connection
        imu_instance = IMUAPI(config_path=CONFIG_FILE, serial_timeout=2.0)

        # --- Calibration Sequence ---
        print("\nStarting Calibration Example...")
        time_calibrated = False # Flag to track if time calibration succeeded
        try:
             # 1. Time Calibration (Required before collection)
             print("Attempting Time Calibration...")
             imu_epoch_us = imu_instance.calibrate_time()
             print(f"Time Calibration Complete. IMU Epoch: {imu_epoch_us} us")
             time_calibrated = True

             # 2. Gravity Calibration (Hold Still)
             input("Prepare for Gravity Calibration. Keep the IMU perfectly still, then press Enter...")
             print("Attempting Gravity Calibration...")
             imu_instance.calibrate_gravity(timeout_seconds=7.0) # Slightly longer timeout
             print("Gravity Calibration Complete.")

             # 3. Gyroscope Calibration (Hold Still)
             input("Prepare for Gyro Calibration. Keep the IMU perfectly still, then press Enter...")
             print("Attempting Gyroscope Calibration...")
             imu_instance.calibrate_gyro(timeout_seconds=7.0) # Slightly longer timeout
             print("Gyro Calibration Complete.")

             # 4. Magnetometer Calibration (Rotate)
             input("Prepare for Magnetometer Calibration. Press Enter to start, then rotate slowly for 10 seconds...")
             print("Attempting Magnetometer Calibration...")
             imu_instance.calibrate_mag(timeout_seconds=15.0) # Wait up to 15s (covers 10s rotation + buffer)
             print("Magnetometer Calibration Complete.")
             print("\n--- ALL CALIBRATIONS COMPLETE ---")

        except (IMUCommunicationError, IMUCalibrationError, IMUTimeoutError) as e:
            print(f"\n--- CALIBRATION FAILED ---")
            print(f"Error: {e}")
            # No exit() here, allow finally block to close port
        except Exception as e:
             print(f"\n--- UNEXPECTED ERROR DURING CALIBRATION ---")
             print(f"Error: {e}")
             # No exit() here, allow finally block to close port


        # --- Data Collection Example ---
        if time_calibrated: # Only proceed if time was calibrated
            print("\nStarting Data Collection Example...")
            try:
                if imu_instance.start_collection():
                    print("Collection started on IMU.")
                    collection_start_time = time.time()
                    num_blocks_received = 0
                    max_blocks = 5 # Collect fewer blocks for the example
                    print(f"Attempting to read {max_blocks} data blocks...")

                    while num_blocks_received < max_blocks:
                         # Wait for the next data block
                         print(f"Waiting for block {num_blocks_received + 1}/{max_blocks}...")
                         # Wait up to 3 seconds for a block to arrive
                         data_block = imu_instance.read_data_block(read_timeout_seconds=3.0)

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
                             # Check if collection duration exceeded an overall limit
                             if time.time() - collection_start_time > 20: # Example overall limit (20s)
                                  print("Overall collection time limit reached.")
                                  break


                    print("\nStopping data collection...")
                    imu_instance.stop_collection()
                    print("Collection stopped.")

                else:
                     print("Failed to start collection (check IMU state/calibration).")

            except (IMUCommunicationError, IMUTimeoutError) as e:
                 print(f"\n--- DATA COLLECTION FAILED ---")
                 print(f"Error: {e}")
                 # Try to stop collection even if error occurred during reading
                 try:
                     if imu_instance: # Check if instance exists
                         print("Attempting to stop collection after error...")
                         imu_instance.stop_collection()
                 except Exception as stop_e:
                     print(f"Could not stop collection after error: {stop_e}")

            except Exception as e:
                 print(f"\n--- UNEXPECTED ERROR DURING COLLECTION ---")
                 print(f"Error: {e}")
                 # Try to stop collection
                 try:
                     if imu_instance: # Check if instance exists
                         print("Attempting to stop collection after error...")
                         imu_instance.stop_collection()
                 except Exception as stop_e:
                     print(f"Could not stop collection after error: {stop_e}")

        else:
             print("\nSkipping data collection because time was not calibrated.")

    except (FileNotFoundError, KeyError, ValueError, IMUCommunicationError) as e:
        # Catch errors during the initial IMUAPI instantiation
        print(f"\n--- FAILED TO INITIALIZE IMUAPI ---")
        print(f"Error: {e}")
    except Exception as e:
        # Catch any other unexpected errors during the setup phase
        print(f"\n--- UNEXPECTED ERROR DURING SETUP ---")
        print(f"Error: {e}")

    finally:
        # Ensure the port is closed regardless of how the script exits
        if imu_instance: # Check if instance was successfully created
             print("\nClosing serial port in finally block.")
             imu_instance.close()
        else:
             print("\nNo IMU instance to close.")

    print("\nIMU API Example Finished.")

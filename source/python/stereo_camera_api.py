import configparser
import cv2
import os
import time
import datetime
from typing import Tuple, Optional, Dict, Any

class StereoCameraSystem:
    """
    Handles stereo camera setup, capture, processing (flipping/swapping),
    and saving based on configuration settings.
    """

    def __init__(self, config_path: str = 'config.ini'):
        """
        Initializes the system by reading the configuration file.

        Args:
            config_path: Path to the configuration (.ini) file.

        Raises:
            FileNotFoundError: If the config file is not found.
            KeyError: If required keys are missing in the config file.
            ValueError: If config values are invalid.
        """
        print(f"Initializing StereoCameraSystem from: {config_path}")
        self.config = configparser.ConfigParser()
        if not os.path.exists(config_path):
            raise FileNotFoundError(f"Configuration file not found: {config_path}")

        self.config.read(config_path)

        # --- Read Stereo Camera Config ---
        try:
            self.left_cam_device = self.config['StereoCamera']['left_image']
            self.right_cam_device = self.config['StereoCamera']['right_image']
            # Handle potential variations in device paths (e.g., /dev/videoX vs integer index)
            try:
                self.left_cam_index = int(self.left_cam_device)
            except ValueError:
                 # Assume it's a path like /dev/videoX, pass directly to OpenCV
                 self.left_cam_index = self.left_cam_device
            try:
                self.right_cam_index = int(self.right_cam_device)
            except ValueError:
                 self.right_cam_index = self.right_cam_device

            self.upside_down = self.config.getboolean('StereoCamera', 'upside_down', fallback=False)
            print(f"  Left Cam Device/Index: {self.left_cam_device} ({self.left_cam_index})")
            print(f"  Right Cam Device/Index: {self.right_cam_device} ({self.right_cam_index})")
            print(f"  Upside Down: {self.upside_down}")
        except KeyError as e:
            raise KeyError(f"Missing key in [StereoCamera] section of '{config_path}': {e}")

        # --- Read Storage Config ---
        try:
            self.scan_folder_path = self.config['Storage']['scan_folder_path']
            if not self.scan_folder_path:
                raise ValueError("scan_folder_path cannot be empty in [Storage] section.")
            print(f"  Scan Folder Path: {self.scan_folder_path}")
        except KeyError as e:
            raise KeyError(f"Missing key in [Storage] section of '{config_path}': {e}")

        self.cap_left: Optional[cv2.VideoCapture] = None
        self.cap_right: Optional[cv2.VideoCapture] = None

    def _initialize_cameras(self) -> bool:
        """Initializes OpenCV VideoCapture objects for both cameras."""
        print("Attempting to initialize cameras...")
        try:
            # Use CAP_V4L2 backend explicitly for Linux devices if needed
            # self.cap_left = cv2.VideoCapture(self.left_cam_index, cv2.CAP_V4L2)
            # self.cap_right = cv2.VideoCapture(self.right_cam_index, cv2.CAP_V4L2)
            self.cap_left = cv2.VideoCapture(self.left_cam_index)
            self.cap_right = cv2.VideoCapture(self.right_cam_index)

            if not self.cap_left.isOpened():
                print(f"ERROR: Could not open left camera: {self.left_cam_device}")
                self.release_cameras()
                return False
            if not self.cap_right.isOpened():
                print(f"ERROR: Could not open right camera: {self.right_cam_device}")
                self.release_cameras()
                return False

            print("Cameras initialized successfully.")
            # Optional: Set camera properties (resolution, FPS) here if needed
            # self.cap_left.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            # self.cap_left.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            # self.cap_right.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            # self.cap_right.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            return True
        except Exception as e:
            print(f"ERROR: Exception during camera initialization: {e}")
            self.release_cameras()
            return False

    def release_cameras(self):
        """Releases the OpenCV VideoCapture objects."""
        if self.cap_left and self.cap_left.isOpened():
            self.cap_left.release()
            print("Left camera released.")
        if self.cap_right and self.cap_right.isOpened():
            self.cap_right.release()
            print("Right camera released.")
        self.cap_left = None
        self.cap_right = None

    def capture_and_save_pair(self, session_id: int = 0) -> Optional[Dict[str, Any]]:
        """
        Captures one stereo image pair, processes according to upside_down flag,
        saves the images, and returns info needed for database insertion.

        Args:
            session_id: The ID of the current scan session (for database linking).

        Returns:
            A dictionary containing information for the StereoImagePair database table,
            or None if capture failed.
            Keys: 'session_id', 'pi_timestamp', 'sensor_timestamp',
                  'left_image_path', 'right_image_path'
        """
        if not self._initialize_cameras():
            return None

        try:
            # Allow cameras some time to stabilize (optional)
            # time.sleep(0.1)

            # --- Capture Frames ---
            # Read multiple frames to ensure buffers are cleared (optional)
            # for _ in range(3):
            #    self.cap_left.grab()
            #    self.cap_right.grab()

            ret_left, frame_left_raw = self.cap_left.read()
            # Capture timestamp as close as possible to frame reads
            pi_timestamp_ns = time.time_ns()
            ret_right, frame_right_raw = self.cap_right.read()

            if not ret_left:
                print("ERROR: Failed to read frame from left camera.")
                return None
            if not ret_right:
                print("ERROR: Failed to read frame from right camera.")
                return None

            print("Frames captured successfully.")

            # --- Process Upside Down Logic ---
            if self.upside_down:
                print("Processing upside_down flag: Rotating and Swapping images.")
                # Rotate 180 degrees
                frame_left_processed = cv2.rotate(frame_left_raw, cv2.ROTATE_180)
                frame_right_processed = cv2.rotate(frame_right_raw, cv2.ROTATE_180)
                # Swap: The physical left camera becomes the logical right image
                final_left_image = frame_right_processed
                final_right_image = frame_left_processed
            else:
                # Use frames as they are
                final_left_image = frame_left_raw
                final_right_image = frame_right_raw

            # --- Prepare Paths and Save ---
            now = datetime.datetime.now()
            timestamp_str = now.strftime("%Y%m%d_%H%M%S_%f")[:-3] # YYYYMMDD_HHMMSS_ms
            session_folder_name = f"session_{session_id:04d}" # e.g., session_0000
            image_subfolder = os.path.join(self.scan_folder_path, session_folder_name, "stereo_images", timestamp_str)

            # Create directories if they don't exist
            try:
                os.makedirs(image_subfolder, exist_ok=True)
            except OSError as e:
                print(f"ERROR: Could not create directory {image_subfolder}: {e}")
                return None

            # Define relative paths for database storage
            relative_base = os.path.join(session_folder_name, "stereo_images", timestamp_str)
            left_filename = f"left_{timestamp_str}.png"
            right_filename = f"right_{timestamp_str}.png"
            relative_left_path = os.path.join(relative_base, left_filename)
            relative_right_path = os.path.join(relative_base, right_filename)

            # Define absolute paths for saving
            abs_left_path = os.path.join(image_subfolder, left_filename)
            abs_right_path = os.path.join(image_subfolder, right_filename)

            # Save images
            print(f"Saving left image to: {abs_left_path}")
            save_success_left = cv2.imwrite(abs_left_path, final_left_image)
            print(f"Saving right image to: {abs_right_path}")
            save_success_right = cv2.imwrite(abs_right_path, final_right_image)

            if not save_success_left or not save_success_right:
                print("ERROR: Failed to save one or both images.")
                # Optionally attempt to clean up partially saved files
                if os.path.exists(abs_left_path): os.remove(abs_left_path)
                if os.path.exists(abs_right_path): os.remove(abs_right_path)
                return None

            print("Images saved successfully.")

            # --- Prepare Database Info ---
            # OpenCV doesn't easily provide a reliable sensor timestamp matching the frame.
            # Set to None or 0, or investigate camera-specific APIs if needed.
            sensor_timestamp = None

            db_info = {
                'session_id': session_id,
                'pi_timestamp': pi_timestamp_ns, # Use nanoseconds for precision
                'sensor_timestamp': sensor_timestamp,
                'left_image_path': relative_left_path, # Store relative path
                'right_image_path': relative_right_path # Store relative path
            }
            print(f"Prepared DB Info: {db_info}")
            return db_info

        except cv2.error as e:
            print(f"ERROR: OpenCV error during capture/processing: {e}")
            return None
        except Exception as e:
            print(f"ERROR: Unexpected error during capture/saving: {e}")
            import traceback
            traceback.print_exc()
            return None
        finally:
            # Ensure cameras are always released
            self.release_cameras()


# --- Example Usage ---
if __name__ == "__main__":
    CONFIG_FILE = 'config.ini' # Make sure this file exists and is configured

    # Create a dummy config file for testing if it doesn't exist
    if not os.path.exists(CONFIG_FILE):
        print(f"WARNING: {CONFIG_FILE} not found. Creating a dummy one for testing.")
        print("         >>> Please edit it with your actual camera devices and paths! <<<")
        dummy_config = configparser.ConfigParser()
        dummy_config['IMU'] = {'port': '/dev/ttyACM0', 'baud_rate': '115200'}
        dummy_config['LiDAR'] = {'port': '/dev/ttyUSB0', 'baud_rate': '230400', 'enabled': 'true'}
        dummy_config['Calibration'] = {'still_delay_seconds': '5', 'rotation_delay_seconds': '10'}
        dummy_config['StereoCamera'] = {'left_image': '0', 'right_image': '2', 'upside_down': 'false'} # Use indices 0 and 2 as example
        dummy_config['Storage'] = {'scan_folder_path': './scan_data_output'} # Save to current dir subfolder
        with open(CONFIG_FILE, 'w') as configfile:
            dummy_config.write(configfile)
        # Create the dummy output directory
        if not os.path.exists('./scan_data_output'):
             os.makedirs('./scan_data_output')


    try:
        stereo_system = StereoCameraSystem(config_path=CONFIG_FILE)

        print("\nAttempting to capture one stereo pair...")
        # Assume session_id is 1 for this example
        capture_result = stereo_system.capture_and_save_pair(session_id=1)

        if capture_result:
            print("\n--- Capture Successful ---")
            print("Database information prepared:")
            for key, value in capture_result.items():
                print(f"  {key}: {value}")
            print("\nCorresponding images saved under:", stereo_system.scan_folder_path)
            # You would now typically insert this 'capture_result' dictionary
            # into your SQLite database's StereoImagePair table.
        else:
            print("\n--- Capture Failed ---")
            print("Please check camera connections, permissions, and config file settings.")

    except (FileNotFoundError, KeyError, ValueError) as e:
        print(f"\nERROR: Configuration Error - {e}")
    except Exception as e:
        print(f"\nERROR: An unexpected error occurred: {e}")
        import traceback
        traceback.print_exc()

    print("\nScript finished.")

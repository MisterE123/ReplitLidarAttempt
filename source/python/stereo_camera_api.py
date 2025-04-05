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
            try: self.left_cam_index = int(self.left_cam_device)
            except ValueError: self.left_cam_index = self.left_cam_device
            try: self.right_cam_index = int(self.right_cam_device)
            except ValueError: self.right_cam_index = self.right_cam_device
            self.upside_down = self.config.getboolean('StereoCamera', 'upside_down', fallback=False)
            print(f"  Left Cam Device/Index: {self.left_cam_device} ({self.left_cam_index})")
            print(f"  Right Cam Device/Index: {self.right_cam_device} ({self.right_cam_index})")
            print(f"  Upside Down: {self.upside_down}")
        except KeyError as e:
            raise KeyError(f"Missing key in [StereoCamera] section of '{config_path}': {e}")

        # Storage path is no longer read here, it's determined by DataManager per session

        self.cap_left: Optional[cv2.VideoCapture] = None
        self.cap_right: Optional[cv2.VideoCapture] = None

    def _initialize_cameras(self) -> bool:
        """Initializes OpenCV VideoCapture objects for both cameras."""
        # print("Attempting to initialize cameras...") # Reduce noise
        try:
            self.cap_left = cv2.VideoCapture(self.left_cam_index)
            self.cap_right = cv2.VideoCapture(self.right_cam_index)
            if not self.cap_left.isOpened():
                print(f"ERROR: Could not open left camera: {self.left_cam_device}")
                self.release_cameras(); return False
            if not self.cap_right.isOpened():
                print(f"ERROR: Could not open right camera: {self.right_cam_device}")
                self.release_cameras(); return False
            # print("Cameras initialized successfully.") # Reduce noise
            return True
        except Exception as e:
            print(f"ERROR: Exception during camera initialization: {e}")
            self.release_cameras(); return False

    def release_cameras(self):
        """Releases the OpenCV VideoCapture objects."""
        if self.cap_left and self.cap_left.isOpened(): self.cap_left.release()
        if self.cap_right and self.cap_right.isOpened(): self.cap_right.release()
        # print("Cameras released.") # Reduce noise
        self.cap_left = None
        self.cap_right = None

    def capture_and_save_pair(self, session_id: int, session_folder_path: str) -> Optional[Dict[str, Any]]:
        """
        Captures one stereo image pair, processes, saves to the specific session folder,
        and returns info needed for database insertion.

        Args:
            session_id: The ID of the current scan session.
            session_folder_path: The full path to the directory for the current session's data.

        Returns:
            A dictionary containing information for the StereoImagePair database table,
            or None if capture failed.
        """
        if not session_folder_path or not os.path.isdir(session_folder_path):
             print(f"ERROR: Invalid session_folder_path provided: {session_folder_path}")
             return None

        if not self._initialize_cameras():
            return None

        try:
            ret_left, frame_left_raw = self.cap_left.read()
            pi_timestamp_ns = time.time_ns()
            ret_right, frame_right_raw = self.cap_right.read()

            if not ret_left or not ret_right:
                print("ERROR: Failed to read frame from one or both cameras.")
                return None
            # print("Frames captured successfully.") # Reduce noise

            if self.upside_down:
                # print("Processing upside_down flag: Rotating and Swapping images.") # Reduce noise
                frame_left_processed = cv2.rotate(frame_left_raw, cv2.ROTATE_180)
                frame_right_processed = cv2.rotate(frame_right_raw, cv2.ROTATE_180)
                final_left_image = frame_right_processed
                final_right_image = frame_left_processed
            else:
                final_left_image = frame_left_raw
                final_right_image = frame_right_raw

            # --- Prepare Paths and Save ---
            now = datetime.datetime.fromtimestamp(pi_timestamp_ns / 1e9) # Use capture time for timestamp
            timestamp_str = now.strftime("%Y%m%d_%H%M%S_%f")[:-3] # YYYYMMDD_HHMMSS_ms

            # Define image subfolder within the session path
            image_subfolder = os.path.join(session_folder_path, "stereo_images")
            os.makedirs(image_subfolder, exist_ok=True) # Ensure it exists

            # Define filenames
            left_filename = f"left_{timestamp_str}.png"
            right_filename = f"right_{timestamp_str}.png"

            # Define relative paths for database storage (relative to base scan folder)
            # Need the session folder *name* relative to the base scan folder
            session_folder_name = os.path.basename(session_folder_path)
            relative_base = os.path.join(session_folder_name, "stereo_images")
            relative_left_path = os.path.join(relative_base, left_filename)
            relative_right_path = os.path.join(relative_base, right_filename)

            # Define absolute paths for saving
            abs_left_path = os.path.join(image_subfolder, left_filename)
            abs_right_path = os.path.join(image_subfolder, right_filename)

            # Save images
            # print(f"Saving left image to: {abs_left_path}") # Reduce noise
            save_success_left = cv2.imwrite(abs_left_path, final_left_image)
            # print(f"Saving right image to: {abs_right_path}") # Reduce noise
            save_success_right = cv2.imwrite(abs_right_path, final_right_image)

            if not save_success_left or not save_success_right:
                print("ERROR: Failed to save one or both images.")
                if os.path.exists(abs_left_path): os.remove(abs_left_path)
                if os.path.exists(abs_right_path): os.remove(abs_right_path)
                return None
            # print("Images saved successfully.") # Reduce noise

            sensor_timestamp = None # OpenCV doesn't easily provide this

            db_info = {
                'session_id': session_id,
                'pi_timestamp': pi_timestamp_ns,
                'sensor_timestamp': sensor_timestamp,
                'left_image_path': relative_left_path, # Store relative path
                'right_image_path': relative_right_path # Store relative path
            }
            # print(f"Prepared DB Info: {db_info}") # Reduce noise
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
            self.release_cameras() # Ensure cameras are always released

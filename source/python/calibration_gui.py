import tkinter as tk
from tkinter import ttk # Themed widgets
from tkinter import messagebox
import time
import sys
import threading # To run IMU init in background
import json # To potentially format sync results display
from typing import Dict, Any, Optional # <--- Added import for type hinting

# Import the updated API and specific exceptions
# Make sure imu_api_v3.py (renamed from imu_api.py) and lidar_api.py are accessible
try:
    # Assuming imu_api_v3.py contains the updated IMUAPI class
    from imu_api import IMUAPI, IMUCommunicationError, IMUCalibrationError, IMUTimeoutError
    # Lidar errors might also be relevant if init fails within IMUAPI
    try:
        from lidar_api import LidarCommunicationError
    except ImportError:
        LidarCommunicationError = Exception # Define dummy if lidar_api not found
except ImportError:
    messagebox.showerror("Error", "Could not find imu_api.py. Make sure it's in the Python path.")
    sys.exit(1)
except FileNotFoundError as e:
     messagebox.showerror("Config Error", f"Could not find config.ini: {e}")
     sys.exit(1)
except (KeyError, ValueError) as e:
     messagebox.showerror("Config Error", f"Error reading config.ini: {e}")
     sys.exit(1)


# Define states (though managed differently in Tkinter)
STATE_MAIN_MENU = 0
STATE_CALIBRATING = 1

class CalibrationGUI_Tk:
    def __init__(self, root):
        self.root = root
        self.root.title("IMU & LiDAR Calibration Tool (Tkinter)")
        # Make window slightly larger to accommodate more status info
        self.root.geometry("750x550") # Increased height slightly for new button
        self.root.protocol("WM_DELETE_WINDOW", self._on_closing) # Handle window close

        self.imu_api: Optional[IMUAPI] = None # Use updated IMUAPI
        self.api_init_error = None
        self.current_state = STATE_MAIN_MENU # To control button states

        # --- Calibration Status Flags ---
        self.clocks_calibrated = False
        self.still_calibrated = False
        self.motion_calibrated = False
        # self.is_calibrated is redundant now, remove its usage

        # --- Style ---
        self.style = ttk.Style()
        available_themes = self.style.theme_names()
        print(f"Available themes: {available_themes}")
        if 'clam' in available_themes: self.style.theme_use('clam')
        elif 'alt' in available_themes: self.style.theme_use('alt')
        self.style.configure('TButton', padding=6)
        self.style.configure('Status.TLabel', foreground='grey') # Style for detailed status
        self.style.configure('Error.TLabel', foreground='red')
        self.style.configure('Success.TLabel', foreground='green')
        self.style.configure('Finish.TButton', foreground='blue', font=('Helvetica', 10, 'bold')) # Style for finish button

        # --- Main Frames ---
        self.main_menu_frame = ttk.Frame(root, padding="10")
        self.calibration_frame = ttk.Frame(root, padding="10")

        # --- Status Display Area ---
        # Use a Text widget for more detailed status updates
        self.status_frame = ttk.LabelFrame(root, text="Status Log", padding="5")
        self.status_text = tk.Text(self.status_frame, height=10, width=80, wrap=tk.WORD, state=tk.DISABLED, relief=tk.SUNKEN, borderwidth=1) # Increased height
        self.status_scrollbar = ttk.Scrollbar(self.status_frame, orient=tk.VERTICAL, command=self.status_text.yview)
        self.status_text.config(yscrollcommand=self.status_scrollbar.set)
        self.status_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.status_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.status_frame.pack(side=tk.BOTTOM, fill=tk.X, padx=5, pady=5)

        # --- Set Initial Status ---
        self.log_status("Initializing...", tag='info') # Initial status

        # --- Setup UI Elements ---
        self._setup_main_menu_frame()
        self._setup_calibration_frame()

        # --- Start API Initialization in Background ---
        self.log_status("Connecting to IMU and LiDAR (based on config.ini)...", tag='info')
        self.init_thread = threading.Thread(target=self._initialize_apis, daemon=True)
        self.init_thread.start()
        # Check periodically if the init thread is done
        self.root.after(100, self._check_api_init)

    def log_status(self, message: str, tag: str = 'info'):
        """Appends a message to the status Text widget with optional styling."""
        print(f"GUI Log ({tag}): {message}") # Also print to console
        if hasattr(self, 'status_text'):
            self.status_text.config(state=tk.NORMAL)
            # Define tags if they don't exist yet (safer)
            if tag not in self.status_text.tag_names():
                 if tag == 'error': self.status_text.tag_config('error', foreground='red')
                 elif tag == 'success': self.status_text.tag_config('success', foreground='green')
                 elif tag == 'warning': self.status_text.tag_config('warning', foreground='orange')
                 else: self.status_text.tag_config('info', foreground='black')

            self.status_text.insert(tk.END, f"[{tag.upper()}] {message}\n", tag)
            self.status_text.config(state=tk.DISABLED)
            self.status_text.see(tk.END) # Scroll to the bottom

        # self.root.update_idletasks() # Force update if needed

    def _initialize_apis(self):
        """Runs in a separate thread to avoid blocking the GUI."""
        print("DEBUG: Background thread: Attempting to initialize IMUAPI (v3)...")
        try:
            # IMUAPI v3 now handles internal LiDAR init based on config
            self.imu_api = IMUAPI() # Reads config internally
            # Check status of internal LiDAR if applicable
            if self.imu_api.lidar:
                 print("DEBUG: Background thread: IMUAPI initialization successful (LiDAR active).")
            else:
                 print("DEBUG: Background thread: IMUAPI initialization successful (LiDAR inactive/disabled/failed).")

        except (FileNotFoundError, KeyError, ValueError) as e:
            self.imu_api = None
            self.api_init_error = f"Config Error: {str(e)}"
            print(f"DEBUG: Background thread: Config Error: {str(e)}")
        except (IMUCommunicationError, LidarCommunicationError) as e:
             # Catch errors from IMU or internal LiDAR connection attempts
             self.imu_api = None
             self.api_init_error = f"Connection Error: {str(e)}"
             print(f"DEBUG: Background thread: Connection Error: {str(e)}")
        except Exception as e:
            self.imu_api = None
            self.api_init_error = f"Unexpected Init Error: {str(e)}"
            print(f"DEBUG: Background thread: Unexpected Initialization Error: {str(e)}")
            import traceback
            traceback.print_exc()


    def _check_api_init(self):
        """Checks the result of the API initialization thread."""
        if self.init_thread.is_alive():
            # Still running, check again later
            self.root.after(100, self._check_api_init)
        else:
            # Thread finished
            if self.imu_api:
                status_msg = "IMU API Ready."
                if self.imu_api.lidar:
                    status_msg += " LiDAR Active."
                else:
                    status_msg += " LiDAR Inactive/Disabled."
                self.log_status(status_msg, tag='success')
                # Enable the main calibrate button now that APIs are ready
                if hasattr(self, 'calibrate_button'):
                    self.calibrate_button.config(state=tk.NORMAL)
                self._show_frame(self.main_menu_frame)
            else:
                # Update status with the error found in the background thread
                self.log_status(self.api_init_error or "Failed to initialize APIs.", tag='error')
                self.log_status("Please check config.ini and sensor connections.", tag='error')
                if hasattr(self, 'calibrate_button'):
                    self.calibrate_button.config(state=tk.DISABLED)
                self._show_frame(self.main_menu_frame) # Show main menu even on error

    def _setup_main_menu_frame(self):
        """Creates widgets for the main menu."""
        for widget in self.main_menu_frame.winfo_children(): widget.destroy()

        label = ttk.Label(self.main_menu_frame, text="Main Menu", font=("Helvetica", 16))
        label.pack(pady=20)

        # Calibrate Button - State determined by _check_api_init
        self.calibrate_button = ttk.Button(
            self.main_menu_frame,
            text="Start Calibration", # Renamed slightly
            command=self._go_to_calibration_steps, # Changed target method
            state=tk.DISABLED # Start disabled, enabled later if APIs init
        )
        self.calibrate_button.pack(pady=30, ipadx=20, ipady=10) # Make button larger

        quit_button = ttk.Button(self.main_menu_frame, text="Quit", command=self._on_closing)
        quit_button.pack(pady=10)


    def _setup_calibration_frame(self):
        """Creates widgets for the calibration steps menu."""
        for widget in self.calibration_frame.winfo_children(): widget.destroy()

        label = ttk.Label(self.calibration_frame, text="Calibration Steps", font=("Helvetica", 16))
        label.pack(pady=10) # Reduced padding

        # Step 1: Clock Sync (Now combined)
        self.clock_sync_button = ttk.Button(
            self.calibration_frame,
            text="1. Calibrate Clocks (IMU + LiDAR)",
            command=self._do_clock_calibration_tk,
            state=tk.DISABLED # Disabled until main calibrate button clicked
        )
        self.clock_sync_button.pack(pady=5, fill=tk.X, padx=50) # Reduced padding

        # Step 2: Still Calibration
        self.still_cal_button = ttk.Button(
            self.calibration_frame,
            text="2. Calibrate Still (Gravity+Gyro)",
            command=self._do_still_calibration_tk,
            state=tk.DISABLED # Disabled until clocks sync'd
        )
        self.still_cal_button.pack(pady=5, fill=tk.X, padx=50) # Reduced padding

        # Step 3: Motion Calibration
        self.motion_cal_button = ttk.Button(
            self.calibration_frame,
            text="3. Calibrate Motion (Magnetometer)",
            command=self._do_motion_calibration_tk,
            state=tk.DISABLED # Disabled until clocks sync'd
        )
        self.motion_cal_button.pack(pady=5, fill=tk.X, padx=50) # Reduced padding

        # --- Separator ---
        ttk.Separator(self.calibration_frame, orient='horizontal').pack(fill='x', padx=40, pady=15)

        # Original Done Button (for exiting early)
        self.done_button = ttk.Button(
            self.calibration_frame,
            text="Done (Back to Main Menu - Incomplete)",
            command=self._go_to_main_menu,
            state=tk.DISABLED # Tied to clock button state initially
        )
        self.done_button.pack(pady=5, fill=tk.X, padx=50)

        # New Finish Button (only enabled when all steps complete)
        self.finish_calibration_button = ttk.Button(
            self.calibration_frame,
            text="FINISH CALIBRATION & Return",
            command=self._go_to_main_menu, # Also returns to main menu
            state=tk.DISABLED, # Initially disabled
            style='Finish.TButton' # Apply special style
        )
        self.finish_calibration_button.pack(pady=10, fill=tk.X, padx=50)


    def _set_calibration_buttons_state(self, clock_state=tk.DISABLED, sensor_state=tk.DISABLED):
        """Enable/disable calibration step buttons."""
        if hasattr(self, 'clock_sync_button'): self.clock_sync_button.config(state=clock_state)
        if hasattr(self, 'still_cal_button'): self.still_cal_button.config(state=sensor_state)
        if hasattr(self, 'motion_cal_button'): self.motion_cal_button.config(state=sensor_state)
        # Original Done button follows clock state
        if hasattr(self, 'done_button'): self.done_button.config(state=clock_state)
        # Ensure Finish button is disabled unless specifically enabled later
        if hasattr(self, 'finish_calibration_button'):
             # Only enable finish button if explicitly told AND all steps are done
             # This function is mostly for disabling/initial enabling,
             # _update_finish_button_state handles the final enabling.
             if clock_state == tk.DISABLED or sensor_state == tk.DISABLED:
                  self.finish_calibration_button.config(state=tk.DISABLED)
             # If clock/sensor states are NORMAL, _update_finish_button_state will handle it


    def _update_finish_button_state(self):
        """Checks if all calibrations are done and enables the finish button."""
        all_done = self.clocks_calibrated and self.still_calibrated and self.motion_calibrated
        if hasattr(self, 'finish_calibration_button'):
            new_state = tk.NORMAL if all_done else tk.DISABLED
            self.finish_calibration_button.config(state=new_state)
            if all_done:
                 self.log_status("All calibration steps complete! Ready to finish.", tag='success')
                 # Optional: Disable the step buttons once all are done?
                 # self._set_calibration_buttons_state(clock_state=tk.DISABLED, sensor_state=tk.DISABLED)
                 # self.done_button.config(state=tk.DISABLED) # Disable early exit?


    def _show_frame(self, frame_to_show):
        """Hides other frames and shows the specified frame."""
        self.main_menu_frame.pack_forget()
        self.calibration_frame.pack_forget()
        frame_to_show.pack(fill=tk.BOTH, expand=True)

    def _go_to_calibration_steps(self):
        """Callback for the main 'Start Calibration' button."""
        if not self.imu_api:
            self.log_status("Error: APIs not initialized.", tag='error')
            messagebox.showerror("Error", "APIs not initialized. Cannot calibrate.")
            return

        self.log_status("Navigating to calibration steps.", tag='info')
        self.current_state = STATE_CALIBRATING
        # Reset flags when entering calibration screen
        self.clocks_calibrated = False
        self.still_calibrated = False
        self.motion_calibrated = False
        # Enable the clock sync button, disable others initially
        self._set_calibration_buttons_state(clock_state=tk.NORMAL, sensor_state=tk.DISABLED)
        self._update_finish_button_state() # Ensure finish button is disabled
        self._show_frame(self.calibration_frame)
        self.log_status("Ready for Clock Calibration.", tag='info')

    # --- Clock Calibration ---
    def _do_clock_calibration_tk(self):
        """Handles the 'Calibrate Clocks' button click."""
        if not self.imu_api:
            self.log_status("Error: APIs not initialized.", tag='error')
            messagebox.showerror("Error", "APIs not initialized.")
            return

        self.log_status("Attempting clock synchronization (IMU + LiDAR)...", tag='info')
        self._set_calibration_buttons_state(clock_state=tk.DISABLED, sensor_state=tk.DISABLED) # Disable all during attempt
        self._update_finish_button_state() # Ensure finish button is disabled

        # Run clock calibration in a separate thread
        cal_thread = threading.Thread(target=self._perform_clock_calibration, daemon=True)
        cal_thread.start()

    def _perform_clock_calibration(self):
        """Performs clock calibration (called from _do_clock_calibration_tk thread)."""
        try:
            # Ensure imu_api exists before calling method
            if self.imu_api:
                sync_result = self.imu_api.calibrate_clocks()
                # Schedule GUI updates back on the main thread
                self.root.after(0, self._clock_calibration_finished, sync_result)
            else:
                 # Should not happen if button is enabled, but handle defensively
                 result = {'status': 'Failed', 'message': "IMU API instance not available."}
                 self.root.after(0, self._clock_calibration_finished, result)

        except (IMUCommunicationError, IMUTimeoutError, LidarCommunicationError) as e:
             # Catch errors raised directly by the API call itself
             result = {'status': 'Failed', 'message': f"Error during clock calibration call: {e}"}
             self.root.after(0, self._clock_calibration_finished, result)
        except Exception as e:
            # Catch unexpected errors
            result = {'status': 'Failed', 'message': f"Unexpected error during clock calibration: {e}"}
            self.root.after(0, self._clock_calibration_finished, result)
            import traceback
            traceback.print_exc()

    # Note: Added type hint import 'from typing import Dict, Any, Optional' at the top
    def _clock_calibration_finished(self, result: Dict[str, Any]):
        """GUI update after clock calibration attempt (runs in main thread)."""
        status = result.get('status', 'Failed')
        message = result.get('message', 'Unknown error.')

        # Determine tag based on status for logging
        log_tag = 'info'
        if status == 'Failed':
             log_tag = 'error'
             self.clocks_calibrated = False # Ensure flag is false on failure
        elif status == 'Success':
             log_tag = 'success'
             self.clocks_calibrated = True # Set flag on success
        elif status.startswith('Partial'):
             log_tag = 'warning'
             self.clocks_calibrated = True # Set flag even on partial success (IMU part worked)

        self.log_status(f"Clock Sync Status: {status}", tag=log_tag)
        self.log_status(f"Details: {message}", tag='info') # Keep details as info

        if status != 'Failed':
            # Display detailed results
            imu_pi_ns = result.get('pi_sys_time_for_imu_cal_ns')
            imu_sens_us = result.get('imu_sensor_timestamp_at_cal_us')
            lid_pi_ns = result.get('pi_sys_time_anchor_lidar_ns')
            lid_sens_s = result.get('lidar_sensor_timestamp_anchor_s')

            # Format timestamps for readability if they exist
            imu_pi_str = f"{imu_pi_ns:,}" if imu_pi_ns is not None else "N/A"
            imu_sens_str = f"{imu_sens_us:,}" if imu_sens_us is not None else "N/A"
            lid_pi_str = f"{lid_pi_ns:,}" if lid_pi_ns is not None else "N/A"
            lid_sens_str = f"{lid_sens_s:.3f}" if lid_sens_s is not None else "N/A"

            self.log_status(f"  IMU Pi Time (ns):   {imu_pi_str}", tag='info')
            self.log_status(f"  IMU Sensor Time (us): {imu_sens_str}", tag='info')
            if status != 'Partial (No LiDAR API)': # Only show LiDAR details if it was attempted
                 self.log_status(f"  LiDAR Pi Time (ns):   {lid_pi_str}", tag='info')
                 self.log_status(f"  LiDAR Sensor Time (s):   {lid_sens_str}", tag='info')
                 if lid_pi_ns is None and status != 'Partial (No LiDAR Sync)': # Check if sync failed specifically
                      self.log_status("  (LiDAR sync data not captured in window)", tag='warning')

            # Enable sensor calibration buttons
            self.log_status("Clock calibration finished. Ready for sensor calibration.", tag='success')
            self._set_calibration_buttons_state(clock_state=tk.NORMAL, sensor_state=tk.NORMAL)
            self._update_finish_button_state() # Check if all steps are now complete

        else:
            self.clocks_calibrated = False
            self.log_status(f"Clock calibration failed. Cannot proceed with sensor calibration.", tag='error')
            messagebox.showerror("Clock Calibration Failed", message)
            # Re-enable clock sync button to allow retry, keep others disabled
            self._set_calibration_buttons_state(clock_state=tk.NORMAL, sensor_state=tk.DISABLED)
            self._update_finish_button_state() # Ensure finish button is disabled


    # --- Sensor Calibration Methods (Mostly similar, check clock calibration first) ---

    def _do_still_calibration_tk(self):
        """Handles the 'Still Calibration' button click."""
        if not self.imu_api: self.log_status("Error: APIs not initialized.", tag='error'); return
        if not self.clocks_calibrated:
             self.log_status("Clock calibration must be performed successfully first.", tag='warning')
             messagebox.showwarning("Prerequisite", "Please perform clock calibration successfully before sensor calibration.")
             return

        try:
            delay_sec = self.imu_api.still_delay
            delay_ms = int(delay_sec * 1000)
            instruction = f"Place device flat and still. Waiting {delay_sec:.1f}s..."
            self.log_status("Preparing for Still Calibration.", tag='info')
            self.log_status(instruction, tag='info')
            self._set_calibration_buttons_state(clock_state=tk.DISABLED, sensor_state=tk.DISABLED) # Disable all during cal
            self._update_finish_button_state() # Ensure finish button is disabled

            # Schedule the actual calibration after the delay
            self.root.after(delay_ms, self._schedule_still_calibration_thread)

        except AttributeError:
            self.log_status("Error: Could not find delay settings.", tag='error')
            messagebox.showerror("Error", "Could not find delay settings in IMU API/config.")
            self._set_calibration_buttons_state(clock_state=tk.NORMAL, sensor_state=tk.NORMAL) # Re-enable on error
            self._update_finish_button_state() # Update finish button state based on flags
        except Exception as e:
            self.log_status(f"Unexpected Error preparing still calibration: {str(e)}", tag='error')
            messagebox.showerror("Error", f"Unexpected error: {str(e)}")
            self._set_calibration_buttons_state(clock_state=tk.NORMAL, sensor_state=tk.NORMAL)
            self._update_finish_button_state()

    def _schedule_still_calibration_thread(self):
        """Schedules the still calibration to run in a background thread."""
        cal_thread = threading.Thread(target=self._perform_actual_still_calibration, daemon=True)
        cal_thread.start()

    def _perform_actual_still_calibration(self):
        """Called by thread to perform the still calibration steps."""
        success = False
        try:
            # Ensure imu_api exists
            if not self.imu_api: raise Exception("IMU API not available in thread.")

            # Gravity Calibration
            self.root.after(0, lambda: self.log_status("Calibrating Gravity...", tag='info'))
            self.root.after(0, lambda: self.log_status("Keep device perfectly still.", tag='info'))
            self.imu_api.calibrate_gravity()
            self.root.after(0, lambda: self.log_status("Gravity calibrated.", tag='success'))

            # Gyro Calibration (immediately after gravity)
            self.root.after(0, lambda: self.log_status("Starting Gyro calibration (keep still)...", tag='info'))
            self.imu_api.calibrate_gyro()
            self.root.after(0, lambda: self.log_status("Still Calibration Complete (Gravity & Gyro).", tag='success'))
            success = True

        except (IMUCommunicationError, IMUCalibrationError, IMUTimeoutError) as e:
            self.root.after(0, self._sensor_calibration_failure, "Still", e)
        except Exception as e:
            self.root.after(0, self._sensor_calibration_failure, "Still", e)
            import traceback
            traceback.print_exc()
        finally:
            # Update status flag on main thread
            if success:
                 self.root.after(0, self._mark_still_cal_done)
            # Re-enable buttons after attempt
            self.root.after(0, lambda: self._set_calibration_buttons_state(clock_state=tk.NORMAL, sensor_state=tk.NORMAL))
            # Check if all calibrations are now complete
            self.root.after(0, self._update_finish_button_state)

    def _mark_still_cal_done(self):
         """Sets the still calibration flag (runs in main thread)."""
         self.still_calibrated = True

    def _do_motion_calibration_tk(self):
        """Handles the 'Motion Calibration' button click."""
        if not self.imu_api: self.log_status("Error: APIs not initialized.", tag='error'); return
        if not self.clocks_calibrated:
             self.log_status("Clock calibration must be performed successfully first.", tag='warning')
             messagebox.showwarning("Prerequisite", "Please perform clock calibration successfully before sensor calibration.")
             return

        try:
            delay_sec = self.imu_api.rotation_delay
            delay_ms = int(delay_sec * 1000)
            prep_instruction = f"Prepare to rotate IMU through all axes. Starting in {delay_sec:.0f}s..."
            self.log_status("Preparing for Motion Calibration.", tag='info')
            self.log_status(prep_instruction, tag='info')
            self._set_calibration_buttons_state(clock_state=tk.DISABLED, sensor_state=tk.DISABLED) # Disable all during cal
            self._update_finish_button_state() # Ensure finish button is disabled

            self.root.after(delay_ms, self._schedule_motion_calibration_thread)

        except AttributeError:
            self.log_status("Error: Could not find delay settings.", tag='error')
            messagebox.showerror("Error", "Could not find delay settings in IMU API/config.")
            self._set_calibration_buttons_state(clock_state=tk.NORMAL, sensor_state=tk.NORMAL) # Re-enable on error
            self._update_finish_button_state()
        except Exception as e:
            self.log_status(f"Unexpected Error preparing motion calibration: {str(e)}", tag='error')
            messagebox.showerror("Error", f"Unexpected error: {str(e)}")
            self._set_calibration_buttons_state(clock_state=tk.NORMAL, sensor_state=tk.NORMAL)
            self._update_finish_button_state()

    def _schedule_motion_calibration_thread(self):
        """Schedules the motion calibration to run in a background thread."""
        cal_thread = threading.Thread(target=self._perform_actual_motion_calibration, daemon=True)
        cal_thread.start()

    def _perform_actual_motion_calibration(self):
        """Called by thread to perform the motion calibration step."""
        success = False
        try:
            # Ensure imu_api exists
            if not self.imu_api: raise Exception("IMU API not available in thread.")

            self.root.after(0, lambda: self.log_status("Calibrating Magnetometer...", tag='info'))
            self.root.after(0, lambda: self.log_status("Rotate IMU slowly in all directions for ~10s.", tag='info'))
            self.imu_api.calibrate_mag()
            self.root.after(0, lambda: self.log_status("Motion Calibration Complete (Magnetometer).", tag='success'))
            success = True

        except (IMUCommunicationError, IMUCalibrationError, IMUTimeoutError) as e:
            self.root.after(0, self._sensor_calibration_failure, "Motion", e)
        except Exception as e:
            self.root.after(0, self._sensor_calibration_failure, "Motion", e)
            import traceback
            traceback.print_exc()
        finally:
             # Update status flag on main thread
             if success:
                  self.root.after(0, self._mark_motion_cal_done)
             # Re-enable buttons after attempt
             self.root.after(0, lambda: self._set_calibration_buttons_state(clock_state=tk.NORMAL, sensor_state=tk.NORMAL))
             # Check if all calibrations are now complete
             self.root.after(0, self._update_finish_button_state)

    def _mark_motion_cal_done(self):
         """Sets the motion calibration flag (runs in main thread)."""
         self.motion_calibrated = True

    def _sensor_calibration_failure(self, cal_type: str, error: Exception):
        """GUI update after failed sensor calibration (runs in main thread)."""
        error_str = str(error)
        # Reset corresponding flag on failure
        if cal_type == "Still":
             self.still_calibrated = False
        elif cal_type == "Motion":
             self.motion_calibrated = False

        self.log_status(f"{cal_type} Calibration Error: {error_str}", tag='error')
        self.log_status("Please try again.", tag='info')
        messagebox.showerror(f"{cal_type} Calibration Error", f"{error_str}\nPlease try again.")
        # Update finish button state (it should become disabled if it was enabled)
        self._update_finish_button_state()


    # --- Navigation and Closing ---

    def _go_to_main_menu(self):
        """Callback for the 'Done' or 'Finish' buttons."""
        self.current_state = STATE_MAIN_MENU
        self._show_frame(self.main_menu_frame)
        # Re-enable main calibrate button if APIs are still connected
        if self.imu_api and hasattr(self, 'calibrate_button'):
             self.calibrate_button.config(state=tk.NORMAL)
             status_msg = "IMU API Ready."
             # Check lidar status via imu_api instance
             if self.imu_api.lidar and self.imu_api.lidar._is_connected:
                 status_msg += " LiDAR Active."
             else:
                 status_msg += " LiDAR Inactive."
             self.log_status(f"Returned to Main Menu. {status_msg}", tag='info')
        elif hasattr(self, 'calibrate_button'):
             self.calibrate_button.config(state=tk.DISABLED)
             self.log_status(f"Returned to Main Menu. APIs disconnected or failed.", tag='warning')

        # Reset all calibration flags when returning to main menu
        self.clocks_calibrated = False
        self.still_calibrated = False
        self.motion_calibrated = False
        # Ensure finish button is disabled (handled by _go_to_calibration_steps next time)


    def _on_closing(self):
        """Handles the window close event."""
        print("DEBUG: Window closing...")
        if self.imu_api:
            print("Closing API resources (IMU/LiDAR)...")
            # IMUAPI's close() method now handles internal LiDAR disconnect too
            try:
                self.imu_api.close()
            except Exception as e:
                print(f"Error closing APIs: {e}") # Log error but continue closing GUI
        self.root.destroy() # Close the Tkinter window


# --- Main Execution ---
if __name__ == "__main__":
    print("DEBUG: Creating Tkinter root window...")
    root = tk.Tk() # Keep it simple for now

    print("DEBUG: Creating CalibrationGUI_Tk instance...")
    try:
        app = CalibrationGUI_Tk(root)
        print("DEBUG: Starting Tkinter main loop...")
        root.mainloop()
    except Exception as e:
         # Catch initialization errors that might prevent mainloop
         print(f"\nFATAL ERROR during GUI initialization: {e}")
         messagebox.showerror("Fatal Error", f"Failed to initialize GUI: {e}\n\nPlease check configuration and logs.")
         # Attempt to clean up if root exists
         try:
              if root: root.destroy()
         except:
              pass
         import traceback
         traceback.print_exc()
    finally:
         print("DEBUG: Application finished.")

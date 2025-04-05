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
# STATE_CALIBRATING is less relevant now with sequential flow, but keep for potential use
STATE_MAIN_MENU = 0
STATE_CALIBRATING = 1

class CalibrationGUI_Tk:
    def __init__(self, root):
        self.root = root
        self.root.title("IMU & LiDAR Calibration Tool (Sequential)")
        self.root.geometry("750x550")
        self.root.protocol("WM_DELETE_WINDOW", self._on_closing)

        self.imu_api: Optional[IMUAPI] = None
        self.api_init_error = None
        self.current_state = STATE_MAIN_MENU

        # --- Calibration Status Flags ---
        # These are less critical for flow control now, but good for tracking
        self.clocks_calibrated = False
        self.still_calibrated = False
        self.motion_calibrated = False

        # --- Style ---
        self.style = ttk.Style()
        available_themes = self.style.theme_names()
        print(f"Available themes: {available_themes}")
        if 'clam' in available_themes: self.style.theme_use('clam')
        elif 'alt' in available_themes: self.style.theme_use('alt')
        self.style.configure('TButton', padding=6)
        self.style.configure('Finish.TButton', foreground='blue', font=('Helvetica', 10, 'bold'))

        # --- Main Frames ---
        self.main_menu_frame = ttk.Frame(root, padding="10")
        self.calibration_frame = ttk.Frame(root, padding="10")

        # --- Status Display Area ---
        self.status_frame = ttk.LabelFrame(root, text="Status Log", padding="5")
        self.status_text = tk.Text(self.status_frame, height=10, width=80, wrap=tk.WORD, state=tk.DISABLED, relief=tk.SUNKEN, borderwidth=1)
        self.status_scrollbar = ttk.Scrollbar(self.status_frame, orient=tk.VERTICAL, command=self.status_text.yview)
        self.status_text.config(yscrollcommand=self.status_scrollbar.set)
        self.status_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.status_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.status_frame.pack(side=tk.BOTTOM, fill=tk.X, padx=5, pady=5)

        # --- Set Initial Status ---
        self.log_status("Initializing...", tag='info')

        # --- Setup UI Elements ---
        self._setup_main_menu_frame()
        self._setup_calibration_frame() # Creates buttons but doesn't show them yet

        # --- Start API Initialization in Background ---
        self.log_status("Connecting to IMU and LiDAR (based on config.ini)...", tag='info')
        self.init_thread = threading.Thread(target=self._initialize_apis, daemon=True)
        self.init_thread.start()
        self.root.after(100, self._check_api_init)

    # --- Logging ---
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

    # --- Initialization ---
    def _initialize_apis(self):
        """Runs in a separate thread to avoid blocking the GUI."""
        print("DEBUG: Background thread: Attempting to initialize IMUAPI (v3)...")
        try:
            self.imu_api = IMUAPI()
            if self.imu_api.lidar: print("DEBUG: Background thread: IMUAPI initialization successful (LiDAR active).")
            else: print("DEBUG: Background thread: IMUAPI initialization successful (LiDAR inactive/disabled/failed).")
        except (FileNotFoundError, KeyError, ValueError) as e:
            self.imu_api = None; self.api_init_error = f"Config Error: {str(e)}"
            print(f"DEBUG: Background thread: Config Error: {str(e)}")
        except (IMUCommunicationError, LidarCommunicationError) as e:
             self.imu_api = None; self.api_init_error = f"Connection Error: {str(e)}"
             print(f"DEBUG: Background thread: Connection Error: {str(e)}")
        except Exception as e:
            self.imu_api = None; self.api_init_error = f"Unexpected Init Error: {str(e)}"
            print(f"DEBUG: Background thread: Unexpected Initialization Error: {str(e)}")
            import traceback; traceback.print_exc()

    def _check_api_init(self):
        """Checks the result of the API initialization thread."""
        if self.init_thread.is_alive():
            self.root.after(100, self._check_api_init)
        else:
            if self.imu_api:
                status_msg = "IMU API Ready."
                status_msg += " LiDAR Active." if self.imu_api.lidar else " LiDAR Inactive/Disabled."
                self.log_status(status_msg, tag='success')
                if hasattr(self, 'calibrate_button'): self.calibrate_button.config(state=tk.NORMAL)
                self._show_frame(self.main_menu_frame)
            else:
                self.log_status(self.api_init_error or "Failed to initialize APIs.", tag='error')
                self.log_status("Please check config.ini and sensor connections.", tag='error')
                if hasattr(self, 'calibrate_button'): self.calibrate_button.config(state=tk.DISABLED)
                self._show_frame(self.main_menu_frame)

    # --- UI Setup ---
    def _setup_main_menu_frame(self):
        """Creates widgets for the main menu."""
        for widget in self.main_menu_frame.winfo_children(): widget.destroy()
        label = ttk.Label(self.main_menu_frame, text="Main Menu", font=("Helvetica", 16))
        label.pack(pady=20)
        self.calibrate_button = ttk.Button(
            self.main_menu_frame, text="Start Calibration",
            command=self._go_to_calibration_steps, state=tk.DISABLED
        )
        self.calibrate_button.pack(pady=30, ipadx=20, ipady=10)
        quit_button = ttk.Button(self.main_menu_frame, text="Quit", command=self._on_closing)
        quit_button.pack(pady=10)

    def _setup_calibration_frame(self):
        """Creates widgets for the calibration steps menu, initially hidden."""
        for widget in self.calibration_frame.winfo_children(): widget.destroy()

        self.cal_frame_label = ttk.Label(self.calibration_frame, text="Calibration Steps", font=("Helvetica", 16))
        # Don't pack label yet, pack it when entering the frame

        # Create buttons but don't pack them yet
        self.clock_sync_button = ttk.Button(
            self.calibration_frame, text="1. Calibrate Clocks (IMU + LiDAR)",
            command=self._do_clock_calibration_tk
        )
        self.still_cal_button = ttk.Button(
            self.calibration_frame, text="2. Calibrate Still (Gravity+Gyro)",
            command=self._do_still_calibration_tk
        )
        self.motion_cal_button = ttk.Button(
            self.calibration_frame, text="3. Calibrate Motion (Magnetometer)",
            command=self._do_motion_calibration_tk
        )
        self.finish_calibration_button = ttk.Button(
            self.calibration_frame, text="FINISH CALIBRATION & Return",
            command=self._go_to_main_menu, style='Finish.TButton'
        )
        # No early "Done" button anymore

    def _show_calibration_button(self, button_to_show: Optional[ttk.Button]):
        """Hides all calibration buttons and shows the specified one."""
        # Ensure all buttons exist before trying to hide
        buttons = [
            getattr(self, 'clock_sync_button', None),
            getattr(self, 'still_cal_button', None),
            getattr(self, 'motion_cal_button', None),
            getattr(self, 'finish_calibration_button', None)
        ]
        for btn in buttons:
            if btn and btn.winfo_ismapped(): # Check if it exists and is packed
                btn.pack_forget()

        # Show the requested button
        if button_to_show:
            button_to_show.pack(pady=20, fill=tk.X, padx=50) # Consistent padding
            button_to_show.config(state=tk.NORMAL)

    # --- Frame Navigation ---
    def _show_frame(self, frame_to_show):
        """Hides other frames and shows the specified frame."""
        self.main_menu_frame.pack_forget()
        self.calibration_frame.pack_forget()
        frame_to_show.pack(fill=tk.BOTH, expand=True)

    def _go_to_calibration_steps(self):
        """Callback for the main 'Start Calibration' button."""
        if not self.imu_api:
            self.log_status("Error: APIs not initialized.", tag='error'); messagebox.showerror("Error", "APIs not initialized."); return

        self.log_status("Navigating to calibration steps.", tag='info')
        self.current_state = STATE_CALIBRATING
        # Reset flags
        self.clocks_calibrated = False; self.still_calibrated = False; self.motion_calibrated = False

        # Setup calibration frame display
        if hasattr(self, 'cal_frame_label') and not self.cal_frame_label.winfo_ismapped():
             self.cal_frame_label.pack(pady=10) # Show label only once

        self._show_calibration_button(self.clock_sync_button) # Show only first step button
        self._show_frame(self.calibration_frame)
        self.log_status("Ready for Clock Calibration.", tag='info')

    # --- Clock Calibration Logic ---
    def _do_clock_calibration_tk(self):
        """Handles the 'Calibrate Clocks' button click."""
        if not self.imu_api: self.log_status("Error: APIs not initialized.", tag='error'); messagebox.showerror("Error", "APIs not initialized."); return

        self.log_status("Attempting clock synchronization (IMU + LiDAR)...", tag='info')
        self.clock_sync_button.config(state=tk.DISABLED) # Disable button during attempt

        cal_thread = threading.Thread(target=self._perform_clock_calibration, daemon=True)
        cal_thread.start()

    def _perform_clock_calibration(self):
        """Performs clock calibration (runs in background thread)."""
        result = {'status': 'Failed', 'message': "IMU API instance not available."} # Default fail result
        try:
            if self.imu_api: sync_result = self.imu_api.calibrate_clocks()
            else: sync_result = result # Use default fail if API gone
            self.root.after(0, self._clock_calibration_finished, sync_result)
        except (IMUCommunicationError, IMUTimeoutError, LidarCommunicationError) as e:
             result = {'status': 'Failed', 'message': f"Error during clock calibration call: {e}"}
             self.root.after(0, self._clock_calibration_finished, result)
        except Exception as e:
            result = {'status': 'Failed', 'message': f"Unexpected error during clock calibration: {e}"}
            self.root.after(0, self._clock_calibration_finished, result)
            import traceback; traceback.print_exc()

    def _clock_calibration_finished(self, result: Dict[str, Any]):
        """GUI update after clock calibration attempt (runs in main thread)."""
        status = result.get('status', 'Failed'); message = result.get('message', 'Unknown error.')
        log_tag = 'error' if status == 'Failed' else ('warning' if status.startswith('Partial') else 'success')
        self.clocks_calibrated = (status != 'Failed') # True if Success or Partial

        self.log_status(f"Clock Sync Status: {status}", tag=log_tag)
        self.log_status(f"Details: {message}", tag='info')
        # Log detailed timestamps if successful/partial
        if self.clocks_calibrated:
             # ... (timestamp logging code from previous version - kept for info) ...
            imu_pi_ns=result.get('pi_sys_time_for_imu_cal_ns'); imu_sens_us=result.get('imu_sensor_timestamp_at_cal_us')
            lid_pi_ns=result.get('pi_sys_time_anchor_lidar_ns'); lid_sens_s=result.get('lidar_sensor_timestamp_anchor_s')
            imu_pi_str=f"{imu_pi_ns:,}" if imu_pi_ns is not None else "N/A"; imu_sens_str=f"{imu_sens_us:,}" if imu_sens_us is not None else "N/A"
            lid_pi_str=f"{lid_pi_ns:,}" if lid_pi_ns is not None else "N/A"; lid_sens_str=f"{lid_sens_s:.3f}" if lid_sens_s is not None else "N/A"
            self.log_status(f"  IMU Pi Time (ns):   {imu_pi_str}", tag='info'); self.log_status(f"  IMU Sensor Time (us): {imu_sens_str}", tag='info')
            if status != 'Partial (No LiDAR API)':
                 self.log_status(f"  LiDAR Pi Time (ns):   {lid_pi_str}", tag='info'); self.log_status(f"  LiDAR Sensor Time (s):   {lid_sens_str}", tag='info')
                 if lid_pi_ns is None and status != 'Partial (No LiDAR Sync)': self.log_status("  (LiDAR sync data not captured)", tag='warning')

            # --- Flow Control ---
            self.log_status("Clock calibration finished. Ready for Still calibration.", tag='success')
            self._show_calibration_button(self.still_cal_button) # Show next button
        else:
            self.log_status(f"Clock calibration failed. Please check connection/config and retry.", tag='error')
            messagebox.showerror("Clock Calibration Failed", message)
            self.clock_sync_button.config(state=tk.NORMAL) # Re-enable button to allow retry

    # --- Still Calibration Logic ---
    def _do_still_calibration_tk(self):
        """Handles the 'Still Calibration' button click."""
        if not self.imu_api: self.log_status("Error: APIs not initialized.", tag='error'); return
        # No need to check clocks_calibrated here, button wouldn't be visible otherwise

        try:
            delay_sec = self.imu_api.still_delay; delay_ms = int(delay_sec * 1000)
            instruction = f"Place device flat and still. Waiting {delay_sec:.1f}s..."
            self.log_status("Preparing for Still Calibration.", tag='info'); self.log_status(instruction, tag='info')
            self.still_cal_button.config(state=tk.DISABLED) # Disable button during attempt
            self.root.after(delay_ms, self._schedule_still_calibration_thread)
        except AttributeError:
            self.log_status("Error: Could not find delay settings.", tag='error'); messagebox.showerror("Error", "Could not find delay settings.")
            self.still_cal_button.config(state=tk.NORMAL) # Re-enable on error
        except Exception as e:
            self.log_status(f"Unexpected Error preparing still calibration: {str(e)}", tag='error'); messagebox.showerror("Error", f"Unexpected error: {str(e)}")
            self.still_cal_button.config(state=tk.NORMAL)

    def _schedule_still_calibration_thread(self):
        cal_thread = threading.Thread(target=self._perform_actual_still_calibration, daemon=True)
        cal_thread.start()

    def _perform_actual_still_calibration(self):
        """Performs still calibration (runs in background thread)."""
        success = False
        try:
            if not self.imu_api: raise Exception("IMU API not available in thread.")
            self.root.after(0, lambda: self.log_status("Calibrating Gravity...", tag='info'))
            self.root.after(0, lambda: self.log_status("Keep device perfectly still.", tag='info'))
            self.imu_api.calibrate_gravity()
            self.root.after(0, lambda: self.log_status("Gravity calibrated.", tag='success'))
            self.root.after(0, lambda: self.log_status("Starting Gyro calibration (keep still)...", tag='info'))
            self.imu_api.calibrate_gyro()
            self.root.after(0, lambda: self.log_status("Still Calibration Complete (Gravity & Gyro).", tag='success'))
            success = True
        except (IMUCommunicationError, IMUCalibrationError, IMUTimeoutError) as e:
            self.root.after(0, self._sensor_calibration_failure, "Still", e)
        except Exception as e:
            self.root.after(0, self._sensor_calibration_failure, "Still", e)
            import traceback; traceback.print_exc()
        finally:
            # Update state and UI on main thread
            self.root.after(0, self._still_calibration_finished, success)

    def _still_calibration_finished(self, success: bool):
        """GUI update after still calibration attempt."""
        self.still_calibrated = success
        if success:
            # --- Flow Control ---
            self.log_status("Still calibration successful. Ready for Motion calibration.", tag='success')
            self._show_calibration_button(self.motion_cal_button) # Show next button
        else:
            # Error message handled by _sensor_calibration_failure
            self.still_cal_button.config(state=tk.NORMAL) # Re-enable button to allow retry

    # --- Motion Calibration Logic ---
    def _do_motion_calibration_tk(self):
        """Handles the 'Motion Calibration' button click."""
        if not self.imu_api: self.log_status("Error: APIs not initialized.", tag='error'); return

        try:
            delay_sec = self.imu_api.rotation_delay; delay_ms = int(delay_sec * 1000)
            prep_instruction = f"Prepare to rotate IMU through all axes. Starting in {delay_sec:.0f}s..."
            self.log_status("Preparing for Motion Calibration.", tag='info'); self.log_status(prep_instruction, tag='info')
            self.motion_cal_button.config(state=tk.DISABLED) # Disable button during attempt
            self.root.after(delay_ms, self._schedule_motion_calibration_thread)
        except AttributeError:
            self.log_status("Error: Could not find delay settings.", tag='error'); messagebox.showerror("Error", "Could not find delay settings.")
            self.motion_cal_button.config(state=tk.NORMAL) # Re-enable on error
        except Exception as e:
            self.log_status(f"Unexpected Error preparing motion calibration: {str(e)}", tag='error'); messagebox.showerror("Error", f"Unexpected error: {str(e)}")
            self.motion_cal_button.config(state=tk.NORMAL)

    def _schedule_motion_calibration_thread(self):
        cal_thread = threading.Thread(target=self._perform_actual_motion_calibration, daemon=True)
        cal_thread.start()

    def _perform_actual_motion_calibration(self):
        """Performs motion calibration (runs in background thread)."""
        success = False
        try:
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
            import traceback; traceback.print_exc()
        finally:
            # Update state and UI on main thread
            self.root.after(0, self._motion_calibration_finished, success)

    def _motion_calibration_finished(self, success: bool):
        """GUI update after motion calibration attempt."""
        self.motion_calibrated = success
        if success:
            # --- Flow Control ---
            self.log_status("All calibration steps complete!", tag='success')
            self._show_calibration_button(self.finish_calibration_button) # Show finish button
        else:
            # Error message handled by _sensor_calibration_failure
            self.motion_cal_button.config(state=tk.NORMAL) # Re-enable button to allow retry

    # --- Common Sensor Failure Handler ---
    def _sensor_calibration_failure(self, cal_type: str, error: Exception):
        """GUI update after failed sensor calibration (runs in main thread)."""
        error_str = str(error)
        # Reset corresponding flag on failure
        if cal_type == "Still": self.still_calibrated = False
        elif cal_type == "Motion": self.motion_calibrated = False

        self.log_status(f"{cal_type} Calibration Error: {error_str}", tag='error')
        self.log_status("Please try again.", tag='info')
        messagebox.showerror(f"{cal_type} Calibration Error", f"{error_str}\nPlease try again.")
        # Button re-enabling is handled in the calling _finished method

    # --- Navigation and Closing ---
    def _go_to_main_menu(self):
        """Callback for the 'Finish' button."""
        self.current_state = STATE_MAIN_MENU
        # Hide all calibration buttons before switching frame
        self._show_calibration_button(None)
        self._show_frame(self.main_menu_frame)

        # Update status log
        if self.imu_api and hasattr(self, 'calibrate_button'):
             self.calibrate_button.config(state=tk.NORMAL)
             status_msg = "IMU API Ready."
             status_msg += " LiDAR Active." if self.imu_api.lidar and self.imu_api.lidar._is_connected else " LiDAR Inactive."
             self.log_status(f"Calibration complete. Returned to Main Menu. {status_msg}", tag='info')
        elif hasattr(self, 'calibrate_button'):
             self.calibrate_button.config(state=tk.DISABLED)
             self.log_status(f"Returned to Main Menu. APIs disconnected or failed.", tag='warning')

        # Reset flags (already done when entering calibration)


    def _on_closing(self):
        """Handles the window close event."""
        print("DEBUG: Window closing...")
        if self.imu_api:
            print("Closing API resources (IMU/LiDAR)...")
            try: self.imu_api.close()
            except Exception as e: print(f"Error closing APIs: {e}")
        self.root.destroy()


# --- Main Execution ---
if __name__ == "__main__":
    print("DEBUG: Creating Tkinter root window...")
    root = tk.Tk()
    print("DEBUG: Creating CalibrationGUI_Tk instance...")
    try:
        app = CalibrationGUI_Tk(root)
        print("DEBUG: Starting Tkinter main loop...")
        root.mainloop()
    except Exception as e:
         print(f"\nFATAL ERROR during GUI initialization: {e}")
         messagebox.showerror("Fatal Error", f"Failed to initialize GUI: {e}\n\nPlease check configuration and logs.")
         try:
              if root: root.destroy()
         except: pass
         import traceback; traceback.print_exc()
    finally:
         print("DEBUG: Application finished.")

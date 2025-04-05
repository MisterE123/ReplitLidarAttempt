import tkinter as tk
from tkinter import ttk # Themed widgets
from tkinter import messagebox
import time
import sys
import threading # To run IMU init and collection in background
import traceback # For logging exceptions
from typing import Dict, Any, Optional

# --- Import API Modules ---
try:
    from imu_api import IMUAPI, IMUCommunicationError, IMUCalibrationError, IMUTimeoutError
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

# --- Import Data Collector ---
try:
    # Assuming main_collector.py contains the DataCollector class
    from main_collector import DataCollector
except ImportError as e:
    messagebox.showerror("Error", f"Could not import DataCollector from main_collector.py: {e}")
    # Allow GUI to start but disable collection button
    DataCollector = None # type: ignore
except Exception as e:
     messagebox.showerror("Error", f"Error importing DataCollector: {e}")
     DataCollector = None # type: ignore


# Define states
STATE_MAIN_MENU = 0
STATE_CALIBRATING = 1
STATE_COLLECTING = 2 # New state

class CalibrationGUI_Tk:
    def __init__(self, root):
        self.root = root
        self.root.title("IMU & LiDAR Calibration & Collection Tool")
        self.root.geometry("750x600") # Increased height slightly
        self.root.protocol("WM_DELETE_WINDOW", self._on_closing)

        self.imu_api: Optional[IMUAPI] = None
        self.api_init_error = None
        self.current_state = STATE_MAIN_MENU

        # --- Calibration Status Flags ---
        self.clocks_calibrated = False
        self.still_calibrated = False
        self.motion_calibrated = False

        # --- Collection State ---
        self.is_collecting = False
        self.collection_thread: Optional[threading.Thread] = None
        self.collector_stop_event = threading.Event() # Event to signal collection thread

        # --- Style ---
        self.style = ttk.Style()
        available_themes = self.style.theme_names()
        print(f"Available themes: {available_themes}")
        if 'clam' in available_themes: self.style.theme_use('clam')
        elif 'alt' in available_themes: self.style.theme_use('alt')
        self.style.configure('TButton', padding=6)
        self.style.configure('Finish.TButton', foreground='blue', font=('Helvetica', 10, 'bold'))
        self.style.configure('Stop.TButton', foreground='red', font=('Helvetica', 10, 'bold')) # Style for Stop button

        # --- Main Frames ---
        self.main_menu_frame = ttk.Frame(root, padding="10")
        self.calibration_frame = ttk.Frame(root, padding="10")
        # No separate collection frame needed, buttons change in main menu

        # --- Status Display Area ---
        self.status_frame = ttk.LabelFrame(root, text="Status Log", padding="5")
        self.status_text = tk.Text(self.status_frame, height=12, width=80, wrap=tk.WORD, state=tk.DISABLED, relief=tk.SUNKEN, borderwidth=1) # Increased height
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
        if hasattr(self, 'status_text') and self.status_text.winfo_exists():
            try:
                self.status_text.config(state=tk.NORMAL)
                # Define tags if they don't exist yet
                tag_config = {'error': {'foreground': 'red'},
                              'success': {'foreground': 'green'},
                              'warning': {'foreground': 'orange'},
                              'info': {'foreground': 'black'},
                              'collect': {'foreground': 'purple', 'font': ('Helvetica', 9, 'bold')}}
                for t, config in tag_config.items():
                    if t not in self.status_text.tag_names():
                        self.status_text.tag_config(t, **config)

                self.status_text.insert(tk.END, f"[{tag.upper()}] {message}\n", tag)
                self.status_text.config(state=tk.DISABLED)
                self.status_text.see(tk.END) # Scroll to the bottom
            except tk.TclError:
                 print("GUI Log WARN: Status text widget no longer exists.") # Handle race condition on close
            except Exception as e:
                 print(f"GUI Log ERROR: Unexpected error logging status: {e}")


    # --- Initialization ---
    def _initialize_apis(self):
        """Runs in a separate thread to avoid blocking the GUI."""
        print("DEBUG: Background thread: Attempting to initialize IMUAPI...")
        try:
            # IMUAPI now handles internal LiDAR init based on config
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
            traceback.print_exc()

    def _check_api_init(self):
        """Checks the result of the API initialization thread."""
        if self.init_thread.is_alive():
            self.root.after(100, self._check_api_init)
        else:
            if self.imu_api:
                status_msg = "IMU API Ready."
                status_msg += " LiDAR Active." if self.imu_api.lidar and self.imu_api.lidar._is_connected else " LiDAR Inactive/Disabled."
                self.log_status(status_msg, tag='success')
                # Enable buttons that depend on APIs
                if hasattr(self, 'calibrate_button'): self.calibrate_button.config(state=tk.NORMAL)
                if hasattr(self, 'start_collection_button'):
                     # Only enable if DataCollector was imported successfully
                     if DataCollector: self.start_collection_button.config(state=tk.NORMAL)
                     else: self.start_collection_button.config(state=tk.DISABLED)

                self._show_frame(self.main_menu_frame)
            else:
                self.log_status(self.api_init_error or "Failed to initialize APIs.", tag='error')
                self.log_status("Please check config.ini and sensor connections.", tag='error')
                # Disable buttons
                if hasattr(self, 'calibrate_button'): self.calibrate_button.config(state=tk.DISABLED)
                if hasattr(self, 'start_collection_button'): self.start_collection_button.config(state=tk.DISABLED)
                self._show_frame(self.main_menu_frame)

    # --- UI Setup ---
    def _setup_main_menu_frame(self):
        """Creates widgets for the main menu."""
        for widget in self.main_menu_frame.winfo_children(): widget.destroy()
        label = ttk.Label(self.main_menu_frame, text="Main Menu", font=("Helvetica", 16))
        label.pack(pady=20)

        # Calibration Button
        self.calibrate_button = ttk.Button(
            self.main_menu_frame, text="Start Calibration",
            command=self._go_to_calibration_steps, state=tk.DISABLED
        )
        self.calibrate_button.pack(pady=15, ipadx=20, ipady=10)

        # Data Collection Buttons (Start shown initially)
        self.start_collection_button = ttk.Button(
             self.main_menu_frame, text="Start Data Collection",
             command=self._start_data_collection, state=tk.DISABLED # Disabled until APIs ready
        )
        self.stop_collection_button = ttk.Button(
             self.main_menu_frame, text="STOP Data Collection",
             command=self._stop_data_collection, style='Stop.TButton', state=tk.DISABLED
        )
        # Pack the start button initially
        self.start_collection_button.pack(pady=15, ipadx=20, ipady=10)
        # Stop button is packed/unpacked dynamically

        # Quit Button
        quit_button = ttk.Button(self.main_menu_frame, text="Quit", command=self._on_closing)
        quit_button.pack(pady=15)

        # Check if DataCollector failed import and permanently disable start button
        if DataCollector is None:
             self.log_status("Data collection disabled due to import error.", tag='error')
             self.start_collection_button.config(state=tk.DISABLED)


    def _setup_calibration_frame(self):
        """Creates widgets for the calibration steps menu, initially hidden."""
        # ... (Setup remains the same as before) ...
        for widget in self.calibration_frame.winfo_children(): widget.destroy()
        self.cal_frame_label = ttk.Label(self.calibration_frame, text="Calibration Steps", font=("Helvetica", 16))
        self.clock_sync_button = ttk.Button(self.calibration_frame, text="1. Calibrate Clocks (IMU + LiDAR)", command=self._do_clock_calibration_tk)
        self.still_cal_button = ttk.Button(self.calibration_frame, text="2. Calibrate Still (Gravity+Gyro)", command=self._do_still_calibration_tk)
        self.motion_cal_button = ttk.Button(self.calibration_frame, text="3. Calibrate Motion (Magnetometer)", command=self._do_motion_calibration_tk)
        self.finish_calibration_button = ttk.Button(self.calibration_frame, text="FINISH CALIBRATION & Return", command=self._go_to_main_menu, style='Finish.TButton')


    def _show_calibration_button(self, button_to_show: Optional[ttk.Button]):
        """Hides all calibration buttons and shows the specified one."""
        # ... (Logic remains the same as before) ...
        buttons = [getattr(self, name, None) for name in ['clock_sync_button', 'still_cal_button', 'motion_cal_button', 'finish_calibration_button']]
        for btn in buttons:
            if btn and btn.winfo_ismapped(): btn.pack_forget()
        if button_to_show:
            button_to_show.pack(pady=20, fill=tk.X, padx=50)
            button_to_show.config(state=tk.NORMAL)


    # --- Frame Navigation ---
    def _show_frame(self, frame_to_show):
        """Hides other frames and shows the specified frame."""
        # ... (Logic remains the same as before) ...
        self.main_menu_frame.pack_forget()
        self.calibration_frame.pack_forget()
        frame_to_show.pack(fill=tk.BOTH, expand=True)

    def _go_to_calibration_steps(self):
        """Callback for the main 'Start Calibration' button."""
        # ... (Logic remains largely the same, ensure collection isn't running) ...
        if self.is_collecting:
             messagebox.showwarning("Busy", "Cannot start calibration while data collection is running.")
             return
        if not self.imu_api:
            self.log_status("Error: APIs not initialized.", tag='error'); messagebox.showerror("Error", "APIs not initialized."); return

        self.log_status("Navigating to calibration steps.", tag='info')
        self.current_state = STATE_CALIBRATING
        self.clocks_calibrated = False; self.still_calibrated = False; self.motion_calibrated = False
        if hasattr(self, 'cal_frame_label') and not self.cal_frame_label.winfo_ismapped():
             self.cal_frame_label.pack(pady=10)
        self._show_calibration_button(self.clock_sync_button)
        self._show_frame(self.calibration_frame)
        self.log_status("Ready for Clock Calibration.", tag='info')

    # --- Clock Calibration Logic ---
    # ... (All calibration methods _do_..._tk, _perform_..., _..._finished remain the same) ...
    def _do_clock_calibration_tk(self):
        if not self.imu_api: self.log_status("Error: APIs not initialized.", tag='error'); return
        self.log_status("Attempting clock synchronization (IMU + LiDAR)...", tag='info')
        self.clock_sync_button.config(state=tk.DISABLED)
        cal_thread = threading.Thread(target=self._perform_clock_calibration, daemon=True)
        cal_thread.start()

    def _perform_clock_calibration(self):
        result = {'status': 'Failed', 'message': "IMU API instance not available."}
        try:
            if self.imu_api: sync_result = self.imu_api.calibrate_clocks()
            else: sync_result = result
            self.root.after(0, self._clock_calibration_finished, sync_result)
        except (IMUCommunicationError, IMUTimeoutError, LidarCommunicationError) as e:
             result = {'status': 'Failed', 'message': f"Error during clock calibration call: {e}"}
             self.root.after(0, self._clock_calibration_finished, result)
        except Exception as e:
            result = {'status': 'Failed', 'message': f"Unexpected error during clock calibration: {e}"}
            self.root.after(0, self._clock_calibration_finished, result)
            traceback.print_exc()

    def _clock_calibration_finished(self, result: Dict[str, Any]):
        status = result.get('status', 'Failed'); message = result.get('message', 'Unknown error.')
        log_tag = 'error' if status == 'Failed' else ('warning' if status.startswith('Partial') else 'success')
        self.clocks_calibrated = (status != 'Failed')
        self.log_status(f"Clock Sync Status: {status}", tag=log_tag)
        self.log_status(f"Details: {message}", tag='info')
        if self.clocks_calibrated:
            imu_pi_ns=result.get('pi_sys_time_for_imu_cal_ns'); imu_sens_us=result.get('imu_sensor_timestamp_at_cal_us')
            lid_pi_ns=result.get('pi_sys_time_anchor_lidar_ns'); lid_sens_s=result.get('lidar_sensor_timestamp_anchor_s')
            imu_pi_str=f"{imu_pi_ns:,}" if imu_pi_ns is not None else "N/A"; imu_sens_str=f"{imu_sens_us:,}" if imu_sens_us is not None else "N/A"
            lid_pi_str=f"{lid_pi_ns:,}" if lid_pi_ns is not None else "N/A"; lid_sens_str=f"{lid_sens_s:.3f}" if lid_sens_s is not None else "N/A"
            self.log_status(f"  IMU Pi Time (ns):   {imu_pi_str}", tag='info'); self.log_status(f"  IMU Sensor Time (us): {imu_sens_str}", tag='info')
            if status != 'Partial (No LiDAR API)':
                 self.log_status(f"  LiDAR Pi Time (ns):   {lid_pi_str}", tag='info'); self.log_status(f"  LiDAR Sensor Time (s):   {lid_sens_str}", tag='info')
                 if lid_pi_ns is None and status != 'Partial (No LiDAR Sync)': self.log_status("  (LiDAR sync data not captured)", tag='warning')
            self.log_status("Clock calibration finished. Ready for Still calibration.", tag='success')
            self._show_calibration_button(self.still_cal_button)
        else:
            self.log_status(f"Clock calibration failed. Please check connection/config and retry.", tag='error')
            messagebox.showerror("Clock Calibration Failed", message)
            self.clock_sync_button.config(state=tk.NORMAL)

    # --- Still Calibration Logic ---
    def _do_still_calibration_tk(self):
        if not self.imu_api: self.log_status("Error: APIs not initialized.", tag='error'); return
        try:
            delay_sec = self.imu_api.still_delay; delay_ms = int(delay_sec * 1000)
            instruction = f"Place device flat and still. Waiting {delay_sec:.1f}s..."
            self.log_status("Preparing for Still Calibration.", tag='info'); self.log_status(instruction, tag='info')
            self.still_cal_button.config(state=tk.DISABLED)
            self.root.after(delay_ms, self._schedule_still_calibration_thread)
        except AttributeError:
            self.log_status("Error: Could not find delay settings.", tag='error'); messagebox.showerror("Error", "Could not find delay settings.")
            self.still_cal_button.config(state=tk.NORMAL)
        except Exception as e:
            self.log_status(f"Unexpected Error preparing still calibration: {str(e)}", tag='error'); messagebox.showerror("Error", f"Unexpected error: {str(e)}")
            self.still_cal_button.config(state=tk.NORMAL)

    def _schedule_still_calibration_thread(self):
        cal_thread = threading.Thread(target=self._perform_actual_still_calibration, daemon=True)
        cal_thread.start()

    def _perform_actual_still_calibration(self):
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
            traceback.print_exc()
        finally:
            self.root.after(0, self._still_calibration_finished, success)

    def _still_calibration_finished(self, success: bool):
        self.still_calibrated = success
        if success:
            self.log_status("Still calibration successful. Ready for Motion calibration.", tag='success')
            self._show_calibration_button(self.motion_cal_button)
        else:
            self.still_cal_button.config(state=tk.NORMAL)

    # --- Motion Calibration Logic ---
    def _do_motion_calibration_tk(self):
        if not self.imu_api: self.log_status("Error: APIs not initialized.", tag='error'); return
        try:
            delay_sec = self.imu_api.rotation_delay; delay_ms = int(delay_sec * 1000)
            prep_instruction = f"Prepare to rotate IMU through all axes. Starting in {delay_sec:.0f}s..."
            self.log_status("Preparing for Motion Calibration.", tag='info'); self.log_status(prep_instruction, tag='info')
            self.motion_cal_button.config(state=tk.DISABLED)
            self.root.after(delay_ms, self._schedule_motion_calibration_thread)
        except AttributeError:
            self.log_status("Error: Could not find delay settings.", tag='error'); messagebox.showerror("Error", "Could not find delay settings.")
            self.motion_cal_button.config(state=tk.NORMAL)
        except Exception as e:
            self.log_status(f"Unexpected Error preparing motion calibration: {str(e)}", tag='error'); messagebox.showerror("Error", f"Unexpected error: {str(e)}")
            self.motion_cal_button.config(state=tk.NORMAL)

    def _schedule_motion_calibration_thread(self):
        cal_thread = threading.Thread(target=self._perform_actual_motion_calibration, daemon=True)
        cal_thread.start()

    def _perform_actual_motion_calibration(self):
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
            traceback.print_exc()
        finally:
            self.root.after(0, self._motion_calibration_finished, success)

    def _motion_calibration_finished(self, success: bool):
        self.motion_calibrated = success
        if success:
            self.log_status("All calibration steps complete!", tag='success')
            self._show_calibration_button(self.finish_calibration_button)
        else:
            self.motion_cal_button.config(state=tk.NORMAL)

    # --- Common Sensor Failure Handler ---
    def _sensor_calibration_failure(self, cal_type: str, error: Exception):
        # ... (Logic remains the same) ...
        error_str = str(error)
        if cal_type == "Still": self.still_calibrated = False
        elif cal_type == "Motion": self.motion_calibrated = False
        self.log_status(f"{cal_type} Calibration Error: {error_str}", tag='error')
        self.log_status("Please try again.", tag='info')
        messagebox.showerror(f"{cal_type} Calibration Error", f"{error_str}\nPlease try again.")


    # --- Data Collection Logic ---

    def _start_data_collection(self):
        """Callback for the 'Start Data Collection' button."""
        if self.is_collecting:
            self.log_status("Collection is already running.", tag='warning')
            return
        if self.current_state == STATE_CALIBRATING:
             messagebox.showwarning("Busy", "Cannot start collection while in calibration mode.")
             return
        if not self.imu_api or not DataCollector:
             self.log_status("Cannot start collection: APIs not ready or DataCollector missing.", tag='error')
             messagebox.showerror("Error", "APIs not ready or DataCollector module missing.")
             return

        self.log_status("Starting data collection process...", tag='collect')
        self.current_state = STATE_COLLECTING
        self.is_collecting = True

        # Update button states
        self.calibrate_button.config(state=tk.DISABLED)
        self.start_collection_button.pack_forget() # Hide Start button
        self.stop_collection_button.config(state=tk.NORMAL) # Enable Stop button
        self.stop_collection_button.pack(pady=15, ipadx=20, ipady=10) # Show Stop button

        # Reset the stop event for the new collection run
        self.collector_stop_event.clear()

        # Start the collection process in a background thread
        self.collection_thread = threading.Thread(target=self._run_collection_thread, daemon=True)
        self.collection_thread.start()

    def _stop_data_collection(self):
        """Callback for the 'Stop Data Collection' button."""
        if not self.is_collecting or self.collection_thread is None:
            self.log_status("No active collection process to stop.", tag='warning')
            return

        self.log_status("Sending stop signal to data collection process...", tag='collect')
        self.stop_collection_button.config(state=tk.DISABLED) # Disable button while stopping

        # Signal the collection thread to stop
        self.collector_stop_event.set()
        # The _collection_finished_update method will handle UI changes when the thread finishes

    def _run_collection_thread(self):
        """Target function for the background data collection thread."""
        self.log_status("Collection thread started.", tag='info')
        try:
            # Instantiate and run the DataCollector from main_collector.py
            collector = DataCollector(config_path='config.ini')
            # The run method now blocks until stop_event is set and cleanup is done
            collector.run(self.collector_stop_event)
            self.log_status("Data collection process finished cleanly.", tag='success')
        except FileNotFoundError as e:
             self.log_status(f"Collection Error: Config file not found: {e}", tag='error')
        except (KeyError, ValueError) as e:
             self.log_status(f"Collection Error: Invalid config: {e}", tag='error')
        except Exception as e:
            self.log_status(f"Data collection thread encountered an error: {e}", tag='error')
            traceback.print_exc() # Log full traceback to console/log file
        finally:
            self.log_status("Collection thread attempting final update.", tag='info')
            # Schedule the UI update to run on the main thread
            if self.root.winfo_exists(): # Check if root window still exists
                 self.root.after(0, self._collection_finished_update)
            else:
                 print("Collection thread finished, but GUI root window no longer exists.")


    def _collection_finished_update(self):
        """Updates the GUI after the collection thread has finished."""
        self.log_status("Updating GUI after collection finished.", tag='info')
        self.is_collecting = False
        self.current_state = STATE_MAIN_MENU # Return to main menu state
        self.collection_thread = None # Clear thread reference

        # Update button states and visibility
        self.stop_collection_button.pack_forget() # Hide Stop button
        # Only show/enable start button if APIs are ready and DataCollector exists
        if self.imu_api and DataCollector:
             self.start_collection_button.config(state=tk.NORMAL)
             self.calibrate_button.config(state=tk.NORMAL)
        else:
             self.start_collection_button.config(state=tk.DISABLED)
             self.calibrate_button.config(state=tk.DISABLED)

        # Ensure start button is packed if it should be visible
        if not self.start_collection_button.winfo_ismapped():
             self.start_collection_button.pack(pady=15, ipadx=20, ipady=10) # Show Start button


    # --- Navigation and Closing ---
    def _go_to_main_menu(self):
        """Callback for the 'Finish Calibration' button."""
        self.current_state = STATE_MAIN_MENU
        self._show_calibration_button(None) # Hide all calibration buttons
        self._show_frame(self.main_menu_frame)

        # Update status log and button states for main menu
        if self.imu_api:
             status_msg = "IMU API Ready."
             status_msg += " LiDAR Active." if self.imu_api.lidar and self.imu_api.lidar._is_connected else " LiDAR Inactive."
             self.log_status(f"Returned to Main Menu. {status_msg}", tag='info')
             self.calibrate_button.config(state=tk.NORMAL)
             if DataCollector: self.start_collection_button.config(state=tk.NORMAL)
             else: self.start_collection_button.config(state=tk.DISABLED)
        else:
             self.log_status(f"Returned to Main Menu. APIs disconnected or failed.", tag='warning')
             self.calibrate_button.config(state=tk.DISABLED)
             self.start_collection_button.config(state=tk.DISABLED)

        # Ensure correct collection button is shown
        self.stop_collection_button.pack_forget()
        if not self.start_collection_button.winfo_ismapped():
             self.start_collection_button.pack(pady=15, ipadx=20, ipady=10)


    def _on_closing(self):
        """Handles the window close event."""
        print("DEBUG: Window closing...")
        if self.is_collecting:
            self.log_status("Window closed during collection. Attempting to stop...", tag='warning')
            self.collector_stop_event.set()
            if self.collection_thread:
                print("DEBUG: Waiting briefly for collection thread to stop...")
                self.collection_thread.join(timeout=2.0) # Wait max 2 seconds
                if self.collection_thread.is_alive():
                     print("DEBUG: Collection thread did not stop cleanly on close.")

        # Close IMU API if initialized
        if self.imu_api:
            print("Closing API resources (IMU/LiDAR)...")
            try: self.imu_api.close()
            except Exception as e: print(f"Error closing APIs: {e}")

        print("DEBUG: Destroying root window.")
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
         traceback.print_exc()
         try:
              if root and root.winfo_exists(): root.destroy()
         except: pass
    finally:
         print("DEBUG: Application finished.")

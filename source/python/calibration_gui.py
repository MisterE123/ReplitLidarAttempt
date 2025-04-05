import tkinter as tk
from tkinter import ttk , simpledialog, messagebox # Added simpledialog
import time
import sys
import threading
import traceback
import re # For sanitizing filenames
from typing import Dict, Any, Optional

# --- Import API Modules ---
try:
    from imu_api import IMUAPI, IMUCommunicationError, IMUCalibrationError, IMUTimeoutError
    try: from lidar_api import LidarCommunicationError
    except ImportError: LidarCommunicationError = Exception
except ImportError: messagebox.showerror("Error", "Could not find imu_api.py."); sys.exit(1)
except FileNotFoundError as e: messagebox.showerror("Config Error", f"Could not find config.ini: {e}"); sys.exit(1)
except (KeyError, ValueError) as e: messagebox.showerror("Config Error", f"Error reading config.ini: {e}"); sys.exit(1)

# --- Import Data Collector ---
DataCollector = None # Initialize to None
try:
    # Assumes updated main_collector.py with DataCollector class
    from main_collector import DataCollector
    # --- ADDED LOGGING ---
    if DataCollector:
        print("DEBUG: Successfully imported DataCollector from main_collector.py.")
    else:
        # This case shouldn't happen if the import succeeded without error,
        # but adding for completeness.
        print("DEBUG: Imported main_collector.py, but DataCollector is None/False.")
    # --- END ADDED LOGGING ---
except ImportError as e:
    messagebox.showerror("Error", f"Could not import DataCollector from main_collector.py: {e}")
    # --- ADDED LOGGING ---
    print(f"DEBUG: ImportError occurred when importing DataCollector: {e}")
    # --- END ADDED LOGGING ---
    DataCollector = None # type: ignore
except Exception as e:
     messagebox.showerror("Error", f"Error importing DataCollector: {e}")
     # --- ADDED LOGGING ---
     print(f"DEBUG: An unexpected Exception occurred when importing DataCollector: {e}")
     traceback.print_exc() # Print stack trace for unexpected errors
     # --- END ADDED LOGGING ---
     DataCollector = None # type: ignore


# Define states
STATE_MAIN_MENU = 0
STATE_CALIBRATING = 1
STATE_COLLECTING = 2

class CalibrationGUI_Tk:
    def __init__(self, root):
        self.root = root
        self.root.title("IMU & LiDAR Calibration & Collection Tool")
        self.root.geometry("750x600")
        self.root.protocol("WM_DELETE_WINDOW", self._on_closing)

        self.imu_api: Optional[IMUAPI] = None
        self.api_init_error = None
        self.current_state = STATE_MAIN_MENU

        self.clocks_calibrated = False
        self.still_calibrated = False
        self.motion_calibrated = False

        self.is_collecting = False
        self.collection_thread: Optional[threading.Thread] = None
        self.collector_stop_event = threading.Event()
        self.data_collector_instance: Optional[DataCollector] = None # Store instance

        # --- Style ---
        self.style = ttk.Style()
        available_themes = self.style.theme_names(); print(f"Available themes: {available_themes}")
        if 'clam' in available_themes: self.style.theme_use('clam')
        elif 'alt' in available_themes: self.style.theme_use('alt')
        self.style.configure('TButton', padding=6)
        self.style.configure('Finish.TButton', foreground='blue', font=('Helvetica', 10, 'bold'))
        self.style.configure('Stop.TButton', foreground='red', font=('Helvetica', 10, 'bold'))

        # --- Frames & Status ---
        self.main_menu_frame = ttk.Frame(root, padding="10")
        self.calibration_frame = ttk.Frame(root, padding="10")
        self.status_frame = ttk.LabelFrame(root, text="Status Log", padding="5")
        self.status_text = tk.Text(self.status_frame, height=12, width=80, wrap=tk.WORD, state=tk.DISABLED, relief=tk.SUNKEN, borderwidth=1)
        self.status_scrollbar = ttk.Scrollbar(self.status_frame, orient=tk.VERTICAL, command=self.status_text.yview)
        self.status_text.config(yscrollcommand=self.status_scrollbar.set)
        self.status_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True); self.status_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.status_frame.pack(side=tk.BOTTOM, fill=tk.X, padx=5, pady=5)

        self.log_status("Initializing...", tag='info')
        self._setup_main_menu_frame()
        self._setup_calibration_frame()

        self.log_status("Connecting to IMU and LiDAR...", tag='info')
        self.init_thread = threading.Thread(target=self._initialize_apis, daemon=True); self.init_thread.start()
        self.root.after(100, self._check_api_init)

    # --- Logging ---
    def log_status(self, message: str, tag: str = 'info'):
        """Appends a message to the status Text widget."""
        print(f"GUI Log ({tag}): {message}")
        if hasattr(self, 'status_text') and self.status_text.winfo_exists():
            try:
                self.status_text.config(state=tk.NORMAL)
                tag_config = {'error': {'foreground': 'red'}, 'success': {'foreground': 'green'},
                              'warning': {'foreground': 'orange'}, 'info': {'foreground': 'black'},
                              'collect': {'foreground': 'purple', 'font': ('Helvetica', 9, 'bold')}}
                for t, config in tag_config.items():
                    if t not in self.status_text.tag_names(): self.status_text.tag_config(t, **config)
                self.status_text.insert(tk.END, f"[{tag.upper()}] {message}\n", tag)
                self.status_text.config(state=tk.DISABLED); self.status_text.see(tk.END)
            except tk.TclError: print("GUI Log WARN: Status text widget no longer exists.")
            except Exception as e: print(f"GUI Log ERROR: Unexpected error logging status: {e}")

    # --- Initialization ---
    def _initialize_apis(self):
        """Initializes IMU API in background."""
        print("DEBUG: Background thread: Attempting to initialize IMUAPI...")
        try:
            self.imu_api = IMUAPI()
            if self.imu_api.lidar: print("DEBUG: IMUAPI initialization successful (LiDAR active).")
            else: print("DEBUG: IMUAPI initialization successful (LiDAR inactive).")
        except (FileNotFoundError, KeyError, ValueError) as e: self.imu_api = None; self.api_init_error = f"Config Error: {str(e)}"
        except (IMUCommunicationError, LidarCommunicationError) as e: self.imu_api = None; self.api_init_error = f"Connection Error: {str(e)}"
        except Exception as e: self.imu_api = None; self.api_init_error = f"Unexpected Init Error: {str(e)}"; traceback.print_exc()

    def _check_api_init(self):
        """Checks API initialization result."""
        if self.init_thread.is_alive():
            self.root.after(100, self._check_api_init); return

        if self.imu_api:
            status_msg = "IMU API Ready."
            status_msg += " LiDAR Active." if self.imu_api.lidar and self.imu_api.lidar._is_connected else " LiDAR Inactive/Disabled."
            self.log_status(status_msg, tag='success')
            if hasattr(self, 'calibrate_button'): self.calibrate_button.config(state=tk.NORMAL)
            if hasattr(self, 'start_collection_button'):
                 # --- Check DataCollector status here too ---
                 print(f"DEBUG (_check_api_init): Checking DataCollector status. DataCollector is None: {DataCollector is None}")
                 if DataCollector:
                      self.start_collection_button.config(state=tk.NORMAL)
                      print("DEBUG (_check_api_init): Enabling Start Collection button.")
                 else:
                      self.start_collection_button.config(state=tk.DISABLED)
                      print("DEBUG (_check_api_init): DataCollector is None, keeping Start Collection button DISABLED.")
                 # --- End Check ---
            self._show_frame(self.main_menu_frame)
        else:
            self.log_status(self.api_init_error or "Failed to initialize APIs.", tag='error')
            self.log_status("Please check config.ini and sensor connections.", tag='error')
            if hasattr(self, 'calibrate_button'): self.calibrate_button.config(state=tk.DISABLED)
            if hasattr(self, 'start_collection_button'): self.start_collection_button.config(state=tk.DISABLED)
            self._show_frame(self.main_menu_frame)

    # --- UI Setup ---
    def _setup_main_menu_frame(self):
        """Creates widgets for the main menu."""
        for widget in self.main_menu_frame.winfo_children(): widget.destroy()
        label = ttk.Label(self.main_menu_frame, text="Main Menu", font=("Helvetica", 16)); label.pack(pady=20)
        self.calibrate_button = ttk.Button(self.main_menu_frame, text="Start Calibration", command=self._go_to_calibration_steps, state=tk.DISABLED)
        self.calibrate_button.pack(pady=15, ipadx=20, ipady=10)
        self.start_collection_button = ttk.Button(self.main_menu_frame, text="Start Data Collection", command=self._start_data_collection, state=tk.DISABLED)
        self.stop_collection_button = ttk.Button(self.main_menu_frame, text="STOP Data Collection", command=self._stop_data_collection, style='Stop.TButton', state=tk.DISABLED)
        self.start_collection_button.pack(pady=15, ipadx=20, ipady=10) # Start shown initially
        quit_button = ttk.Button(self.main_menu_frame, text="Quit", command=self._on_closing); quit_button.pack(pady=15)
        if DataCollector is None:
             # Log this potential issue during setup
             self.log_status("Data collection disabled (DataCollector is None at setup).", tag='warning')
             self.start_collection_button.config(state=tk.DISABLED)

    def _setup_calibration_frame(self):
        """Creates widgets for calibration steps."""
        for widget in self.calibration_frame.winfo_children(): widget.destroy()
        self.cal_frame_label = ttk.Label(self.calibration_frame, text="Calibration Steps", font=("Helvetica", 16))
        self.clock_sync_button = ttk.Button(self.calibration_frame, text="1. Calibrate Clocks (IMU + LiDAR)", command=self._do_clock_calibration_tk)
        self.still_cal_button = ttk.Button(self.calibration_frame, text="2. Calibrate Still (Gravity+Gyro)", command=self._do_still_calibration_tk)
        self.motion_cal_button = ttk.Button(self.calibration_frame, text="3. Calibrate Motion (Magnetometer)", command=self._do_motion_calibration_tk)
        self.finish_calibration_button = ttk.Button(self.calibration_frame, text="FINISH CALIBRATION & Return", command=self._go_to_main_menu, style='Finish.TButton')

    def _show_calibration_button(self, button_to_show: Optional[ttk.Button]):
        """Shows/hides calibration step buttons."""
        buttons = [getattr(self, name, None) for name in ['clock_sync_button', 'still_cal_button', 'motion_cal_button', 'finish_calibration_button']]
        for btn in buttons:
            if btn and btn.winfo_ismapped(): btn.pack_forget()
        if button_to_show:
            button_to_show.pack(pady=20, fill=tk.X, padx=50); button_to_show.config(state=tk.NORMAL)

    # --- Frame Navigation ---
    def _show_frame(self, frame_to_show):
        """Shows the specified frame."""
        self.main_menu_frame.pack_forget(); self.calibration_frame.pack_forget()
        frame_to_show.pack(fill=tk.BOTH, expand=True)

    def _go_to_calibration_steps(self):
        """Navigates to the calibration frame."""
        if self.is_collecting: messagebox.showwarning("Busy", "Cannot start calibration while collecting data."); return
        if not self.imu_api: messagebox.showerror("Error", "APIs not initialized."); return
        self.log_status("Navigating to calibration steps.", tag='info')
        self.current_state = STATE_CALIBRATING
        self.clocks_calibrated = False; self.still_calibrated = False; self.motion_calibrated = False
        if hasattr(self, 'cal_frame_label') and not self.cal_frame_label.winfo_ismapped(): self.cal_frame_label.pack(pady=10)
        self._show_calibration_button(self.clock_sync_button); self._show_frame(self.calibration_frame)
        self.log_status("Ready for Clock Calibration.", tag='info')

    # --- Calibration Logic Methods ---
    # ... (All calibration methods remain unchanged) ...
    def _do_clock_calibration_tk(self):
        if not self.imu_api: self.log_status("Error: APIs not initialized.", tag='error'); return
        self.log_status("Attempting clock synchronization (IMU + LiDAR)...", tag='info')
        self.clock_sync_button.config(state=tk.DISABLED)
        cal_thread = threading.Thread(target=self._perform_clock_calibration, daemon=True); cal_thread.start()
    def _perform_clock_calibration(self):
        result = {'status': 'Failed', 'message': "IMU API instance not available."}
        try:
            if self.imu_api: sync_result = self.imu_api.calibrate_clocks()
            else: sync_result = result
            self.root.after(0, self._clock_calibration_finished, sync_result)
        except (IMUCommunicationError, IMUTimeoutError, LidarCommunicationError) as e: result = {'status': 'Failed', 'message': f"Cal clock error: {e}"}; self.root.after(0, self._clock_calibration_finished, result)
        except Exception as e: result = {'status': 'Failed', 'message': f"Unexpected cal clock error: {e}"}; self.root.after(0, self._clock_calibration_finished, result); traceback.print_exc()
    def _clock_calibration_finished(self, result: Dict[str, Any]):
        status = result.get('status', 'Failed'); message = result.get('message', 'Unknown error.')
        log_tag = 'error' if status == 'Failed' else ('warning' if status.startswith('Partial') else 'success')
        self.clocks_calibrated = (status != 'Failed')
        self.log_status(f"Clock Sync Status: {status}", tag=log_tag); self.log_status(f"Details: {message}", tag='info')
        if self.clocks_calibrated: self._show_calibration_button(self.still_cal_button); self.log_status("Ready for Still calibration.", tag='success')
        else: messagebox.showerror("Clock Calibration Failed", message); self.clock_sync_button.config(state=tk.NORMAL)
    def _do_still_calibration_tk(self):
        if not self.imu_api: self.log_status("Error: APIs not initialized.", tag='error'); return
        try:
            delay_sec = self.imu_api.still_delay; delay_ms = int(delay_sec * 1000)
            instruction = f"Place device flat and still. Waiting {delay_sec:.1f}s..."
            self.log_status("Preparing for Still Calibration.", tag='info'); self.log_status(instruction, tag='info')
            self.still_cal_button.config(state=tk.DISABLED); self.root.after(delay_ms, self._schedule_still_calibration_thread)
        except Exception as e: messagebox.showerror("Error", f"Error preparing still cal: {str(e)}"); self.still_cal_button.config(state=tk.NORMAL)
    def _schedule_still_calibration_thread(self): cal_thread = threading.Thread(target=self._perform_actual_still_calibration, daemon=True); cal_thread.start()
    def _perform_actual_still_calibration(self):
        success = False
        try:
            if not self.imu_api: raise Exception("IMU API not available.")
            self.root.after(0, lambda: self.log_status("Calibrating Gravity & Gyro...", tag='info'))
            self.imu_api.calibrate_gravity(); self.root.after(0, lambda: self.log_status("Gravity calibrated.", tag='success'))
            self.imu_api.calibrate_gyro(); self.root.after(0, lambda: self.log_status("Gyro calibrated.", tag='success')); success = True
        except (IMUCommunicationError, IMUCalibrationError, IMUTimeoutError) as e: self.root.after(0, self._sensor_calibration_failure, "Still", e)
        except Exception as e: self.root.after(0, self._sensor_calibration_failure, "Still", e); traceback.print_exc()
        finally: self.root.after(0, self._still_calibration_finished, success)
    def _still_calibration_finished(self, success: bool):
        self.still_calibrated = success
        if success: self.log_status("Still calibration successful.", tag='success'); self._show_calibration_button(self.motion_cal_button)
        else: self.still_cal_button.config(state=tk.NORMAL)
    def _do_motion_calibration_tk(self):
        if not self.imu_api: self.log_status("Error: APIs not initialized.", tag='error'); return
        try:
            delay_sec = self.imu_api.rotation_delay; delay_ms = int(delay_sec * 1000)
            prep_instruction = f"Prepare to rotate IMU through all axes. Starting in {delay_sec:.0f}s..."
            self.log_status("Preparing for Motion Calibration.", tag='info'); self.log_status(prep_instruction, tag='info')
            self.motion_cal_button.config(state=tk.DISABLED); self.root.after(delay_ms, self._schedule_motion_calibration_thread)
        except Exception as e: messagebox.showerror("Error", f"Error preparing motion cal: {str(e)}"); self.motion_cal_button.config(state=tk.NORMAL)
    def _schedule_motion_calibration_thread(self): cal_thread = threading.Thread(target=self._perform_actual_motion_calibration, daemon=True); cal_thread.start()
    def _perform_actual_motion_calibration(self):
        success = False
        try:
            if not self.imu_api: raise Exception("IMU API not available.")
            self.root.after(0, lambda: self.log_status("Calibrating Magnetometer...", tag='info'))
            self.root.after(0, lambda: self.log_status("Rotate IMU slowly for ~10s.", tag='info'))
            self.imu_api.calibrate_mag(); self.root.after(0, lambda: self.log_status("Motion Calibration Complete.", tag='success')); success = True
        except (IMUCommunicationError, IMUCalibrationError, IMUTimeoutError) as e: self.root.after(0, self._sensor_calibration_failure, "Motion", e)
        except Exception as e: self.root.after(0, self._sensor_calibration_failure, "Motion", e); traceback.print_exc()
        finally: self.root.after(0, self._motion_calibration_finished, success)
    def _motion_calibration_finished(self, success: bool):
        self.motion_calibrated = success
        if success: self.log_status("All calibration steps complete!", tag='success'); self._show_calibration_button(self.finish_calibration_button)
        else: self.motion_cal_button.config(state=tk.NORMAL)
    def _sensor_calibration_failure(self, cal_type: str, error: Exception):
        error_str = str(error)
        if cal_type == "Still": self.still_calibrated = False
        elif cal_type == "Motion": self.motion_calibrated = False
        self.log_status(f"{cal_type} Calibration Error: {error_str}", tag='error'); self.log_status("Please try again.", tag='info')
        messagebox.showerror(f"{cal_type} Calibration Error", f"{error_str}\nPlease try again.")

    # --- Data Collection Logic ---

    @staticmethod
    def _sanitize_filename(name: str) -> str:
        """Removes/replaces invalid characters for directory/file names."""
        if not name: name = "unnamed_scan"
        name = name.strip()
        # Replace spaces with underscores
        name = re.sub(r'\s+', '_', name)
        # Remove characters that are not alphanumeric, underscore, or hyphen
        name = re.sub(r'(?u)[^-\w]+', '', name)
        # Remove leading/trailing underscores/hyphens
        name = name.strip('_-')
        if not name: name = "sanitized_scan"
        return name

    def _start_data_collection(self):
        """Prompts for scan name and starts the data collection thread."""
        if self.is_collecting: self.log_status("Collection already running.", tag='warning'); return
        if self.current_state == STATE_CALIBRATING: messagebox.showwarning("Busy", "Finish calibration first."); return
        # --- Check DataCollector again before starting ---
        if not self.imu_api or DataCollector is None:
             print(f"DEBUG (_start_data_collection): Cannot start. IMU API Ready: {self.imu_api is not None}. DataCollector Ready: {DataCollector is not None}")
             messagebox.showerror("Error", "APIs not ready or DataCollector missing."); return
        # --- End Check ---

        # --- Prompt for Scan Name ---
        scan_name = simpledialog.askstring("Scan Name", "Enter a name/description for this scan:", parent=self.root)
        if not scan_name: # User cancelled or entered empty string
            self.log_status("Scan name not provided. Collection cancelled.", tag='warning')
            return
        # Use original name for description, sanitized name for folder is handled by DataManager
        # sanitized_name = self._sanitize_filename(scan_name)
        # self.log_status(f"Using scan name: {scan_name}", tag='info')
        # ---

        self.log_status("Preparing data collection process...", tag='collect')
        self.current_state = STATE_COLLECTING
        self.is_collecting = True

        # Disable buttons
        self.calibrate_button.config(state=tk.DISABLED)
        self.start_collection_button.config(state=tk.DISABLED) # Disable start btn itself
        self.start_collection_button.pack_forget()
        self.stop_collection_button.config(state=tk.NORMAL)
        self.stop_collection_button.pack(pady=15, ipadx=20, ipady=10)

        # Create DataCollector instance (moved here from thread)
        try:
             self.data_collector_instance = DataCollector(config_path='config.ini')
             # Prepare the session (creates folder, starts DB connection)
             if not self.data_collector_instance.prepare_session(session_name=scan_name): # Use original name
                  self.log_status("Failed to prepare data collection session.", tag='error')
                  messagebox.showerror("Error", "Failed to prepare session. Check logs and storage path.")
                  self._collection_finished_update() # Reset UI
                  return
        except Exception as e:
             self.log_status(f"Error initializing DataCollector or preparing session: {e}", tag='error')
             messagebox.showerror("Error", f"Failed to initialize collector: {e}")
             self._collection_finished_update() # Reset UI
             return

        # Reset stop event and start thread
        self.collector_stop_event.clear()
        self.collection_thread = threading.Thread(target=self._run_collection_thread,
                                                  args=(self.data_collector_instance,), # Pass instance
                                                  daemon=True)
        self.collection_thread.start()
        # Use the actual session name generated by DataManager if available
        actual_session_name = self.data_collector_instance.db_manager.current_session_name if self.data_collector_instance.db_manager else scan_name
        self.log_status(f"Data collection started for session: {actual_session_name}", tag='collect')


    def _stop_data_collection(self):
        """Signals the data collection thread to stop."""
        if not self.is_collecting or self.collection_thread is None:
            self.log_status("No active collection process to stop.", tag='warning'); return

        self.log_status("Sending stop signal to data collection process...", tag='collect')
        self.stop_collection_button.config(state=tk.DISABLED)
        self.collector_stop_event.set()

    def _run_collection_thread(self, collector_instance: DataCollector):
        """Target function for the background data collection thread."""
        self.log_status("Collection thread started.", tag='info')
        try:
            # Run the collector - it blocks until stop_event is set and cleanup is done
            collector_instance.run(self.collector_stop_event)
            self.log_status("Data collection process finished cleanly.", tag='success')
        except Exception as e:
            self.log_status(f"Data collection thread error: {e}", tag='error')
            traceback.print_exc()
        finally:
            self.log_status("Collection thread scheduling GUI update.", tag='info')
            if self.root.winfo_exists():
                 self.root.after(0, self._collection_finished_update)
            else:
                 print("Collection thread finished, but GUI root window no longer exists.")


    def _collection_finished_update(self):
        """Updates the GUI after the collection thread has finished."""
        self.log_status("Updating GUI after collection finished.", tag='info')
        self.is_collecting = False
        self.current_state = STATE_MAIN_MENU
        self.collection_thread = None
        self.data_collector_instance = None # Clear instance

        # Update button states
        self.stop_collection_button.pack_forget()

        # --- ADDED LOGGING before setting button state ---
        print(f"DEBUG (_collection_finished_update): Checking conditions before enabling buttons.")
        print(f"DEBUG: self.imu_api is None: {self.imu_api is None}")
        print(f"DEBUG: DataCollector is None: {DataCollector is None}")
        # --- END ADDED LOGGING ---

        if self.imu_api and DataCollector:
             self.start_collection_button.config(state=tk.NORMAL)
             self.calibrate_button.config(state=tk.NORMAL)
             print("DEBUG (_collection_finished_update): Enabling Start Collection and Calibrate buttons.")
        else:
             self.start_collection_button.config(state=tk.DISABLED)
             self.calibrate_button.config(state=tk.DISABLED)
             print("DEBUG (_collection_finished_update): Keeping Start Collection and Calibrate buttons DISABLED.")

        # Ensure start button is visible again
        if not self.start_collection_button.winfo_ismapped():
             self.start_collection_button.pack(pady=15, ipadx=20, ipady=10)


    # --- Navigation and Closing ---
    def _go_to_main_menu(self):
        """Returns to the main menu frame."""
        self.current_state = STATE_MAIN_MENU
        self._show_calibration_button(None) # Hide calibration buttons
        self._show_frame(self.main_menu_frame)

        # --- ADDED LOGGING before setting button state ---
        print(f"DEBUG (_go_to_main_menu): Checking conditions before setting button states.")
        print(f"DEBUG: self.imu_api is None: {self.imu_api is None}")
        print(f"DEBUG: DataCollector is None: {DataCollector is None}")
        # --- END ADDED LOGGING ---

        button_state_after_config = "unknown" # Initialize

        if self.imu_api:
             status_msg = "IMU API Ready."
             status_msg += " LiDAR Active." if self.imu_api.lidar and self.imu_api.lidar._is_connected else " LiDAR Inactive."
             self.log_status(f"Returned to Main Menu. {status_msg}", tag='info')
             self.calibrate_button.config(state=tk.NORMAL)
             if DataCollector:
                  self.start_collection_button.config(state=tk.NORMAL)
                  print("DEBUG (_go_to_main_menu): Enabling Start Collection button.")
                  # --- ADDED: Force update and check state ---
                  try:
                       print("DEBUG (_go_to_main_menu): Forcing GUI update...")
                       self.root.update_idletasks()
                       button_state_after_config = self.start_collection_button.cget('state')
                       print(f"DEBUG (_go_to_main_menu): Button state *after* config and update: {button_state_after_config}")
                  except Exception as e:
                       print(f"DEBUG (_go_to_main_menu): Error during update/cget: {e}")
                  # --- END ADDED ---
             else:
                  self.start_collection_button.config(state=tk.DISABLED)
                  print("DEBUG (_go_to_main_menu): DataCollector is None, keeping Start Collection button DISABLED.")
        else:
             self.log_status(f"Returned to Main Menu. APIs disconnected or failed.", tag='warning')
             self.calibrate_button.config(state=tk.DISABLED)
             self.start_collection_button.config(state=tk.DISABLED)
             print("DEBUG (_go_to_main_menu): IMU API is None, keeping Start Collection and Calibrate buttons DISABLED.")

        # Hide stop button and ensure start button is visible
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
                print("DEBUG: Waiting briefly for collection thread...")
                self.collection_thread.join(timeout=2.0)
                if self.collection_thread.is_alive(): print("DEBUG: Collection thread did not stop cleanly.")
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
         messagebox.showerror("Fatal Error", f"Failed to initialize GUI: {e}\n\nPlease check logs.")
         traceback.print_exc()
         try:
              if root and root.winfo_exists(): root.destroy()
         except: pass
    finally:
         print("DEBUG: Application finished.")

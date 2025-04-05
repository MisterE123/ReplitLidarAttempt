import tkinter as tk
from tkinter import ttk # Themed widgets
from tkinter import messagebox
import time
import sys
import threading # To run IMU init in background

# Import the API and specific exceptions from the existing file
# Make sure imu_api.py is in the same directory or Python path
try:
    from imu_api import IMUAPI, IMUCommunicationError, IMUCalibrationError, IMUTimeoutError
except ImportError:
    messagebox.showerror("Error", "Could not find imu_api.py. Make sure it's in the same directory.")
    sys.exit(1)

# Define states (though managed differently in Tkinter)
STATE_MAIN_MENU = 0
STATE_CALIBRATING = 1

class CalibrationGUI_Tk:
    def __init__(self, root):
        self.root = root
        self.root.title("IMU Calibration Tool (Tkinter)")
        # Make window slightly larger
        self.root.geometry("700x450")
        self.root.protocol("WM_DELETE_WINDOW", self._on_closing) # Handle window close

        self.imu = None
        self.imu_init_error = None
        self.is_calibrated = False # Basic flag
        self.current_state = STATE_MAIN_MENU # To control button states

        # --- Style ---
        self.style = ttk.Style()
        # Try to use a theme that looks better across platforms
        available_themes = self.style.theme_names()
        print(f"Available themes: {available_themes}") # Debug print
        if 'clam' in available_themes:
             self.style.theme_use('clam')
        elif 'alt' in available_themes:
             self.style.theme_use('alt')
        # Configure button style for padding
        self.style.configure('TButton', padding=6)

        # --- Main Frames ---
        # Use frames to switch between "pages" or states
        self.main_menu_frame = ttk.Frame(root, padding="10")
        self.calibration_frame = ttk.Frame(root, padding="10")

        # --- Status Display ---
        # Place status at the bottom, spanning across columns if using grid
        self.status_var = tk.StringVar()
        self.status_label = ttk.Label(root, textvariable=self.status_var, relief=tk.SUNKEN, anchor=tk.W, padding="5")
        self.status_label.pack(side=tk.BOTTOM, fill=tk.X, padx=5, pady=5)
        self.set_status("Initializing...", "") # Initial status

        # --- Instructions Display ---
        self.instructions_var = tk.StringVar()
        self.instructions_label = ttk.Label(root, textvariable=self.instructions_var, relief=tk.SUNKEN, anchor=tk.W, padding="5")
        self.instructions_label.pack(side=tk.BOTTOM, fill=tk.X, padx=5, pady=5)

        # --- Setup UI Elements ---
        self._setup_main_menu_frame()
        self._setup_calibration_frame()

        # --- Start IMU Initialization in Background ---
        # This prevents the GUI from freezing during the initial connection attempt
        self.set_status("Connecting to IMU...", "Please wait...")
        self.init_thread = threading.Thread(target=self._initialize_imu, daemon=True)
        self.init_thread.start()
        # Check periodically if the init thread is done
        self.root.after(100, self._check_imu_init)

    def _initialize_imu(self):
        """Runs in a separate thread to avoid blocking the GUI."""
        print("DEBUG: Background thread: Attempting to initialize IMUAPI...")
        try:
            # This might block or take time
            self.imu = IMUAPI()
            print("DEBUG: Background thread: IMUAPI initialization successful.")
        except (FileNotFoundError, KeyError, ValueError, IMUCommunicationError) as e:
            self.imu = None
            self.imu_init_error = f"Error initializing IMU: {str(e)}"
            print(f"DEBUG: Background thread: IMUAPI Initialization Error: {str(e)}")
        except Exception as e:
            self.imu = None
            self.imu_init_error = f"Unexpected Error initializing IMU: {str(e)}"
            print(f"DEBUG: Background thread: IMUAPI Unexpected Initialization Error: {str(e)}")

    def _check_imu_init(self):
        """Checks the result of the IMU initialization thread."""
        if self.init_thread.is_alive():
            # Still running, check again later
            self.root.after(100, self._check_imu_init)
        else:
            # Thread finished
            if self.imu:
                self.set_status("IMU Connected. Ready.", "")
                # Enable the main calibrate button now that IMU is ready
                self.calibrate_button.config(state=tk.NORMAL)
                # Start by showing the main menu
                self._show_frame(self.main_menu_frame)
            else:
                # Update status with the error found in the background thread
                self.set_status(self.imu_init_error or "Failed to connect to IMU.",
                                "Please check config.ini and connection.")
                # Keep calibrate button disabled
                self.calibrate_button.config(state=tk.DISABLED)
                # Show main menu frame even on error, but button is disabled
                self._show_frame(self.main_menu_frame)

    def _setup_main_menu_frame(self):
        """Creates widgets for the main menu."""
        # Clear the frame first if needed (though it's empty initially)
        for widget in self.main_menu_frame.winfo_children():
            widget.destroy()

        label = ttk.Label(self.main_menu_frame, text="Main Menu", font=("Helvetica", 16))
        label.pack(pady=20)

        # Calibrate Button - Initially disabled, enabled when IMU connects
        self.calibrate_button = ttk.Button(
            self.main_menu_frame,
            text="Calibrate IMU",
            command=self._go_to_calibration,
            state=tk.DISABLED # Enabled by _check_imu_init on success
        )
        self.calibrate_button.pack(pady=30, ipadx=20, ipady=10) # Make button larger

        # Quit Button
        quit_button = ttk.Button(
            self.main_menu_frame,
            text="Quit",
            command=self._on_closing
        )
        quit_button.pack(pady=10)


    def _setup_calibration_frame(self):
        """Creates widgets for the calibration menu."""
         # Clear the frame first if needed
        for widget in self.calibration_frame.winfo_children():
            widget.destroy()

        label = ttk.Label(self.calibration_frame, text="Calibration Steps", font=("Helvetica", 16))
        label.pack(pady=20)

        # Still Calibration Button
        self.still_cal_button = ttk.Button(
            self.calibration_frame,
            text="1. Calibrate Still (Gravity+Gyro)",
            command=self._do_still_calibration_tk
        )
        self.still_cal_button.pack(pady=10, fill=tk.X, padx=50)

        # Motion Calibration Button
        self.motion_cal_button = ttk.Button(
            self.calibration_frame,
            text="2. Calibrate Motion (Magnetometer)",
            command=self._do_motion_calibration_tk
        )
        self.motion_cal_button.pack(pady=10, fill=tk.X, padx=50)

        # Done Button
        self.done_button = ttk.Button(
            self.calibration_frame,
            text="Done (Back to Main Menu)",
            command=self._go_to_main_menu
        )
        self.done_button.pack(pady=30, fill=tk.X, padx=50)

        # Initially disable calibration buttons until time is calibrated
        self._set_calibration_buttons_state(tk.DISABLED)


    def _show_frame(self, frame_to_show):
        """Hides other frames and shows the specified frame."""
        self.main_menu_frame.pack_forget()
        self.calibration_frame.pack_forget()
        frame_to_show.pack(fill=tk.BOTH, expand=True)

    def set_status(self, status: str, instructions: str = ""):
        """Updates the status and instruction labels."""
        print(f"GUI Status: {status} | Instructions: {instructions}") # Also print to console
        self.status_var.set(f"Status: {status}")
        self.instructions_var.set(f"Instructions: {instructions}")
        # self.root.update_idletasks() # Force GUI update if needed, but usually automatic

    def _set_calibration_buttons_state(self, state):
        """Enable or disable calibration step buttons."""
        self.still_cal_button.config(state=state)
        self.motion_cal_button.config(state=state)
        self.done_button.config(state=state) # Also control done button

    def _go_to_calibration(self):
        """Callback for the main 'Calibrate IMU' button."""
        if not self.imu:
            self.set_status("Error: IMU not connected.", "Cannot calibrate.")
            return

        print("DEBUG: Calibrate button clicked. Attempting time calibration...")
        self.set_status("Calibrating time...", "Communicating with IMU...")
        self.calibrate_button.config(state=tk.DISABLED) # Disable while attempting

        try:
            # Time calibration is usually quick, do it directly
            offset = self.imu.calibrate_time()
            self.set_status(f"Time calibration successful.", f"IMU Offset: {offset} μs. Choose next step.")
            self.current_state = STATE_CALIBRATING
            self._set_calibration_buttons_state(tk.NORMAL) # Enable calibration step buttons
            self._show_frame(self.calibration_frame) # Switch to calibration frame

        except (IMUCommunicationError, IMUTimeoutError) as e:
            self.set_status(f"Time calibration failed: {str(e)}", "Check connection and retry.")
            messagebox.showerror("Time Calibration Failed", f"{str(e)}\nCheck connection and retry.")
            self.calibrate_button.config(state=tk.NORMAL) # Re-enable button on failure
        except Exception as e:
            self.set_status(f"Unexpected Error during time calibration: {str(e)}", "Check logs.")
            messagebox.showerror("Time Calibration Error", f"An unexpected error occurred:\n{str(e)}")
            self.calibrate_button.config(state=tk.NORMAL) # Re-enable button on failure

    def _go_to_main_menu(self):
        """Callback for the 'Done' button in calibration frame."""
        self.current_state = STATE_MAIN_MENU
        self._show_frame(self.main_menu_frame)
        # Re-enable main calibrate button if IMU is still connected
        if self.imu:
             self.calibrate_button.config(state=tk.NORMAL)
             self.set_status("IMU Connected. Ready.", "")
        else:
             self.calibrate_button.config(state=tk.DISABLED)
             self.set_status(self.imu_init_error or "IMU Disconnected.", "Cannot calibrate.")
        self.set_status("Returned to Main Menu.", "")


    # --- Calibration Methods (Tkinter specific) ---

    def _do_still_calibration_tk(self):
        """Handles the 'Still Calibration' button click."""
        if not self.imu:
            self.set_status("Error: IMU not connected.")
            messagebox.showerror("Error", "IMU not connected.")
            return

        try:
            delay_sec = self.imu.still_delay
            delay_ms = int(delay_sec * 1000)
            instruction = f"Place device flat and still. Waiting {delay_sec:.1f}s..."
            self.set_status("Preparing for Still Calibration.", instruction)
            self._set_calibration_buttons_state(tk.DISABLED) # Disable buttons during wait/cal

            # Schedule the actual calibration after the delay
            self.root.after(delay_ms, self._perform_actual_still_calibration)

        except AttributeError:
            self.set_status("Error: Could not find delay settings.", "Check IMU API/config.")
            messagebox.showerror("Error", "Could not find delay settings in IMU API/config.")
            self._set_calibration_buttons_state(tk.NORMAL) # Re-enable on error
        except Exception as e:
            self.set_status(f"Unexpected Error preparing still calibration: {str(e)}", "Check logs.")
            messagebox.showerror("Error", f"Unexpected error: {str(e)}")
            self._set_calibration_buttons_state(tk.NORMAL) # Re-enable on error

    def _perform_actual_still_calibration(self):
        """Called by root.after() to perform the still calibration steps."""
        try:
            # Gravity Calibration
            self.set_status("Calibrating Gravity...", "Keep device perfectly still.")
            self.root.update_idletasks() # Ensure status update is visible
            self.imu.calibrate_gravity()
            self.set_status("Gravity calibrated.", "Starting Gyro calibration (keep still)...")
            self.root.update_idletasks() # Ensure status update is visible

            # Gyro Calibration (immediately after gravity)
            self.imu.calibrate_gyro()
            self.set_status("Still Calibration Complete.", "Gravity & Gyro calibrated.")
            self.is_calibrated = True # Example flag

        except (IMUCommunicationError, IMUCalibrationError, IMUTimeoutError) as e:
            self.set_status(f"Still Calibration Error: {str(e)}", "Please try again.")
            messagebox.showerror("Still Calibration Error", f"{str(e)}\nPlease try again.")
        except Exception as e:
            self.set_status(f"Unexpected Error during still calibration: {str(e)}", "Check logs.")
            messagebox.showerror("Still Calibration Error", f"An unexpected error occurred:\n{str(e)}")
        finally:
            # Re-enable buttons regardless of success or failure
             self._set_calibration_buttons_state(tk.NORMAL)


    def _do_motion_calibration_tk(self):
        """Handles the 'Motion Calibration' button click."""
        if not self.imu:
            self.set_status("Error: IMU not connected.")
            messagebox.showerror("Error", "IMU not connected.")
            return

        try:
            delay_sec = self.imu.rotation_delay
            delay_ms = int(delay_sec * 1000)
            prep_instruction = f"Prepare to rotate IMU through all axes. Starting in {delay_sec:.0f}s..."
            self.set_status("Preparing for Motion Calibration.", prep_instruction)
            self._set_calibration_buttons_state(tk.DISABLED) # Disable buttons during wait/cal

            # Schedule the actual calibration after the delay
            self.root.after(delay_ms, self._perform_actual_motion_calibration)

        except AttributeError:
            self.set_status("Error: Could not find delay settings.", "Check IMU API/config.")
            messagebox.showerror("Error", "Could not find delay settings in IMU API/config.")
            self._set_calibration_buttons_state(tk.NORMAL) # Re-enable on error
        except Exception as e:
            self.set_status(f"Unexpected Error preparing motion calibration: {str(e)}", "Check logs.")
            messagebox.showerror("Error", f"Unexpected error: {str(e)}")
            self._set_calibration_buttons_state(tk.NORMAL) # Re-enable on error

    def _perform_actual_motion_calibration(self):
        """Called by root.after() to perform the motion calibration step."""
        try:
            self.set_status("Calibrating Magnetometer...", "Rotate IMU slowly in all directions for ~10s.")
            self.root.update_idletasks() # Ensure status update is visible
            self.imu.calibrate_mag() # API handles the timing internally
            self.set_status("Motion Calibration Complete.", "Magnetometer calibrated.")
            self.is_calibrated = True # Example flag

        except (IMUCommunicationError, IMUCalibrationError, IMUTimeoutError) as e:
            self.set_status(f"Motion Calibration Error: {str(e)}", "Please try again.")
            messagebox.showerror("Motion Calibration Error", f"{str(e)}\nPlease try again.")
        except Exception as e:
            self.set_status(f"Unexpected Error during motion calibration: {str(e)}", "Check logs.")
            messagebox.showerror("Motion Calibration Error", f"An unexpected error occurred:\n{str(e)}")
        finally:
            # Re-enable buttons regardless of success or failure
            self._set_calibration_buttons_state(tk.NORMAL)


    def _on_closing(self):
        """Handles the window close event."""
        print("DEBUG: Window closing...")
        if self.imu:
            print("Closing serial port...")
            try:
                self.imu.close()
            except Exception as e:
                print(f"Error closing IMU port: {e}") # Log error but continue closing GUI
        self.root.destroy() # Close the Tkinter window


# --- Main Execution ---
if __name__ == "__main__":
    print("DEBUG: Creating Tkinter root window...")
    root = tk.Tk()
    print("DEBUG: Creating CalibrationGUI_Tk instance...")
    app = CalibrationGUI_Tk(root)
    print("DEBUG: Starting Tkinter main loop...")
    root.mainloop()
    print("DEBUG: Tkinter main loop finished.")


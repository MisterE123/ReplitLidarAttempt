import pygame
import sys
import time
# Import the API and specific exceptions
from imu_api import IMUAPI, IMUCommunicationError, IMUCalibrationError, IMUTimeoutError

# Define states
STATE_MAIN_MENU = 0
STATE_CALIBRATING = 1

class CalibrationGUI:
    def __init__(self):
        print("DEBUG: Initializing Pygame...") # ADDED
        pygame.init()
        self.screen_width = 800
        self.screen_height = 600
        self.screen = pygame.display.set_mode((self.screen_width, self.screen_height))
        pygame.display.set_caption("IMU Calibration Tool")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.Font(None, 36)
        self.font_small = pygame.font.Font(None, 28) # For instructions

        self.status = "Connecting to IMU..."
        self.instructions = "" # Separate line for instructions
        self.is_calibrated = False # Flag to indicate calibration status (basic implementation)
        self.current_state = STATE_MAIN_MENU

        print("DEBUG: Attempting to initialize IMUAPI...") # ADDED
        try:
            # Attempt to create an instance of the IMU API
            # This is where the connection and initial communication happens
            self.imu = IMUAPI()
            print("DEBUG: IMUAPI initialization successful.") # ADDED
            self.status = "IMU Connected. Ready."
        except (FileNotFoundError, KeyError, ValueError, IMUCommunicationError) as e:
            # Handle specific errors during IMU initialization
            self.imu = None
            print(f"DEBUG: IMUAPI Initialization Error: {str(e)}") # ADDED
            self.status = f"Error initializing IMU: {str(e)}"
            self.instructions = "Please check config.ini and connection."
        except Exception as e:
            # Handle any other unexpected errors during initialization
            self.imu = None
            print(f"DEBUG: IMUAPI Unexpected Initialization Error: {str(e)}") # ADDED
            self.status = f"Unexpected Error: {str(e)}"


        # Define Buttons (position/size will be set in drawing logic)
        # Buttons are stored in a dictionary, specific to the current state
        self.buttons = {}

    def draw_button(self, text: str, rect: pygame.Rect, enabled: bool = True) -> pygame.Rect:
        """Draws a button and returns its Rect."""
        # Set button color based on enabled status
        color = (100, 100, 100) if enabled else (50, 50, 50)
        text_color = (255, 255, 255) if enabled else (150, 150, 150)
        # Draw the button rectangle
        pygame.draw.rect(self.screen, color, rect)
        # Render the button text
        text_surf = self.font.render(text, True, text_color)
        text_rect = text_surf.get_rect(center=rect.center)
        # Draw the text onto the screen
        self.screen.blit(text_surf, text_rect)
        return rect

    def draw_status_and_instructions(self):
        """Draws the status and instruction lines at the bottom."""
        # Render and draw the status text
        status_text = self.font.render(f"Status: {self.status}", True, (255, 255, 255))
        self.screen.blit(status_text, (20, self.screen_height - 80))

        # Render and draw the instruction text
        instruction_text = self.font_small.render(self.instructions, True, (200, 200, 200))
        self.screen.blit(instruction_text, (20, self.screen_height - 45))


    def set_status(self, status: str, instructions: str = ""):
        """Helper function to update status and instructions, and print to console."""
        print(f"GUI Status: {status} | Instructions: {instructions}") # Also print to console
        self.status = status
        self.instructions = instructions
        pygame.display.flip() # Update display immediately to show status change

    # --- Calibration Methods ---
    # These methods encapsulate the calls to the IMU API for different calibration steps.

    def _do_time_calibration(self) -> bool:
        """Attempts time calibration. Returns True on success, False on failure."""
        if not self.imu:
            self.set_status("Error: IMU not connected.", "Cannot calibrate.")
            return False
        try:
            self.set_status("Calibrating time...", "Communicating with IMU...")
            # Call the IMU API method for time calibration
            offset = self.imu.calibrate_time()
            self.set_status(f"Time calibration successful.", f"IMU Offset: {offset} μs. Choose next step.")
            return True
        except (IMUCommunicationError, IMUTimeoutError) as e:
            # Handle specific communication or timeout errors
            self.set_status(f"Time calibration failed: {str(e)}", "Check connection and retry.")
            return False
        except Exception as e:
            # Handle unexpected errors
            self.set_status(f"Unexpected Error: {str(e)}", "Check connection/logs.")
            return False

    def _do_still_calibration(self):
        """Performs gravity and gyro calibration (requires device to be still)."""
        if not self.imu:
            self.set_status("Error: IMU not connected.")
            return
        try:
            # Provide instructions and wait for the user
            instruction = f"Place device flat and still. Waiting {self.imu.still_delay:.1f}s..."
            self.set_status("Preparing for Still Calibration.", instruction)
            pygame.time.wait(int(self.imu.still_delay * 1000)) # Use pygame wait

            # Calibrate Gravity
            self.set_status("Calibrating Gravity...", "Keep device perfectly still.")
            self.imu.calibrate_gravity()
            self.set_status("Gravity calibrated.", "Starting Gyro calibration (keep still)...")

            # Calibrate Gyro (immediately after gravity)
            self.imu.calibrate_gyro()
            self.set_status("Still Calibration Complete.", "Gravity & Gyro calibrated.")
            self.is_calibrated = True # Example: Set flag after a successful calibration part

        except (IMUCommunicationError, IMUCalibrationError, IMUTimeoutError) as e:
            # Handle specific calibration or communication errors
            self.set_status(f"Still Calibration Error: {str(e)}", "Please try again.")
        except AttributeError:
            # Handle case where delay settings might be missing
            self.set_status("Error: Could not find delay settings.", "Check IMU API/config.")
        except Exception as e:
            # Handle unexpected errors
            self.set_status(f"Unexpected Error: {str(e)}", "Check logs.")

    def _do_motion_calibration(self):
        """Performs magnetometer calibration (requires device rotation)."""
        if not self.imu:
            self.set_status("Error: IMU not connected.")
            return
        try:
            # Provide instructions and wait before starting
            prep_instruction = f"Prepare to rotate IMU through all axes. Starting in {self.imu.rotation_delay:.0f}s..."
            self.set_status("Preparing for Motion Calibration.", prep_instruction)
            # Use pygame event loop during wait to keep window responsive
            start_wait = pygame.time.get_ticks()
            while pygame.time.get_ticks() - start_wait < self.imu.rotation_delay * 1000:
                 for event in pygame.event.get():
                      if event.type == pygame.QUIT:
                           self.quit_app()
                 # Keep drawing status and instructions during wait
                 self.screen.fill((50, 50, 50)) # Redraw background
                 self.draw_status_and_instructions()
                 pygame.display.flip() # Keep display updated
                 self.clock.tick(30) # Small tick rate during wait

            # Start magnetometer calibration
            self.set_status("Calibrating Magnetometer...", "Rotate IMU slowly in all directions for ~10s.")
            self.imu.calibrate_mag() # API handles the timing internally
            self.set_status("Motion Calibration Complete.", "Magnetometer calibrated.")
            self.is_calibrated = True # Example: Set flag after a successful calibration part

        except (IMUCommunicationError, IMUCalibrationError, IMUTimeoutError) as e:
            # Handle specific calibration or communication errors
            self.set_status(f"Motion Calibration Error: {str(e)}", "Please try again.")
        except AttributeError:
            # Handle case where delay settings might be missing
            self.set_status("Error: Could not find delay settings.", "Check IMU API/config.")
        except Exception as e:
            # Handle unexpected errors
            self.set_status(f"Unexpected Error: {str(e)}", "Check logs.")

    def quit_app(self):
        """Handles closing the application gracefully."""
        print("Quitting application...") # ADDED Debug print
        if self.imu:
            print("Closing serial port...")
            self.imu.close()
        pygame.quit()
        sys.exit()

    # --- State Drawing and Handling ---
    # Methods to draw UI elements and handle events specific to each state.

    def draw_main_menu(self):
        """Draws the main menu UI elements."""
        self.buttons = {} # Clear buttons from other states
        button_width = 300
        button_height = 60
        x_pos = (self.screen_width - button_width) // 2
        y_pos = (self.screen_height - button_height) // 2 - 50 # Center vertically

        # Draw the "Calibrate IMU" button
        calibrate_rect = pygame.Rect(x_pos, y_pos, button_width, button_height)
        # Enable button only if IMU is connected
        self.buttons['calibrate'] = self.draw_button("Calibrate IMU", calibrate_rect, enabled=(self.imu is not None))

    def handle_main_menu_event(self, event):
        """Handles events specific to the main menu state."""
        if event.type == pygame.MOUSEBUTTONDOWN:
            # Check if the calibrate button was clicked and is enabled
            if 'calibrate' in self.buttons and self.buttons['calibrate'].collidepoint(event.pos) and self.imu:
                print("DEBUG: Calibrate button clicked.") # ADDED
                # Attempt time calibration before moving to the calibration menu
                if self._do_time_calibration():
                     self.current_state = STATE_CALIBRATING
                     # Reset status for the new state
                     self.set_status("Calibration Menu", "Choose calibration type or Done.")
                else:
                     print("DEBUG: Time calibration failed, staying in main menu.") # ADDED


    def draw_calibration_menu(self):
        """Draws the calibration menu UI elements."""
        self.buttons = {} # Clear buttons from other states
        button_width = 600 # Wider buttons for this menu
        button_height = 60
        x_pos = (self.screen_width - button_width) // 2
        y_start = 100 # Starting Y position for the first button
        spacing = 80 # Vertical spacing between buttons

        # Draw "Calibrate Still" button
        still_rect = pygame.Rect(x_pos, y_start, button_width, button_height)
        self.buttons['still'] = self.draw_button("Calibrate Still (Gravity+Gyro)", still_rect)

        # Draw "Calibrate Motion" button
        motion_rect = pygame.Rect(x_pos, y_start + spacing, button_width, button_height)
        self.buttons['motion'] = self.draw_button("Calibrate Motion (Magnetometer)", motion_rect)

        # Draw "Done" button
        done_rect = pygame.Rect(x_pos, y_start + 2 * spacing + 20, button_width, button_height) # Add extra space before Done
        self.buttons['done'] = self.draw_button("Done (Back to Main Menu)", done_rect)

    def handle_calibration_menu_event(self, event):
         """Handles events specific to the calibration menu state."""
         if event.type == pygame.MOUSEBUTTONDOWN:
            pos = event.pos
            # Check which button was clicked
            if 'still' in self.buttons and self.buttons['still'].collidepoint(pos):
                print("DEBUG: Still calibration button clicked.") # ADDED
                self._do_still_calibration()
            elif 'motion' in self.buttons and self.buttons['motion'].collidepoint(pos):
                print("DEBUG: Motion calibration button clicked.") # ADDED
                self._do_motion_calibration()
            elif 'done' in self.buttons and self.buttons['done'].collidepoint(pos):
                print("DEBUG: Done button clicked.") # ADDED
                self.current_state = STATE_MAIN_MENU
                # Reset status message for the main menu
                self.set_status("IMU Connected. Ready.", "")


    # --- Main Loop ---

    def run(self):
        """The main application loop."""
        print("DEBUG: Starting main loop...") # ADDED
        while True:
            # --- Event Handling ---
            for event in pygame.event.get():
                # Handle quit event (closing the window)
                if event.type == pygame.QUIT:
                    self.quit_app()

                # Handle events based on the current state
                if self.current_state == STATE_MAIN_MENU:
                    self.handle_main_menu_event(event)
                elif self.current_state == STATE_CALIBRATING:
                    self.handle_calibration_menu_event(event)

            # --- Drawing ---
            self.screen.fill((50, 50, 50)) # Fill background with dark grey

            # Draw UI elements based on the current state
            if self.current_state == STATE_MAIN_MENU:
                self.draw_main_menu()
            elif self.current_state == STATE_CALIBRATING:
                self.draw_calibration_menu()

            # Draw status and instructions at the bottom (common to all states)
            self.draw_status_and_instructions()

            # --- Update Display ---
            pygame.display.flip() # Update the full screen to show drawn elements

            # --- Frame Rate Control ---
            self.clock.tick(60) # Limit frame rate to 60 FPS

# Entry point of the script
if __name__ == "__main__":
    print("DEBUG: Creating CalibrationGUI instance...") # ADDED
    gui = CalibrationGUI()
    print("DEBUG: Calling gui.run()...") # ADDED
    gui.run()

import time
import math
import matplotlib.pyplot as plt
import sys
import os
from typing import Optional # Added for type hinting lidar

# --- Path Modification ---
# Add the parent directory (e.g., 'python/') to the Python path
# This allows importing 'lidar_api' when this script is in a subfolder (e.g., 'python/tests/')
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if parent_dir not in sys.path:
    print(f"DEBUG: Adding parent directory to sys.path: {parent_dir}")
    sys.path.insert(0, parent_dir)
# --- End Path Modification ---

# Assuming lidar_api.py is in the parent directory
try:
    from lidar_api import LidarAPI, LidarCommunicationError, LidarPacketError
except ImportError:
    print("ERROR: Could not import LidarAPI from parent directory.")
    print("Make sure lidar_api.py is in the directory containing the 'tests' folder.")
    print(f"Current sys.path: {sys.path}")
    sys.exit(1)

class LidarDataCollector:
    """
    Collects and plots LiDAR data using the LidarAPI.
    """
    def __init__(self, duration: float = 10.0, port: str = '/dev/ttyUSB0', baudrate: int = 230400, max_radius: float = 10.0):
        """
        Initialize the data collector.

        Args:
            duration: Duration in seconds to collect data.
            port: Serial port of the LiDAR sensor.
            baudrate: Baud rate for the LiDAR sensor.
            max_radius: Maximum distance (in meters) to include in the plot.
        """
        self.duration = duration
        self.port = port
        self.baudrate = baudrate
        self.max_radius = max_radius
        self.data: list[tuple[float, float]] = [] # List to store (angle_degrees, distance_meters)
        self.lidar: Optional[LidarAPI] = None

    def collect_data(self):
        """
        Connect to LiDAR, collect data for the specified duration using LidarAPI,
        and store valid points.
        """
        print(f"Initializing LidarAPI for port {self.port} at {self.baudrate} baud...")
        # Ensure LidarAPI was successfully imported before instantiating
        if 'LidarAPI' not in globals():
             print("ERROR: LidarAPI class not available due to import error.")
             return
        self.lidar = LidarAPI(port=self.port, baudrate=self.baudrate)
        last_processed_packet_ts = -1 # Timestamp (ns) of the last packet processed

        try:
            print("Connecting to LiDAR...")
            self.lidar.connect()
            print(f"LiDAR connected. Collecting data for {self.duration:.1f} seconds...")
            start_time = time.time()

            while time.time() - start_time < self.duration:
                # Get the latest packet data from the background thread
                latest_packet_info = self.lidar.get_latest_packet()

                if latest_packet_info:
                    pi_timestamp_ns, parsed_data = latest_packet_info

                    # Avoid processing the same packet multiple times if the loop runs fast
                    if pi_timestamp_ns > last_processed_packet_ts:
                        last_processed_packet_ts = pi_timestamp_ns
                        # print(f"Processing packet with Pi timestamp: {pi_timestamp_ns / 1e9:.3f}") # Optional debug

                        points = parsed_data.get('points', [])
                        for point in points:
                            distance = point.get('distance', float('inf'))
                            angle = point.get('angle') # Angle should always exist if point exists

                            # Ensure distance is valid and within radius
                            if angle is not None and 0 < distance <= self.max_radius:
                                self.data.append((angle, distance)) # Store angle in degrees

                # Sleep briefly to prevent busy-waiting and allow other threads
                time.sleep(0.01) # Adjust sleep time as needed

            print(f"Finished collecting data. Total points acquired: {len(self.data)}")

        except LidarCommunicationError as e:
            print(f"ERROR: Communication error during data collection: {e}")
        except Exception as e:
            print(f"ERROR: Unexpected error during data collection: {e}")
            import traceback
            traceback.print_exc() # Print full traceback for unexpected errors
        finally:
            if self.lidar:
                print("Disconnecting LiDAR...")
                self.lidar.disconnect()
                print("LiDAR disconnected.")

    def plot_data(self):
        """
        Plot the collected LiDAR data as a 2D scatter plot.
        (This method remains unchanged from the original disp_lidar.py)
        """
        if not self.data:
            print("No data collected to plot.")
            return

        # Convert degrees to radians for plotting
        angles_rad = [math.radians(angle) for angle, distance in self.data]
        distances = [distance for angle, distance in self.data]

        # Convert polar coordinates (angle, distance) to Cartesian (x, y)
        # Assuming angle 0 is along the positive X-axis, increasing counter-clockwise
        # Adjust signs if your LiDAR's coordinate system is different
        x = [distance * math.cos(angle) for angle, distance in zip(angles_rad, distances)]
        # Use negative sin for Y if Y increases downwards in the desired plot
        # Or positive sin if Y increases upwards
        # Original script used negative:
        y = [-distance * math.sin(angle) for angle, distance in zip(angles_rad, distances)]

        print("Plotting data...")
        plt.figure(figsize=(8, 8))
        plt.scatter(x, y, s=5, c='blue', alpha=0.5) # Small points, semi-transparent
        plt.title("LiDAR Data Visualization (using LidarAPI)")
        plt.xlabel("X (meters)")
        plt.ylabel("Y (meters)")
        plt.axis('equal') # Ensure aspect ratio is equal
        plt.grid(True)
        # Set plot limits based on max_radius for better visualization
        plt.xlim(-self.max_radius * 1.1, self.max_radius * 1.1)
        plt.ylim(-self.max_radius * 1.1, self.max_radius * 1.1)
        plt.show()


if __name__ == "__main__":
    # --- Configuration ---
    LIDAR_PORT = '/dev/ttyUSB0' # CHANGE TO YOUR LIDAR PORT
    LIDAR_BAUDRATE = 230400     # CHANGE TO YOUR LIDAR BAUDRATE
    COLLECTION_DURATION_S = 10 # Duration to collect data
    MAX_PLOT_RADIUS_M = 10.0   # Max distance to plot

    # --- Execution ---
    collector = LidarDataCollector(
        duration=COLLECTION_DURATION_S,
        port=LIDAR_PORT,
        baudrate=LIDAR_BAUDRATE,
        max_radius=MAX_PLOT_RADIUS_M
    )

    collector.collect_data()
    collector.plot_data()

    print("Script finished.")

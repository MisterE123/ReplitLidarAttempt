
import tkinter as tk
from tkinter import ttk, messagebox
from imu_api import IMUAPI

class CalibrationGUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("IMU Calibration")
        self.imu = IMUAPI()
        self.setup_ui()
        
    def setup_ui(self):
        frame = ttk.Frame(self.root, padding="10")
        frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        ttk.Button(frame, text="Time Calibration", command=self.calibrate_time).grid(row=0, column=0, pady=5)
        ttk.Button(frame, text="Gravity Calibration", command=self.calibrate_gravity).grid(row=1, column=0, pady=5)
        ttk.Button(frame, text="Gyro Calibration", command=self.calibrate_gyro).grid(row=2, column=0, pady=5)
        ttk.Button(frame, text="Mag Calibration", command=self.calibrate_mag).grid(row=3, column=0, pady=5)

    def calibrate_time(self):
        try:
            offset = self.imu.calibrate_time()
            messagebox.showinfo("Success", f"Time offset: {offset} microseconds")
        except Exception as e:
            messagebox.showerror("Error", str(e))

    def calibrate_gravity(self):
        try:
            gx, gy, gz = self.imu.calibrate_gravity()
            messagebox.showinfo("Success", f"Gravity vector: [{gx:.2f}, {gy:.2f}, {gz:.2f}]")
        except Exception as e:
            messagebox.showerror("Error", str(e))

    def calibrate_gyro(self):
        try:
            self.imu.calibrate_gyro()
            messagebox.showinfo("Success", "Gyroscope calibration complete")
        except Exception as e:
            messagebox.showerror("Error", str(e))

    def calibrate_mag(self):
        try:
            self.imu.calibrate_mag()
            messagebox.showinfo("Success", "Magnetometer calibration complete")
        except Exception as e:
            messagebox.showerror("Error", str(e))

    def run(self):
        self.root.mainloop()

if __name__ == "__main__":
    gui = CalibrationGUI()
    gui.run()

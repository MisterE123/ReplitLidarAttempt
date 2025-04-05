# minimal_tkinter_test.py
import tkinter as tk
import sys

print("Attempting tk.Tk()...")
try:
    # This is the line that seems to be hanging in your main script
    root = tk.Tk()
    print("tk.Tk() successful.")
    # Immediately close the window if created successfully
    root.destroy()
    print("Root window destroyed.")
except Exception as e:
    print(f"tk.Tk() failed: {e}")
    sys.exit() # Exit if it fails with an error

print("Minimal Tkinter test finished.")
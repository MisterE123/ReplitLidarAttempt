
from flask import Flask, jsonify
from imu_calibration import IMUCalibration
import threading
import queue
import json

app = Flask(__name__)
imu = None
data_queue = queue.Queue()

@app.route('/calibrate/time')
def calibrate_time():
    offset = imu.time_calibration()
    return jsonify({'offset_us': offset})

@app.route('/calibrate/gravity')
def calibrate_gravity():
    x, y, z = imu.gravity_calibration()
    return jsonify({'x': x, 'y': y, 'z': z})

@app.route('/calibrate/gyro')
def calibrate_gyro():
    imu.gyro_calibration()
    return jsonify({'status': 'success'})

@app.route('/calibrate/mag')
def calibrate_mag():
    imu.mag_calibration()
    return jsonify({'status': 'success'})

@app.route('/start')
def start_collection():
    imu.ser.write(b'start_collection\n')
    return jsonify({'status': 'started'})

@app.route('/stop')
def stop_collection():
    imu.ser.write(b'stop_collection\n')
    return jsonify({'status': 'stopped'})

if __name__ == '__main__':
    imu = IMUCalibration()
    app.run(host='0.0.0.0', port=5000)


from flask import Flask, jsonify
from imu_calibration import IMUCalibration
import time

app = Flask(__name__)
imu = IMUCalibration()

@app.route('/calibrate/time')
def calibrate_time():
    offset = imu.time_calibration()
    return jsonify({'offset_us': offset})

@app.route('/calibrate/gravity')
def calibrate_gravity():
    gx, gy, gz = imu.gravity_calibration()
    return jsonify({'gravity_vector': [gx, gy, gz]})

@app.route('/calibrate/gyro')
def calibrate_gyro():
    imu.gyro_calibration()
    return jsonify({'status': 'success'})

@app.route('/calibrate/mag')
def calibrate_mag():
    imu.mag_calibration()
    return jsonify({'status': 'success'})

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)

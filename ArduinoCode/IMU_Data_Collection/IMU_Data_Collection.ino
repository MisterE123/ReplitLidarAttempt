#include <MKRIMU.h>

// States
enum State {
  READY,
  TIME_CALIBRATING,
  GRAVITY_CALIBRATING,
  COLLECTING_DATA
};

State currentState = READY;

// Buffer size based on actual IMU capabilities
// For 100ms collection period:
// - Accelerometer: ~100Hz = 10 readings
// - Gyroscope: ~100Hz = 10 readings
// - Magnetometer: ~20Hz = 2 readings
// - Euler angles: ~100Hz = 10 readings
// With margin of safety (2x)
const int ACCEL_BUFFER_SIZE = 20;
const int GYRO_BUFFER_SIZE = 20;
const int MAG_BUFFER_SIZE = 5;
const int EULER_BUFFER_SIZE = 20;

// Sensor reading structure
struct SensorReading {
  unsigned long timestamp;
  float x, y, z;
};

// Buffers for each sensor type
SensorReading magBuffer[MAG_BUFFER_SIZE];
SensorReading accelBuffer[ACCEL_BUFFER_SIZE];
SensorReading eulerBuffer[EULER_BUFFER_SIZE];
SensorReading gyroBuffer[GYRO_BUFFER_SIZE];

// Buffer indices and counts
int magCount = 0;
int accelCount = 0;
int eulerCount = 0;
int gyroCount = 0;

// Calibration variables
float gravityX = 0.0, gravityY = 0.0, gravityZ = 0.0;
float gyroBiasX = 0.0, gyroBiasY = 0.0, gyroBiasZ = 0.0;
float magOffsetX = 0.0, magOffsetY = 0.0, magOffsetZ = 0.0;

// Timing variables
unsigned long lastSendTime = 0;
const unsigned long SEND_INTERVAL = 100;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("ERROR: Failed to initialize IMU!");
    while (1);
  }

  printReadyMessage();
}

void printReadyMessage() {
  Serial.println("READY");
  Serial.println("Available commands:");
  Serial.println("  time_calibrate - Start time synchronization");
  Serial.println("  gravity_calibrate - Calibrate gravity compensation");
  Serial.println("  gyro_calibrate - Calibrate gyroscope bias");
  Serial.println("  mag_calibrate - Calibrate magnetometer (10s rotation)");
  Serial.println("  start_collection - Begin data collection");
  Serial.println("  stop_collection - Stop data collection");
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command == "time_calibrate") {
      currentState = TIME_CALIBRATING;
      handleTimeCalibration();
    } else if (command == "gravity_calibrate") {
      currentState = GRAVITY_CALIBRATING;
      calibrateGravity();
    } else if (command == "gyro_calibrate") {
      calibrateGyro();
    } else if (command == "mag_calibrate") {
      calibrateMag();
    } else if (command == "start_collection") {
      currentState = COLLECTING_DATA;
      resetBuffers();
      lastSendTime = millis();
    } else if (command == "stop_collection") {
      currentState = READY;
      printReadyMessage();
    }
  }

  if (currentState == COLLECTING_DATA) {
    collectAndSendData();
  }
}

void handleTimeCalibration() {
  // Respond immediately with timestamp
  Serial.print("TIME_ECHO,");
  Serial.println(micros());
  currentState = READY;
  printReadyMessage();
}

void calibrateGravity() {
  Serial.println("GRAVITY_CAL_START");

  const int numSamples = 100;
  float sumX = 0.0, sumY = 0.0, sumZ = 0.0;
  int sampleCount = 0;

  unsigned long startTime = millis();
  while (millis() - startTime < 1000 && sampleCount < numSamples) {
    if (IMU.accelerationAvailable()) {
      float x, y, z;
      IMU.readAcceleration(x, y, z);
      sumX += x;
      sumY += y;
      sumZ += z;
      sampleCount++;
      delay(10);
    }
  }

  if (sampleCount > 0) {
    gravityX = sumX / sampleCount;
    gravityY = sumY / sampleCount;
    gravityZ = sumZ / sampleCount;
    Serial.println("GRAVITY_CAL_COMPLETE");
  } else {
    gravityX = 0.0;
    gravityY = 0.0;
    gravityZ = 1.0;
    Serial.println("GRAVITY_CAL_FAILED");
  }

  currentState = READY;
  printReadyMessage();
}

void collectAndSendData() {
  float x, y, z;
  unsigned long timestamp = micros();

void calibrateGyro() {
  Serial.println("GYRO_CAL_START");
  const int numSamples = 100;
  float sumX = 0.0, sumY = 0.0, sumZ = 0.0;
  int sampleCount = 0;

  while (sampleCount < numSamples) {
    if (IMU.gyroscopeAvailable()) {
      float x, y, z;
      IMU.readGyroscope(x, y, z);
      sumX += x;
      sumY += y;
      sumZ += z;
      sampleCount++;
      delay(10);
    }
  }

  gyroBiasX = sumX / numSamples;
  gyroBiasY = sumY / numSamples;
  gyroBiasZ = sumZ / numSamples;
  
  Serial.println("GYRO_CAL_COMPLETE");
  currentState = READY;
  printReadyMessage();
}

void calibrateMag() {
  Serial.println("MAG_CAL_START");
  Serial.println("Rotate device slowly in all directions for 10 seconds");
  
  float minX = 1000, minY = 1000, minZ = 1000;
  float maxX = -1000, maxY = -1000, maxZ = -1000;
  unsigned long startTime = millis();
  
  while (millis() - startTime < 10000) {
    if (IMU.magneticFieldAvailable()) {
      float x, y, z;
      IMU.readMagneticField(x, y, z);
      
      minX = min(minX, x); maxX = max(maxX, x);
      minY = min(minY, y); maxY = max(maxY, y);
      minZ = min(minZ, z); maxZ = max(maxZ, z);
      
      delay(10);
    }
  }
  
  magOffsetX = (maxX + minX) / 2.0;
  magOffsetY = (maxY + minY) / 2.0;
  magOffsetZ = (maxZ + minZ) / 2.0;
  
  Serial.println("MAG_CAL_COMPLETE");
  currentState = READY;
  printReadyMessage();
}


  if (IMU.magneticFieldAvailable() && magCount < MAG_BUFFER_SIZE) {
    IMU.readMagneticField(x, y, z);
    magBuffer[magCount].timestamp = timestamp;
    magBuffer[magCount].x = x;
    magBuffer[magCount].y = y;
    magBuffer[magCount].z = z;
    magCount++;
  }

  if (IMU.accelerationAvailable() && accelCount < ACCEL_BUFFER_SIZE) {
    IMU.readAcceleration(x, y, z);
    accelBuffer[accelCount].timestamp = timestamp;
    accelBuffer[accelCount].x = x - gravityX;
    accelBuffer[accelCount].y = y - gravityY;
    accelBuffer[accelCount].z = z - gravityZ;
    accelCount++;
  }

  if (IMU.eulerAnglesAvailable() && eulerCount < EULER_BUFFER_SIZE) {
    IMU.readEulerAngles(x, y, z);
    eulerBuffer[eulerCount].timestamp = timestamp;
    eulerBuffer[eulerCount].x = x;
    eulerBuffer[eulerCount].y = y;
    eulerBuffer[eulerCount].z = z;
    eulerCount++;
  }

  if (IMU.gyroscopeAvailable() && gyroCount < GYRO_BUFFER_SIZE) {
    IMU.readGyroscope(x, y, z);
    gyroBuffer[gyroCount].timestamp = timestamp;
    gyroBuffer[gyroCount].x = x;
    gyroBuffer[gyroCount].y = y;
    gyroBuffer[gyroCount].z = z;
    gyroCount++;
  }

  unsigned long currentTime = millis();
  if (currentTime - lastSendTime >= SEND_INTERVAL ||
      magCount >= MAG_BUFFER_SIZE ||
      accelCount >= ACCEL_BUFFER_SIZE ||
      eulerCount >= EULER_BUFFER_SIZE ||
      gyroCount >= GYRO_BUFFER_SIZE) {
    sendAllData();
    resetBuffers();
    lastSendTime = currentTime;
  }
}

void sendAllData() {
  Serial.print("BEGIN_DATA_BLOCK,");
  Serial.print(magCount);
  Serial.print(",");
  Serial.print(accelCount);
  Serial.print(",");
  Serial.print(eulerCount);
  Serial.print(",");
  Serial.println(gyroCount);

  for (int i = 0; i < magCount; i++) {
    Serial.print("M,");
    Serial.print(magBuffer[i].timestamp);
    Serial.print(",");
    Serial.print(magBuffer[i].x, 6);
    Serial.print(",");
    Serial.print(magBuffer[i].y, 6);
    Serial.print(",");
    Serial.println(magBuffer[i].z, 6);
  }

  for (int i = 0; i < accelCount; i++) {
    Serial.print("A,");
    Serial.print(accelBuffer[i].timestamp);
    Serial.print(",");
    Serial.print(accelBuffer[i].x, 6);
    Serial.print(",");
    Serial.print(accelBuffer[i].y, 6);
    Serial.print(",");
    Serial.println(accelBuffer[i].z, 6);
  }

  for (int i = 0; i < eulerCount; i++) {
    Serial.print("E,");
    Serial.print(eulerBuffer[i].timestamp);
    Serial.print(",");
    Serial.print(eulerBuffer[i].x, 6);
    Serial.print(",");
    Serial.print(eulerBuffer[i].y, 6);
    Serial.print(",");
    Serial.println(eulerBuffer[i].z, 6);
  }

  for (int i = 0; i < gyroCount; i++) {
    Serial.print("G,");
    Serial.print(gyroBuffer[i].timestamp);
    Serial.print(",");
    Serial.print(gyroBuffer[i].x, 6);
    Serial.print(",");
    Serial.print(gyroBuffer[i].y, 6);
    Serial.print(",");
    Serial.println(gyroBuffer[i].z, 6);
  }

  Serial.println("END_DATA_BLOCK");
}

void resetBuffers() {
  magCount = 0;
  accelCount = 0;
  eulerCount = 0;
  gyroCount = 0;
}
#include <MKRIMU.h>

// === CHANGELOG ===
// - Fixed critical structure error: Moved calibrateGyro() outside of collectAndSendData().
// - Applied Gyroscope Bias: Gyroscope readings in collectAndSendData() now subtract gyroBiasX/Y/Z.
// - Applied Magnetometer Offset: Magnetometer readings in collectAndSendData() now subtract magOffsetX/Y/Z.
// - Added Timeout to Gyro Calibration: calibrateGyro() now includes a 2-second timeout to prevent hangs.
// - Implemented Timestamp Epoch System:
//   - Added timeEpochStartMicros global variable.
//   - handleTimeCalibration() now captures the micros() value at the time of the command, sends it back,
//     and stores it in timeEpochStartMicros. This serves as the zero reference (epoch) for the run.
//   - collectAndSendData() now calculates timestamps as elapsed microseconds *since* timeEpochStartMicros.
//     This prevents rollover issues *within* a single recording session relative to its start time,
//     assuming sessions are shorter than the micros() rollover period (~70 mins).
// - Added state check before starting collection to ensure time has been calibrated at least once.
// - Added comments explaining the gravity calibration fallback assumption.
// === END CHANGELOG ===


// States
enum State {
  READY,
  TIME_CALIBRATING, // Intermediate state during time calibration command handling
  GRAVITY_CALIBRATING, // Intermediate state during gravity calibration
  // Note: Gyro and Mag calibration functions block, so no dedicated state needed if kept that way.
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
const int MAG_BUFFER_SIZE = 5; // Smallest rate sensor dictates buffer strategy somewhat
const int EULER_BUFFER_SIZE = 20;

// Sensor reading structure
struct SensorReading {
  unsigned long timestamp; // Elapsed microseconds since timeEpochStartMicros
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
unsigned long lastSendTime = 0; // Milliseconds timestamp of last send
const unsigned long SEND_INTERVAL = 100; // Milliseconds interval for sending data

// Timestamp Epoch - Set during time calibration
unsigned long timeEpochStartMicros = 0;
bool timeCalibrated = false; // Flag to ensure time calibration happens first

void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for Serial connection

  if (!IMU.begin()) {
    Serial.println("ERROR: Failed to initialize IMU!");
    while (1); // Halt execution
  }

  // Initialize gravity assumption (optional, can be calculated)
  gravityZ = 1.0; // Assume Z is aligned with gravity initially if calibration fails

  printReadyMessage();
}

void printReadyMessage() {
  Serial.println("READY");
  Serial.println("Available commands:");
  Serial.println("  time_calibrate - Start time synchronization (REQUIRED before collection)");
  Serial.println("  gravity_calibrate - Calibrate gravity compensation (IMU stationary)");
  Serial.println("  gyro_calibrate - Calibrate gyroscope bias (IMU stationary)");
  Serial.println("  mag_calibrate - Calibrate magnetometer (10s rotation)");
  Serial.println("  start_collection - Begin data collection");
  Serial.println("  stop_collection - Stop data collection");
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command == "time_calibrate") {
      // No state change needed here as it's handled synchronously
      handleTimeCalibration();
    } else if (command == "gravity_calibrate") {
      // Can directly call blocking calibration function if not collecting
      if (currentState == READY) {
         calibrateGravity();
      } else {
        Serial.println("ERROR: Cannot calibrate while collecting data. Stop collection first.");
      }
    } else if (command == "gyro_calibrate") {
       if (currentState == READY) {
         calibrateGyro();
      } else {
        Serial.println("ERROR: Cannot calibrate while collecting data. Stop collection first.");
      }
    } else if (command == "mag_calibrate") {
       if (currentState == READY) {
         calibrateMag();
       } else {
        Serial.println("ERROR: Cannot calibrate while collecting data. Stop collection first.");
       }
    } else if (command == "start_collection") {
      if (!timeCalibrated) {
          Serial.println("ERROR: Time must be calibrated first using 'time_calibrate'.");
      } else if (currentState == READY) {
          currentState = COLLECTING_DATA;
          resetBuffers();
          lastSendTime = millis(); // Use millis() for send interval timing
          Serial.println("STARTING_COLLECTION");
      } else {
          Serial.println("WARNING: Already collecting data or calibrating.");
      }
    } else if (command == "stop_collection") {
      if (currentState == COLLECTING_DATA) {
        currentState = READY;
        // Optionally send any remaining data in buffers
        // sendAllData(); // Decide if needed based on requirements
        // resetBuffers();
        Serial.println("STOPPING_COLLECTION");
        printReadyMessage();
      } else {
         Serial.println("INFO: Not currently collecting data.");
      }
    } else {
        Serial.print("ERROR: Unknown command '");
        Serial.print(command);
        Serial.println("'");
        printReadyMessage(); // Show available commands again
    }
  }

  if (currentState == COLLECTING_DATA) {
    collectAndSendData();
  }
}

void handleTimeCalibration() {
  // Capture the timestamp *now* to establish the epoch reference
  timeEpochStartMicros = micros();
  timeCalibrated = true; // Mark time as calibrated

  // Respond immediately with the captured epoch timestamp
  Serial.print("TIME_ECHO,");
  Serial.println(timeEpochStartMicros);

  // No state change needed, return to ready implicitly
   // Do not print the full menu here, just confirm echo sent. Host knows command succeeded.
   // printReadyMessage(); // Can be added if explicit READY state confirmation is desired
}

void calibrateGravity() {
  Serial.println("GRAVITY_CAL_START - Keep IMU stationary for 1 second...");
  currentState = GRAVITY_CALIBRATING; // Indicate busy state

  const int numSamples = 100;
  float sumX = 0.0, sumY = 0.0, sumZ = 0.0;
  int sampleCount = 0;

  unsigned long startTime = millis();
  const unsigned long maxCalTime = 1500; // 1.5 seconds timeout

  while (millis() - startTime < maxCalTime && sampleCount < numSamples) {
    if (IMU.accelerationAvailable()) {
      float x, y, z;
      IMU.readAcceleration(x, y, z);
      sumX += x;
      sumY += y;
      sumZ += z;
      sampleCount++;
      delay(10); // ~100Hz sampling during calibration
    }
  }

  if (sampleCount > 0) { // Use whatever samples were collected, even if not numSamples
    gravityX = sumX / sampleCount;
    gravityY = sumY / sampleCount;
    gravityZ = sumZ / sampleCount;
    Serial.print("GRAVITY_CAL_COMPLETE - Samples: ");
    Serial.println(sampleCount);
    Serial.print("  Gravity Vector: ");
    Serial.print(gravityX); Serial.print(", ");
    Serial.print(gravityY); Serial.print(", ");
    Serial.println(gravityZ);
  } else {
    // Keep default/previous values or set a known default
    gravityX = 0.0;
    gravityY = 0.0;
    gravityZ = 1.0; // Fallback assumption: IMU is roughly flat (Z-axis up/down)
    Serial.println("GRAVITY_CAL_FAILED - No samples collected. Using default Z=1.0");
  }

  currentState = READY; // Return to ready state
  printReadyMessage(); // Inform user operation finished
}


// CORRECTED: Moved this function outside of collectAndSendData
void calibrateGyro() {
  Serial.println("GYRO_CAL_START - Keep IMU stationary...");
  // Note: This function blocks, so no dedicated state needed if acceptable UX
  const int numSamples = 100;
  float sumX = 0.0, sumY = 0.0, sumZ = 0.0;
  int sampleCount = 0;
  unsigned long startTime = millis(); // Add start time
  const unsigned long maxCalTime = 2000; // e.g., 2 seconds timeout

  // Add timeout condition to the loop
  while (sampleCount < numSamples && millis() - startTime < maxCalTime) {
    if (IMU.gyroscopeAvailable()) {
      float x, y, z;
      IMU.readGyroscope(x, y, z);
      sumX += x;
      sumY += y;
      sumZ += z;
      sampleCount++;
      delay(10); // ~100Hz sampling
    }
  }

  // Check if enough samples were collected (use what we have if timeout occurred)
  if (sampleCount > 0) {
      gyroBiasX = sumX / sampleCount;
      gyroBiasY = sumY / sampleCount;
      gyroBiasZ = sumZ / sampleCount;
      Serial.print("GYRO_CAL_COMPLETE - Samples: ");
      Serial.println(sampleCount);
      Serial.print("  Bias: ");
      Serial.print(gyroBiasX); Serial.print(", ");
      Serial.print(gyroBiasY); Serial.print(", ");
      Serial.println(gyroBiasZ);
  } else {
      // Handle failure - maybe keep old bias or set to zero
      gyroBiasX = 0.0; // Optional: Reset on failure
      gyroBiasY = 0.0;
      gyroBiasZ = 0.0;
      Serial.println("GYRO_CAL_FAILED - No samples collected. Bias set to zero.");
  }

  // currentState = READY; // Already in READY or returning to it
  printReadyMessage();
}

void calibrateMag() {
  Serial.println("MAG_CAL_START");
  Serial.println("Rotate device slowly in all directions (figure-eight) for 10 seconds...");
  // Note: This function blocks

  float minX = 1000, minY = 1000, minZ = 1000;
  float maxX = -1000, maxY = -1000, maxZ = -1000;
  unsigned long startTime = millis();
  const unsigned long calDuration = 10000; // 10 seconds

  while (millis() - startTime < calDuration) {
    if (IMU.magneticFieldAvailable()) {
      float x, y, z;
      IMU.readMagneticField(x, y, z);

      minX = min(minX, x); maxX = max(maxX, x);
      minY = min(minY, y); maxY = max(maxY, y);
      minZ = min(minZ, z); maxZ = max(maxZ, z);

      // Optional: Provide feedback during calibration?
      // if (millis() % 1000 < 20) { // Print status every second
      //    Serial.print(".");
      // }

      delay(10); // Don't need to sample excessively fast
    }
  }
  Serial.println(); // Newline after dots if feedback is added

  // Check if any data was captured (maxX/minX changed from initial values)
  if (maxX > -1000 && minX < 1000) {
      magOffsetX = (maxX + minX) / 2.0;
      magOffsetY = (maxY + minY) / 2.0;
      magOffsetZ = (maxZ + minZ) / 2.0;

      Serial.println("MAG_CAL_COMPLETE");
      Serial.print("  Offsets: ");
      Serial.print(magOffsetX); Serial.print(", ");
      Serial.print(magOffsetY); Serial.print(", ");
      Serial.println(magOffsetZ);
      // Optional: Calculate scale factors too for hard/soft iron, but requires more complex math
      // float scaleX = (maxX - minX) / 2.0; // Example radius calc
  } else {
      magOffsetX = 0.0; // Reset or keep previous on failure
      magOffsetY = 0.0;
      magOffsetZ = 0.0;
      Serial.println("MAG_CAL_FAILED - No magnetic field data captured or range too small. Offsets set to zero.");
  }


  // currentState = READY; // Already in READY or returning to it
  printReadyMessage();
}


void collectAndSendData() {
  // Calculate timestamp relative to the established epoch
  // This handles micros() rollover correctly for the *difference*, as long as
  // the time since epoch doesn't exceed ~70 minutes.
  unsigned long currentMicros = micros();
  unsigned long elapsedMicros = currentMicros - timeEpochStartMicros;

  float x, y, z; // Temporary variables for readings

  // Read sensors if available and buffer space exists
  // Apply Magnetometer Calibration
  if (IMU.magneticFieldAvailable() && magCount < MAG_BUFFER_SIZE) {
    IMU.readMagneticField(x, y, z);
    magBuffer[magCount].timestamp = elapsedMicros;
    magBuffer[magCount].x = x - magOffsetX; // Apply offset
    magBuffer[magCount].y = y - magOffsetY; // Apply offset
    magBuffer[magCount].z = z - magOffsetZ; // Apply offset
    magCount++;
  }

  // Apply Acceleration Calibration (Gravity Compensation)
  if (IMU.accelerationAvailable() && accelCount < ACCEL_BUFFER_SIZE) {
    IMU.readAcceleration(x, y, z);
    accelBuffer[accelCount].timestamp = elapsedMicros;
    accelBuffer[accelCount].x = x - gravityX; // Apply gravity compensation
    accelBuffer[accelCount].y = y - gravityY; // Apply gravity compensation
    accelBuffer[accelCount].z = z - gravityZ; // Apply gravity compensation
    accelCount++;
  }

  // Read Euler Angles (No explicit calibration applied here, relies on internal fusion)
  if (IMU.eulerAnglesAvailable() && eulerCount < EULER_BUFFER_SIZE) {
    IMU.readEulerAngles(x, y, z); // Assuming pitch, roll, yaw order - check library docs
    eulerBuffer[eulerCount].timestamp = elapsedMicros;
    eulerBuffer[eulerCount].x = x; // Pitch or Roll? Check MKRIMU.h documentation
    eulerBuffer[eulerCount].y = y; // Roll or Pitch?
    eulerBuffer[eulerCount].z = z; // Yaw
    eulerCount++;
  }

  // Apply Gyroscope Calibration
  if (IMU.gyroscopeAvailable() && gyroCount < GYRO_BUFFER_SIZE) {
    IMU.readGyroscope(x, y, z);
    gyroBuffer[gyroCount].timestamp = elapsedMicros;
    gyroBuffer[gyroCount].x = x - gyroBiasX; // Apply bias
    gyroBuffer[gyroCount].y = y - gyroBiasY; // Apply bias
    gyroBuffer[gyroCount].z = z - gyroBiasZ; // Apply bias
    gyroCount++;
  }

  // Check if it's time to send data or if any buffer is full
  unsigned long currentTime = millis();
  if (currentTime - lastSendTime >= SEND_INTERVAL ||
      magCount >= MAG_BUFFER_SIZE ||
      accelCount >= ACCEL_BUFFER_SIZE ||
      eulerCount >= EULER_BUFFER_SIZE ||
      gyroCount >= GYRO_BUFFER_SIZE) {
    sendAllData();
    resetBuffers();
    lastSendTime = currentTime; // Update last send time
  }
}

void sendAllData() {
  // Check if there's actually any data to send to avoid empty blocks
  if (magCount == 0 && accelCount == 0 && eulerCount == 0 && gyroCount == 0) {
    return;
  }

  Serial.print("BEGIN_DATA_BLOCK,");
  Serial.print(magCount);
  Serial.print(",");
  Serial.print(accelCount);
  Serial.print(",");
  Serial.print(eulerCount);
  Serial.print(",");
  Serial.println(gyroCount);

  // Send Magnetometer Data
  for (int i = 0; i < magCount; i++) {
    Serial.print("M,");
    Serial.print(magBuffer[i].timestamp);
    Serial.print(",");
    Serial.print(magBuffer[i].x, 6); // Send with 6 decimal places
    Serial.print(",");
    Serial.print(magBuffer[i].y, 6);
    Serial.print(",");
    Serial.println(magBuffer[i].z, 6);
  }

  // Send Accelerometer Data
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

  // Send Euler Angles Data
  for (int i = 0; i < eulerCount; i++) {
    Serial.print("E,");
    Serial.print(eulerBuffer[i].timestamp);
    Serial.print(",");
    Serial.print(eulerBuffer[i].x, 6); // Assuming Pitch/Roll
    Serial.print(",");
    Serial.print(eulerBuffer[i].y, 6); // Assuming Roll/Pitch
    Serial.print(",");
    Serial.println(eulerBuffer[i].z, 6); // Assuming Yaw
  }

  // Send Gyroscope Data
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
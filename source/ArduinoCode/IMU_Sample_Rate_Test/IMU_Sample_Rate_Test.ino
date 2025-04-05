
#include <MKRIMU.h>

// Test duration in milliseconds
const unsigned long TEST_DURATION = 5000;  // 5 seconds

// Counters for each sensor
unsigned long accelCount = 0;
unsigned long gyroCount = 0;
unsigned long magCount = 0;
unsigned long eulerCount = 0;

// Timing variables
unsigned long startTime;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  Serial.println("Starting sample rate test...");
  Serial.println("Testing for 5 seconds...");
  
  // Start timing
  startTime = millis();
}

void loop() {
  float x, y, z;
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - startTime;

  // Collect data for TEST_DURATION milliseconds
  if (elapsedTime < TEST_DURATION) {
    // Count accelerometer readings
    if (IMU.accelerationAvailable()) {
      IMU.readAcceleration(x, y, z);
      accelCount++;
    }

    // Count gyroscope readings
    if (IMU.gyroscopeAvailable()) {
      IMU.readGyroscope(x, y, z);
      gyroCount++;
    }

    // Count magnetometer readings
    if (IMU.magneticFieldAvailable()) {
      IMU.readMagneticField(x, y, z);
      magCount++;
    }

    // Count euler angle readings
    if (IMU.eulerAnglesAvailable()) {
      IMU.readEulerAngles(x, y, z);
      eulerCount++;
    }
  } 
  // After test duration, print results and stop
  else if (elapsedTime >= TEST_DURATION && accelCount > 0) {
    float testDurationSeconds = TEST_DURATION / 1000.0;
    
    Serial.println("\n=== Sample Rate Test Results ===");
    Serial.print("Test duration: ");
    Serial.print(testDurationSeconds);
    Serial.println(" seconds\n");
    
    Serial.print("Accelerometer: ");
    Serial.print(accelCount / testDurationSeconds, 2);
    Serial.println(" Hz");
    
    Serial.print("Gyroscope: ");
    Serial.print(gyroCount / testDurationSeconds, 2);
    Serial.println(" Hz");
    
    Serial.print("Magnetometer: ");
    Serial.print(magCount / testDurationSeconds, 2);
    Serial.println(" Hz");
    
    Serial.print("Euler Angles: ");
    Serial.print(eulerCount / testDurationSeconds, 2);
    Serial.println(" Hz");
    
    Serial.println("\nTotal samples collected:");
    Serial.print("Accelerometer: "); Serial.println(accelCount);
    Serial.print("Gyroscope: "); Serial.println(gyroCount);
    Serial.print("Magnetometer: "); Serial.println(magCount);
    Serial.print("Euler Angles: "); Serial.println(eulerCount);
    
    // Stop the program
    while(1);
  }
}

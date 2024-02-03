#include <Adafruit_ICM20948.h>
#include <Adafruit_ICM20X.h>

Adafruit_ICM20948 icm;

// Calibration parameters
const int numCalibrationSamples = 100;
float offsetX = 0, offsetY = 0, offsetZ = 0;

// Low-pass filter parameters
const float alpha = 0.002; // Adjust this value between 0 and 1; a higher value gives more smoothing
float filteredAX = 0, filteredAY = 0, filteredAZ = 0;

// Deadband thresholds for each axis
float deadbandThresholdXH, deadbandThresholdXL;
float deadbandThresholdYH, deadbandThresholdYL;
float deadbandThresholdZH, deadbandThresholdZL;

// Rolling window size
const int windowSize = 1;
float windowX[windowSize], windowY[windowSize], windowZ[windowSize];
int windowIndex = 0;

// Function prototypes
void updateWindow(float x, float y, float z);
void recalculateCalibration();

// Time tracking
unsigned long lastTime = 0;
unsigned long lastrecalibrationtime = 0;
const unsigned long recalibrationInterval = 10000*1000; //ms
float sampleRate = 0.01; // 10 milliseconds = 100 Hz

// Position tracking
float posX = 0, posY = 0, posZ = 0;
float velX = 0, velY = 0, velZ = 0;

void setup(void) {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Wire.setPins(21,22);
  if (!icm.begin_I2C()) {
    Serial.println("Failed to find ICM20948 chip");
    while (1) delay(10);
  }

  // Calibration
  calibrateAccelerometer();
  lastTime = millis();
}

void loop() {
  if (millis() - lastTime >= sampleRate * 1000) {
    lastTime = millis();

    sensors_event_t accel;
    icm.getAccelerometerSensor()->getEvent(&accel);
    
    // Apply deadband filters
    float ax = applyDeadbandFilterX(accel.acceleration.x - offsetX);
    float ay = applyDeadbandFilterY(accel.acceleration.y - offsetY);
    float az = applyDeadbandFilterZ(accel.acceleration.z - offsetZ);
    
    // Update the rolling window with the new values
    updateWindow(accel.acceleration.x, accel.acceleration.y, accel.acceleration.z);

    // Periodically recalculate calibration
    if (millis() - lastrecalibrationtime > recalibrationInterval) {
        lastrecalibrationtime = millis();
        recalculateCalibration();
    }

    // Apply low-pass filter
    filteredAX = alpha * (ax) + (1 - alpha) * filteredAX;
    filteredAY = alpha * (ay) + (1 - alpha) * filteredAY;
    filteredAZ = alpha * (az) + (1 - alpha) * filteredAZ;

    // Integrate to get velocity and position
    integrateAcceleration(filteredAX, filteredAY, filteredAZ, sampleRate);

    // Output the position
    // Output the position
    Serial.print(accel.acceleration.x);
    Serial.print(",");
    Serial.print(accel.acceleration.y);
    Serial.print(",");
    Serial.print(accel.acceleration.z);
    Serial.print(",");
    Serial.print(posX);
    Serial.print(",");
    Serial.print(posY);
    Serial.print(",");
    Serial.println(posZ);
  }
}

void calibrateAccelerometer() {
    float meanX = 0, meanY = 0, meanZ = 0;
    float sumSqX = 0, sumSqY = 0, sumSqZ = 0;
    float stdDevX, stdDevY, stdDevZ;

    // Collect calibration data
    for (int i = 0; i < numCalibrationSamples; i++) {
        sensors_event_t accel;
        icm.getAccelerometerSensor()->getEvent(&accel);
        meanX += accel.acceleration.x;
        meanY += accel.acceleration.y;
        meanZ += accel.acceleration.z;
        sumSqX += accel.acceleration.x * accel.acceleration.x;
        sumSqY += accel.acceleration.y * accel.acceleration.y;
        sumSqZ += accel.acceleration.z * accel.acceleration.z;
        delay(10);
    }
    meanX /= numCalibrationSamples;
    meanY /= numCalibrationSamples;
    meanZ /= numCalibrationSamples;
    sumSqX /= numCalibrationSamples;
    sumSqY /= numCalibrationSamples;
    sumSqZ /= numCalibrationSamples;

    // Calculate standard deviation for each axis
    stdDevX = sqrt(sumSqX - (meanX * meanX));
    stdDevY = sqrt(sumSqY - (meanY * meanY));
    stdDevZ = sqrt(sumSqZ - (meanZ * meanZ));

    // Set deadband thresholds based on standard deviation
    deadbandThresholdXH = stdDevX * 2; // Example: 2 standard deviations
    deadbandThresholdXL = -deadbandThresholdXH;
    deadbandThresholdYH = stdDevY * 2;
    deadbandThresholdYL = -deadbandThresholdYH;
    deadbandThresholdZH = stdDevZ * 2;
    deadbandThresholdZL = -deadbandThresholdZH;

    offsetX = meanX;
    offsetY = meanY;
    offsetZ = meanZ;
}

void updateWindow(float x, float y, float z) {
    windowX[windowIndex] = x;
    windowY[windowIndex] = y;
    windowZ[windowIndex] = z;
    windowIndex = (windowIndex + 1) % windowSize;
}

void recalculateCalibration() {
    float sumX = 0, sumY = 0, sumZ = 0;
    float sumSqX = 0, sumSqY = 0, sumSqZ = 0;
    for (int i = 0; i < windowSize; i++) {
        sumX += windowX[i];
        sumY += windowY[i];
        sumZ += windowZ[i];
        sumSqX += windowX[i] * windowX[i];
        sumSqY += windowY[i] * windowY[i];
        sumSqZ += windowZ[i] * windowZ[i];
    }

    float meanX = sumX / windowSize;
    float meanY = sumY / windowSize;
    float meanZ = sumZ / windowSize;

    float stdDevX = sqrt(sumSqX / windowSize - meanX * meanX);
    float stdDevY = sqrt(sumSqY / windowSize - meanY * meanY);
    float stdDevZ = sqrt(sumSqZ / windowSize - meanZ * meanZ);

    offsetX = meanX;
    offsetY = meanY;
    offsetZ = meanZ;

    deadbandThresholdXH = stdDevX * 2; // Example: 2 standard deviations
    deadbandThresholdXL = -deadbandThresholdXH;
    deadbandThresholdYH = stdDevY * 2;
    deadbandThresholdYL = -deadbandThresholdYH;
    deadbandThresholdZH = stdDevZ * 2;
    deadbandThresholdZL = -deadbandThresholdZH;

    // Repeat for Y and Z
}

void integrateAcceleration(float ax, float ay, float az, float dt) {
  // Integrate acceleration to get velocity
  velX += ax * dt;
  velY += ay * dt;
  velZ += az * dt;

  // Integrate velocity to get position
  posX += velX * dt;
  posY += velY * dt;
  posZ += velZ * dt;
}

float applyDeadbandFilterX(float value) {
    if (value > deadbandThresholdXL && value < deadbandThresholdXH) {
        return 0;
    }
    return value;
}

float applyDeadbandFilterY(float value) {
    if (value > deadbandThresholdYL && value < deadbandThresholdYH) {
        return 0;
    }
    return value;
}

float applyDeadbandFilterZ(float value) {
    if (value > deadbandThresholdZL && value < deadbandThresholdZH) {
        return 0;
    }
    return value;
}

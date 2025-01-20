#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Kalman.h>

// Create MPU6050 and Kalman filter objects
Adafruit_MPU6050 mpu;
Kalman kalmanX, kalmanY;

// Variables for Kalman filter
float angleX, angleY;        // Filtered angles
float gyroXrate, gyroYrate;  // Gyroscope rates
float dt;                    // Time delta
unsigned long lastTime;

// Setup CSV logging
#define LOG_INTERVAL 100 // Log every 100 ms
unsigned long lastLogTime = 0;

void setup() {
    Serial.begin(115200);
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        while (1) { delay(10); }
    }

    Serial.println("MPU6050 initialized!");

    // Set MPU6050 ranges
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    // Initialize Kalman filters
    kalmanX.setAngle(0); // Starting angle
    kalmanY.setAngle(0);

    lastTime = millis();

    // Print CSV header
    Serial.println("Time,Pitch,Roll");
}

void loop() {
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);

    // Calculate time delta
    unsigned long currentTime = millis();
    dt = (currentTime - lastTime) / 1000.0;
    lastTime = currentTime;

    // Calculate accelerometer angles (pitch and roll)
    float accelAngleX = atan2(accel.acceleration.y, accel.acceleration.z) * 180 / PI;
    float accelAngleY = atan2(-accel.acceleration.x, accel.acceleration.z) * 180 / PI;

    // Get gyroscope rates
    gyroXrate = gyro.gyro.x * 180 / PI;
    gyroYrate = gyro.gyro.y * 180 / PI;

    // Apply Kalman filter
    angleX = kalmanX.getAngle(accelAngleX, gyroXrate, dt);
    angleY = kalmanY.getAngle(accelAngleY, gyroYrate, dt);

    // Log data to Serial as CSV
    if (currentTime - lastLogTime >= LOG_INTERVAL) {
        lastLogTime = currentTime;
        Serial.print(currentTime / 1000.0, 2); // Time in seconds
        Serial.print(",");
        Serial.print(angleX, 2); // Pitch
        Serial.print(",");
        Serial.println(angleY, 2); // Roll
    }

    delay(10); // Adjust for smoother data
}

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <ArduinoJson.h>

// WiFi settings
const char* ssid = "IAAC-WIFI";        // Replace with your WiFi SSID
const char* password = "password"; // Replace with your WiFi Password

WiFiServer server(80);  // Start a web server on port 80

Adafruit_MPU6050 mpu;

// RGB LED Pins
const int redPin = 19;
const int greenPin = 18;
const int bluePin = 17;

// Timestamp variable
unsigned long previousMillis = 0;
const long interval = 1000;  // Print every second

void setup() {
  // Start Serial for debugging
  Serial.begin(115200);

  // Initialize RGB LED pins as output
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  // Connect to WiFi
  Serial.print("Connecting to WiFi...");
  WiFi.begin(ssid, password);

  // Try connecting to Wi-Fi until success or timeout after 30 attempts
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    attempts++;
    
    // Timeout after 30 attempts (15 seconds)
    if (attempts >= 30) {
      Serial.println("\nFailed to connect to WiFi. Please check your credentials.");
      digitalWrite(redPin, HIGH);  // Red LED to indicate failure
      digitalWrite(greenPin, LOW);
      digitalWrite(bluePin, LOW);
      while (1) {
        delay(1000); // Stop execution on failure
      }
    }
  }

  // Once connected, print IP address and indicate success with green LED
  Serial.println("\nConnected to WiFi!");

  // Increase delay to make sure IP is assigned
  delay(500);

  // Print IP address with timestamp
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Indicate connection success with Green LED
  digitalWrite(redPin, LOW);
  digitalWrite(greenPin, HIGH);  // Green LED to indicate success
  digitalWrite(bluePin, LOW);
  
  // Start the server
  server.begin();
  Serial.println("Server started!");
  
  // Initialize the MPU6050 sensor
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) { delay(10); } // Stop if sensor is not found
  }
  Serial.println("MPU6050 initialized!");
  
  // Set the sensor ranges
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Indicate successful startup with Green LED
  digitalWrite(greenPin, HIGH);
  delay(2000);
  digitalWrite(greenPin, LOW);
}

void loop() {
  // Get the current time
  unsigned long currentMillis = millis();
  
  // Print IP address with timestamp every second (or as desired)
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    
    // Print timestamp and IP address to Serial Monitor
    Serial.print("Timestamp: ");
    Serial.print(currentMillis / 1000); // seconds
    Serial.print("s | IP Address: ");
    Serial.println(WiFi.localIP());  // Print the IP address
  }

  WiFiClient client = server.available();

  if (client) {
    // Wait for client to send data
    while (client.connected()) {
      if (client.available()) {
        // Read the accelerometer, gyro, and temperature data
        sensors_event_t accel, gyro, temp;
        mpu.getEvent(&accel, &gyro, &temp);

        // Create a JSON object to store the data
        StaticJsonDocument<512> doc;
        doc["accel_x"] = accel.acceleration.x;
        doc["accel_y"] = accel.acceleration.y;
        doc["accel_z"] = accel.acceleration.z;
        doc["gyro_x"] = gyro.gyro.x;
        doc["gyro_y"] = gyro.gyro.y;
        doc["gyro_z"] = gyro.gyro.z;
        doc["temp"] = temp.temperature;

        // Serialize the JSON object to a string and send it to the client
        String output;
        serializeJson(doc, output);
        client.println(output); // Send the JSON data as a response

        // Check sensor readings and update LED color accordingly
        if (abs(accel.acceleration.x) > 2.0 || abs(accel.acceleration.y) > 2.0 || abs(accel.acceleration.z) > 2.0) {
          // If significant movement detected -> Red LED (error)
          digitalWrite(redPin, HIGH);
          digitalWrite(greenPin, LOW);
          digitalWrite(bluePin, LOW);
        } else {
          // If stable sensor values -> Green LED (normal)
          digitalWrite(redPin, LOW);
          digitalWrite(greenPin, HIGH);
          digitalWrite(bluePin, LOW);
        }

        // Delay to avoid overwhelming the client
        delay(1000);
      }
    }

    // Close the client connection
    client.stop();
    // Turn off all LEDs when client disconnects (Blue LED - IDLE)
    digitalWrite(redPin, LOW);
    digitalWrite(greenPin, LOW);
    digitalWrite(bluePin, HIGH);
  }
}

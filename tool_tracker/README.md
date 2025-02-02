# WiFi-enabled MPU6050 Sensor with RGB LED Indicator

This project uses the **Adafruit MPU6050** sensor along with an **ESP32/ESP8266** (using `WiFi.h`) to create a simple web server that displays accelerometer, gyroscope, and temperature data in JSON format. The project also includes an RGB LED that visually indicates the status of the device: red for failure, green for normal operation, and blue when idle or when there is no client connection.

## Features

- Connects to Wi-Fi network.
- Starts a web server on port 80.
- Serves sensor data (accelerometer, gyroscope, and temperature) in JSON format.
- Indicates device status via RGB LED:
  - **Red**: Wi-Fi connection failed or significant motion detected.
  - **Green**: Successful Wi-Fi connection and stable sensor values.
  - **Blue**: No client connection or idle mode.
- Provides a timestamp with the device's IP address in the Serial Monitor every second.

## Requirements

- **Hardware**:
  - ESP32 or ESP8266
  - MPU6050 Accelerometer & Gyroscope module
  - RGB LED (3 pins: Red, Green, Blue)

- **Libraries**:
  - `Wire.h` (I2C communication)
  - `Adafruit_MPU6050.h` (MPU6050 sensor library)
  - `Adafruit_Sensor.h` (Adafruit sensor framework)
  - `WiFi.h` (ESP32/ESP8266 Wi-Fi support)
  - `ArduinoJson.h` (JSON parsing and generation)

## Setup Instructions

### 1. Wiring the Components

- **MPU6050 Sensor**:
  - Connect the **VCC** pin to **3.3V** (or **5V** if supported).
  - Connect the **GND** pin to **GND**.
  - Connect the **SCL** pin to the **SCL** pin on the ESP board.
  - Connect the **SDA** pin to the **SDA** pin on the ESP board.

- **RGB LED**:
  - Connect the **Red pin** to **GPIO 19**.
  - Connect the **Green pin** to **GPIO 18**.
  - Connect the **Blue pin** to **GPIO 17**.

### 2. Software Setup

- Install the following libraries via the Arduino Library Manager:
  - **Adafruit MPU6050** by Adafruit
  - **Adafruit Unified Sensor** by Adafruit
  - **ArduinoJson** by Benoit Blanchon

### 3. Modify Wi-Fi Credentials

In the code, replace the following lines with your Wi-Fi network credentials:

```cpp
const char* ssid = "your-wifi-ssid";
const char* password = "your-wifi-password";
```

### 4. Upload the Code

    - Open the Arduino IDE.
    - Select your ESP32/ESP8266 board in the Tools > Board menu.
    - Choose the correct COM port under Tools > Port.
    - Click the Upload button (right arrow icon) to upload the code to your board.

Once the upload is complete, the ESP32/ESP8266 will restart, and the serial monitor will begin displaying the connection process.

### 5. Monitor the Serial Output

Open the Serial Monitor in the Arduino IDE:

    - Go to Tools > Serial Monitor or press Ctrl + Shift + M.
    - Set the baud rate to 115200.

You will see the following information:

    - Wi-Fi connection status.
    - The device's IP address once connected to Wi-Fi.
    - A timestamp with the IP address every second.

6. Access the Web Server

Once the device is connected to Wi-Fi, open a web browser and enter the IP address displayed in the Serial Monitor. For example:

```
http://192.168.x.x
```

The server will respond with JSON data from the MPU6050 sensor, looking like this:

```json
{
  "accel_x": 0.12,
  "accel_y": -0.09,
  "accel_z": 9.81,
  "gyro_x": 0.02,
  "gyro_y": 0.03,
  "gyro_z": -0.01,
  "temp": 25.2
}
```

## Troubleshooting

    - Failed to Connect to Wi-Fi: If the device cannot connect to Wi-Fi, the red LED will light up. Check your SSID and password for accuracy.
    - No Data on Web Server: Ensure the device is connected to your Wi-Fi and that your browser is pointing to the correct IP address.
    - Sensor Not Detected: If the sensor is not found during initialization, ensure that the connections (SDA and SCL) are correct, and check that the sensor is powered correctly.
    - No Response from Web Server: Check that the device is running, connected to the Wi-Fi, and that the IP address is correctly entered into the browser.

## Code Explanation

    - Wi-Fi Setup: The device attempts to connect to the specified Wi-Fi network. If it fails, the red LED will light up, and the device will halt.

    - MPU6050 Sensor Initialization: The sensor is initialized with specific accelerometer and gyroscope ranges and filter bandwidth. If the sensor is not detected, an error message is displayed in the Serial Monitor.

    - Web Server: A simple web server listens for incoming client requests. Upon connection, the server reads the sensor data, creates a JSON object, and sends it to the client.

    - RGB LED: The RGB LED provides visual feedback on the device’s status:
        Red: Wi-Fi connection failed or significant sensor motion detected.
        Green: Successful Wi-Fi connection and stable sensor readings.
        Blue: No client connected or idle mode.

## License

This project is licensed under the MIT License - see the LICENSE file for details

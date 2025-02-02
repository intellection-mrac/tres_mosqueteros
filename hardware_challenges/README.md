# RGB LED Control with Joystick and Buttons

This project demonstrates how to control an RGB LED using a joystick module and buttons. It supports five unique scenarios that allow users to interact with the LED in different ways, including toggling states, adjusting color, and controlling brightness. The joystick's X and Y axes manage the color and brightness, while the joystick button and additional scenario buttons determine the LED's behavior.

---

## Features

### Scenario 1: Joystick Button ON/OFF
- The RGB LED turns **ON** when the joystick button is pressed.
- The LED turns **OFF** immediately when the button is released.

### Scenario 2: LED with 5-Second Delay
- The RGB LED turns **ON** when the joystick button is pressed.
- The LED remains **ON** for **5 seconds** after the button is released.

### Scenario 3: Toggle LED
- The RGB LED toggles **ON/OFF** with each press of the joystick button.

### Scenario 4: Joystick Button ON, Secondary Button OFF
- The RGB LED turns **ON** when the joystick button is pressed.
- The LED stays **ON** until the secondary button is pressed.

### Scenario 5: Dynamic Color and Brightness Control
- The joystick’s **X-axis** adjusts the hue (color spectrum).
- The **Y-axis** adjusts the LED brightness, with changes only applied if the joystick movement exceeds a threshold.

---

## Required Components

| **Component**            | **Quantity** | **Description**                                  |
|--------------------------|--------------|--------------------------------------------------|
| Arduino Uno or Equivalent | 1            | Microcontroller to run the code                  |
| RGB LED                   | 1            | Common cathode RGB LED                          |
| Joystick Module           | 1            | Joystick with X, Y axes and a button            |
| Push Button               | 2            | Additional buttons for scenario and state control|
| Resistors (330Ω)          | 3            | Pull-down resistors for the buttons             |
| Jumper Wires              | As needed    | For connections                                 |
| Breadboard                | 1            | For prototyping                                 |

---

## Wiring

### RGB LED Connections
- **Red Pin (R)** -> Arduino Pin **3**
- **Green Pin (G)** -> Arduino Pin **5**
- **Blue Pin (B)** -> Arduino Pin **6**
- **Cathode (GND)** -> Ground (GND)

### Joystick Connections
- **VCC** -> 5V
- **GND** -> GND
- **X-axis (VRx)** -> Analog Pin **A0**
- **Y-axis (VRy)** -> Analog Pin **A1**
- **Button (SW)** -> Digital Pin **2**

### Button Connections
- **Scenario Button** -> Digital Pin **4**
- **Secondary Button** -> Digital Pin **7**

---

## Installation and Usage

### 1. Upload the Code
   - Open the Arduino IDE.
   - Upload the provided sketch to your Arduino board.

### 2. Set Up the Circuit
   - Connect all components as shown in the circuit diagram.

### 3. Changing the Scenarios
   - Press the **scenario button** (pin 4) to cycle through the five scenarios.
   - Watch the RGB LED’s behavior change based on the joystick and button inputs.

### 4. Monitoring via Serial
   - Open the Serial Monitor (baud rate: 9600) to track the current scenario, LED state, and joystick values.

---

## How It Works

### Scenario Selection
- Press the **scenario button** to cycle through the scenarios (1 to 5). The active scenario will be displayed in the Serial Monitor.

### Joystick Button
- The **joystick button** (pin 2) triggers actions in most scenarios (e.g., turning the LED ON/OFF).

### Secondary Button
- The **secondary button** (pin 7) is used in **Scenario 4** to turn off the LED.

### Color Control (Scenario 5)
- The **X-axis** of the joystick adjusts the hue (color spectrum).
- The **Y-axis** of the joystick controls the LED brightness, adjusting only if the movement exceeds the defined threshold.

---

## Debugging and Monitoring

- Open the **Serial Monitor** to observe real-time updates:
  - **Scenario Change**: `"Scenario changed to: X"`
  - **LED State**: `"Scenario: X | LED State: ON"`
  - **Joystick Input and RGB Values (Scenario 5)**: `"Scenario 5 | Hue: 120 | Brightness: 200 | RGB: (0, 255, 100)"`

---

## Customization Options

- **Joystick Neutral Values**: Adjust `neutralX` and `neutralY` if your joystick module has different neutral values.
- **Debounce Delay**: Fine-tune button responsiveness by adjusting the `debounceDelay`.
- **Color Spectrum**: Customize the hue range in **Scenario 5** to fit your needs.

---

## Contributors

- **Neeeeeeeil**
- **Nacho**
- **Santosh**

If you'd like to contribute, feel free to submit a pull request!

---

## References

- **Arduino Documentation**:  
  [https://www.arduino.cc/en/Reference/HomePage](https://www.arduino.cc/en/Reference/HomePage)  
  Official documentation for Arduino functions, libraries, and best practices.

- **Joystick Module Pinout and Usage**:  
  [https://www.electronicwings.com/arduino/joystick-module](https://www.electronicwings.com/arduino/joystick-module)  
  Guide on how to connect and use a joystick module with Arduino.

## License

This project is licensed under the **MIT License**. Feel free to use and modify the code as needed.

---

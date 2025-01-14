
# **RGB LED Control with Joystick and Buttons**

This project controls an RGB LED using a joystick module and multiple buttons. Depending on the selected scenario, the joystick and buttons provide various ways to manipulate the LED's behavior, including toggling states, timed activation, and dynamic color spectrum control.

---

## **Features**

1. **Scenario 1**:
   - The RGB LED turns **ON** while the joystick button is held down.
   - The LED turns **OFF** immediately upon release.

2. **Scenario 2**:
   - The RGB LED turns **ON** when the joystick button is pressed.
   - The LED stays **ON** for **5 seconds** after the button is released.

3. **Scenario 3**:
   - The RGB LED toggles **ON** or **OFF** with each press of the joystick button.
   - A single press toggles the state.

4. **Scenario 4**:
   - The RGB LED turns **ON** when the joystick button is pressed.
   - The LED stays **ON** until a secondary button is pressed.

5. **Scenario 5**:
   - The joystick's **X-axis** controls the color spectrum (Hue), allowing you to scroll through colors.
   - The **Y-axis** adjusts the brightness, requiring a movement threshold of ±10 from the neutral position for changes.

---

## **Required Components**

| **Component**            | **Quantity** | **Description**                                  |
|---------------------------|--------------|-------------------------------------------------|
| Arduino Uno or Equivalent | 1            | Microcontroller for running the code           |
| RGB LED                   | 1            | Common cathode RGB LED                         |
| Joystick Module           | 1            | Module with X, Y axes, and a button            |
| Push Button               | 2            | Additional buttons for mode and state control  |
| Resistors (10kΩ)          | 2            | Pull-down resistors for the buttons            |
| Jumper Wires              | As needed    | For connections                                |
| Breadboard                | 1            | For prototyping the circuit                    |

---

## **Circuit Diagram**

1. **RGB LED**:
   - **R (Red)** -> Arduino pin **3**.
   - **G (Green)** -> Arduino pin **5**.
   - **B (Blue)** -> Arduino pin **6**.
   - **Cathode** -> Ground (GND).

2. **Joystick**:
   - **VCC** -> 5V.
   - **GND** -> GND.
   - **VRx (X-axis)** -> Analog pin **A0**.
   - **VRy (Y-axis)** -> Analog pin **A1**.
   - **SW (Button)** -> Digital pin **2**.

3. **Buttons**:
   - **Scenario Button**:
     - One pin -> Digital pin **4**.
     - Other pin -> GND with a pull-up or pull-down resistor.
   - **Secondary Button** (for Scenario 4):
     - One pin -> Digital pin **7**.
     - Other pin -> GND with a pull-up or pull-down resistor.

---

## **Installation and Usage**

1. **Upload the Code**:
   - Open the Arduino IDE and upload the provided sketch to your Arduino board.

2. **Set Up the Circuit**:
   - Connect all components as per the circuit diagram.

3. **Control the Scenarios**:
   - Use the **scenario button** (pin 4) to cycle through the five scenarios.
   - Observe the behavior of the RGB LED based on the joystick and button interactions.

4. **Monitor via Serial**:
   - Open the Serial Monitor (baud rate: 9600) to debug and track the system's status, including scenario changes, joystick input, and RGB values.

---

## **How It Works**

1. **Scenario Selection**:
   - Press the **scenario button** (pin 4) to incrementally cycle through the five scenarios.
   - The current scenario number is printed to the Serial Monitor.

2. **Joystick Button**:
   - The **joystick button** (pin 2) is the primary trigger for controlling the LED state in most scenarios.

3. **Secondary Button**:
   - The **secondary button** (pin 7) is used exclusively in Scenario 4 to turn off the LED.

4. **Dynamic Color Control**:
   - In **Scenario 5**, the joystick's X and Y axes control the color and brightness:
     - **X-axis**: Changes the color spectrum (Hue).
     - **Y-axis**: Adjusts brightness, with changes only applied if the movement exceeds a threshold of ±10 from the neutral position.

---

## **Debugging**

- Use the **Serial Monitor** for real-time updates:
  - Scenario changes: `"Scenario changed to: X"`
  - LED state: `"Scenario: X | LED State: ON"`
  - Joystick input and RGB values (Scenario 5): `"Scenario 5 | Hue: 120 | Brightness: 200 | RGB: (0, 255, 100)"`

---

## **Further Customizations**

- Adjust the joystick neutral values (`neutralX` and `neutralY`) in the code if they differ based on your module.
- Modify the debounce delay (`debounceDelay`) to fine-tune button responsiveness.
- Experiment with the mapping ranges for Scenario 5 to customize the color spectrum.

---

## **License**

This project is licensed under the MIT License. Feel free to use and modify the code as needed.

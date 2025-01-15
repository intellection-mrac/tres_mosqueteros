// RGB LED Pins (connected to R3, G5, B6)
const int redPin = 3;
const int greenPin = 5;
const int bluePin = 6;

// Joystick Pins
const int joyX = A0;   // X-axis connected to analog pin A0
const int joyY = A1;   // Y-axis connected to analog pin A1
const int joyButton = 2; // Joystick button connected to digital pin 2

// Additional Buttons
const int scenarioButton = 4; // Button to select scenario
const int secondaryButton = 7; // Secondary button for scenario 4

// Joystick Neutral Values
const int neutralX = 502; // Neutral X position
const int neutralY = 520; // Neutral Y position

// State Variables
int scenario = 1; // Active scenario
bool ledState = false; // LED ON/OFF state
bool buttonPressed = false; // Debouncing for scenario button
bool jbLastState = false; // Previous joystick button state for scenario 3
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

unsigned long startTime = 0; // Timer for Scenario 2

// Threshold for joystick movement
const int threshold = 10;

// RGB Variables for Scenario 5
int hue = 0; // Hue value (0-360)
int brightness = 255; // LED brightness (0-255)

// Function to convert HSV to RGB
void HSVtoRGB(int h, int s, int v, int &r, int &g, int &b) {
  float hf = h / 60.0;
  int i = int(hf) % 6;
  float f = hf - int(hf);
  float p = v * (1 - s / 255.0);
  float q = v * (1 - f * s / 255.0);
  float t = v * (1 - (1 - f) * s / 255.0);

  switch (i) {
    case 0: r = v; g = t; b = p; break;
    case 1: r = q; g = v; b = p; break;
    case 2: r = p; g = v; b = t; break;
    case 3: r = p; g = q; b = v; break;
    case 4: r = t; g = p; b = v; break;
    case 5: r = v; g = p; b = q; break;
  }
}

void setup() {
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  pinMode(joyButton, INPUT_PULLUP); // Joystick button
  pinMode(scenarioButton, INPUT_PULLUP); // Scenario button
  pinMode(secondaryButton, INPUT_PULLUP); // Secondary button for scenario 4

  Serial.begin(9600); // Initialize serial communication
  Serial.println("Setup complete. Ready to start!");
}

void loop() {
  // Handle scenario selection
  if (digitalRead(scenarioButton) == LOW) {
    unsigned long currentTime = millis();
    if (!buttonPressed && (currentTime - lastDebounceTime) > debounceDelay) {
      scenario = (scenario % 5) + 1; // Cycle through 1 to 5
      buttonPressed = true;
      lastDebounceTime = currentTime;
      Serial.print("Scenario changed to: "); Serial.println(scenario);
    }
  } else {
    buttonPressed = false;
  }

  // Joystick button state
  bool jbPressed = digitalRead(joyButton) == LOW;

  switch (scenario) {
    case 1:
      // Scenario 1: LED on while joystick button is pressed
      ledState = jbPressed;
      break;

    case 2:
      // Scenario 2: LED stays on for 5 seconds after release
      if (jbPressed) {
        ledState = true;
        startTime = millis();
      }
      if (millis() - startTime > 5000) {
        ledState = false;
      }
      break;

    case 3:
      // Scenario 3: Toggle LED on/off with each joystick button press
      if (jbPressed && !jbLastState) {
        ledState = !ledState; // Toggle LED state
      }
      jbLastState = jbPressed; // Update last state
      break;

    case 4:
      // Scenario 4: LED on with joystick button, off with secondary button
      if (jbPressed) {
        ledState = true;
      }
      if (digitalRead(secondaryButton) == LOW) {
        ledState = false;
      }
      break;

    case 5:
      // Scenario 5: Color spectrum on X-axis, brightness on Y-axis
      int xValue = analogRead(joyX);
      int yValue = analogRead(joyY);

      int xDelta = xValue - neutralX;
      int yDelta = yValue - neutralY;

      if (abs(xDelta) > threshold) {
        hue = (hue + (xDelta > 0 ? 1 : -1)) % 360;
        if (hue < 0) hue += 360; // Ensure hue stays positive
      }

      if (abs(yDelta) > threshold) {
        brightness = constrain(brightness + (yDelta > 0 ? -5 : 5), 0, 255);
      }

      int r, g, b;
      HSVtoRGB(hue, 255, brightness, r, g, b);
      analogWrite(redPin, r);
      analogWrite(greenPin, g);
      analogWrite(bluePin, b);

      // Debugging output for Scenario 5
      Serial.print("Scenario 5 | Hue: "); Serial.print(hue);
      Serial.print(" | Brightness: "); Serial.print(brightness);
      Serial.print(" | RGB: ("); Serial.print(r); Serial.print(", ");
      Serial.print(g); Serial.print(", "); Serial.print(b); Serial.println(")");
      return; // Skip further processing for scenario 5
  }

  // Apply LED state (ON/OFF)
  if (ledState) {
    analogWrite(redPin, 255);
    analogWrite(greenPin, 255);
    analogWrite(bluePin, 255);
  } else {
    analogWrite(redPin, 0);
    analogWrite(greenPin, 0);
    analogWrite(bluePin, 0);
  }

  // Debugging output for all other scenarios
  Serial.print("Scenario: "); Serial.print(scenario);
  Serial.print(" | LED State: "); Serial.println(ledState ? "ON" : "OFF");
  delay(50); // Stability delay
}

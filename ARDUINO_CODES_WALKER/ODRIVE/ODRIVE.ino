#include <ODriveUART.h>
#include <HardwareSerial.h>


// ODrive Configuration

HardwareSerial odrive_serial(1); // Use UART1
unsigned long baudrate = 115200;
const int ODRIVE_RX_PIN = 18; // Connect to ODrive's TX (GPIO1)
const int ODRIVE_TX_PIN = 19; // Connect to ODrive's RX (GPIO2)
ODriveUART odrive(odrive_serial);


// Joystick Configuration

const int deadmanSwitchPin = 13;
const int motorForwardPin  = 25;
const int moveLeftPin      = 12; // Currently not used for movement
const int moveRightPin     = 14; // Currently not used for movement

// Control Parameters
const float FORWARD_VELOCITY = 0.2f; // Velocity in turns/sec when moving forward
const float BRAKING_TORQUE = 0.5f;   // Braking torque (Nm) when the deadman is released

void setup() {
  // Start communication with the PC for debugging.
  Serial.begin(115200);
  delay(500);
  Serial.println("Starting ODrive control system with Joystick...");

  //  ODrive Initialization 
  odrive_serial.begin(baudrate, SERIAL_8N1, ODRIVE_RX_PIN, ODRIVE_TX_PIN);

  Serial.print("Checking ODrive connection...");
  float vbus = odrive.getParameterAsFloat("vbus_voltage");

  if (vbus > 5.0) {
      Serial.print(" OK! Vbus voltage: ");
      Serial.println(vbus);
  } else {
      Serial.println(" FAILED. Cannot communicate.");
      while(1);
  }

  // Set the control mode to VELOCITY.
  // It's better to do this once with odrivetool and save it.
  odrive_serial.print("w axis0.controller.config.control_mode 2\n");
  delay(100);

  Serial.println("Enabling Closed Loop Control...");
  odrive.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
  delay(500);

  Serial.println("ODrive ready!");

  //Joystick Pin Initialization 
  pinMode(deadmanSwitchPin, INPUT_PULLUP);
  pinMode(motorForwardPin, INPUT_PULLUP);
  pinMode(moveLeftPin, INPUT_PULLUP);
  pinMode(moveRightPin, INPUT_PULLUP);

  Serial.println("System ready to receive commands from the joystick.");
}

void loop() {
  // Reading the Joystick state
  bool deadmanPressed = (digitalRead(deadmanSwitchPin) == LOW);
  bool forwardPressed = (digitalRead(motorForwardPin) == LOW);
  bool leftPressed    = (digitalRead(moveLeftPin) == LOW);
  bool rightPressed   = (digitalRead(moveRightPin) == LOW);

  //Motor Control Logic
  float desiredVelocity = 0.0f; // Default velocity is 0 (stopped)

  // Only if the deadman is pressed, we consider movement.
  if (deadmanPressed) {
    // If the forward button is pressed, we set the forward velocity.
    if (forwardPressed) {
      desiredVelocity = FORWARD_VELOCITY;
    }
    // (Here you could add logic for "backward" if you had another button)

    // Send the velocity command to the ODrive
    odrive.setVelocity(desiredVelocity);

  } else {
    // If the deadman is NOT pressed, the motor must stop.
    // Sending setVelocity(0) is fine, but sometimes a braking torque
    // command gives a more "solid" feel. Try both.
    odrive.setVelocity(0.0f);
    // Alternative with active brake:
    // odrive_serial.print("w axis0.controller.config.control_mode 1\n"); // Torque mode
    // odrive.setTorque(0.0f); // Or a small braking torque
  }

  // (The logic for right/left/reset currently only prints messages,
  // it does not affect the motor)
  if(leftPressed) { /* Logic for turning left */ }
  if(rightPressed) { /* Logic for turning right */ }
  if(leftPressed && rightPressed) { /* Reset logic */ }


  //  Debug Print to Serial Monitor 
  static unsigned long last_print_time = 0;
  if (millis() - last_print_time > 200) {
    last_print_time = millis();

    Serial.print("Deadman: ");
    Serial.print(deadmanPressed ? "ON" : "OFF");
    Serial.print(" | Forward: ");
    Serial.print(forwardPressed ? "ON" : "OFF");
    Serial.print(" | Commanded Vel: ");
    Serial.print(desiredVelocity);

    // Read and print the actual velocity for comparison
    ODriveFeedback feedback = odrive.getFeedback();
    Serial.print(" | Actual Vel: ");
    Serial.println(feedback.vel);
  }
}
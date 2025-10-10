//  NECESSARY LIBRARIES
#include <ODriveUART.h>
#include <HardwareSerial.h>
#include <Dynamixel2Arduino.h>
#include <AS5600.h>
#include <Wire.h>

//  GENERAL CONFIGURATION
#define DEBUG_SERIAL Serial


//  SECTION 1: ODRIVE CONFIGURATION

HardwareSerial odrive_serial(1); // Use UART1 for ODrive
const int ODRIVE_BAUDRATE = 115200;
const int ODRIVE_RX_PIN = 18; // Connect to ODrive's TX
const int ODRIVE_TX_PIN = 19; // Connect to ODrive's RX
ODriveUART odrive(odrive_serial);
const float FORWARD_VELOCITY = 0.5f; // Velocity in turns/sec


//  SECTION 2: DYNAMIXEL AND SENSORS CONFIGURATION

#define DXL_SERIAL   Serial2 // Use UART2 for Dynamixel
#define DXL_BAUDRATE 1000000
const int DXL_DIR_PIN = 2;
const int DXL_RX_PIN  = 26;
const int DXL_TX_PIN  = 27;
const uint8_t DXL_ID = 1;
const float DXL_PROTOCOL_VERSION = 1.0;
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;

AS5600 as5600; // Angular sensor (optional, kept from the original code)

//  SECTION 3: INPUT AND MODE CONFIGURATION

//  Input Pins
const int deadmanSwitchPin = 13;
const int motorForwardPin  = 25;
const int moveLeftPin      = 12;
const int moveRightPin     = 14;
const int modeSwitchPin    = 4;

//  Operating Modes
enum ControlMode {
  MODE_MANUAL_JOYSTICK,
  MODE_AUTOMATIC_DEMO
};
ControlMode currentMode = MODE_MANUAL_JOYSTICK;

//  State Variables
// Manual
double dynamixelSetpoint = 512; // Target position for Dynamixel (0-1023)
// Automatic
int demoStatus = 0;
unsigned long demoStateTimer = 0;
// Timer
unsigned long lastSerialPrintTime = 0;
bool lastModeButtonState = HIGH;

//  SETUP
void setup() {
  DEBUG_SERIAL.begin(115200);
  delay(1000);
  DEBUG_SERIAL.println("\n--- Starting integrated ODrive + Dynamixel control system (corrected v.) ---");

  //  Input Pin Initialization
  pinMode(deadmanSwitchPin, INPUT_PULLUP);
  pinMode(motorForwardPin, INPUT_PULLUP);
  pinMode(moveLeftPin, INPUT_PULLUP);
  pinMode(moveRightPin, INPUT_PULLUP);
  pinMode(modeSwitchPin, INPUT_PULLUP);

  //  ODrive Initialization
  DEBUG_SERIAL.println("Initializing ODrive on Serial1...");
  odrive_serial.begin(ODRIVE_BAUDRATE, SERIAL_8N1, ODRIVE_RX_PIN, ODRIVE_TX_PIN);
  delay(500);
  if (odrive.getParameterAsFloat("vbus_voltage") < 5.0) {
      DEBUG_SERIAL.println("FATAL ERROR: Unable to communicate with ODrive.");
      while(1);
  }
  DEBUG_SERIAL.print("ODrive connected. Vbus voltage: ");
  DEBUG_SERIAL.println(odrive.getParameterAsFloat("vbus_voltage"));

  // CORRECTION: Calling setState without specifying the axis
  odrive.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
  delay(500);
  DEBUG_SERIAL.println("ODrive ready.");

  //  Dynamixel Initialization
  DEBUG_SERIAL.println("Initializing Dynamixel on Serial2...");
  DXL_SERIAL.begin(DXL_BAUDRATE, SERIAL_8N1, DXL_RX_PIN, DXL_TX_PIN);
  dxl.begin(DXL_BAUDRATE);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  if (!dxl.ping(DXL_ID)) {
    DEBUG_SERIAL.println("FATAL ERROR: Dynamixel servo not found.");
    while(1);
  }
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_POSITION);
  dxl.writeControlTableItem(MOVING_SPEED, DXL_ID, 150);
  dxl.torqueOn(DXL_ID);
  dynamixelSetpoint = dxl.getPresentPosition(DXL_ID); // Set the initial setpoint to the current position
  DEBUG_SERIAL.println("Dynamixel ready.");

  //  AS5600 Sensor Initialization (if used)
  Wire.begin(21, 22);
  if (as5600.begin() && as5600.isConnected()) {
    DEBUG_SERIAL.println("AS5600 angular sensor found.");
  }

  DEBUG_SERIAL.println("\n--- System ready. Initial mode: MANUAL ---");
}


//  OPERATING MODE FUNCTIONS


void runManualJoystickControl() {
  // --- Reading joystick state ---
  bool deadmanPressed = (digitalRead(deadmanSwitchPin) == LOW);
  bool forwardPressed = (digitalRead(motorForwardPin) == LOW);
  bool leftPressed    = (digitalRead(moveLeftPin) == LOW);
  bool rightPressed   = (digitalRead(moveRightPin) == LOW);

  float desiredVelocity = 0.0f;

  if (deadmanPressed) {
    // --- ODrive Motor Logic (Velocity) ---
    if (forwardPressed) {
      desiredVelocity = FORWARD_VELOCITY;
    }
    // Add logic for reverse here if needed

    //  Dynamixel Servo Logic (Position)
    if (leftPressed && !rightPressed) {
      dynamixelSetpoint -= 1;
    } else if (rightPressed && !leftPressed) {
      dynamixelSetpoint += 1;
    } else if (leftPressed && rightPressed) {
      dynamixelSetpoint = 477; // Reset to center position
    }

    // Constrain the setpoint within the servo limits
    dynamixelSetpoint = constrain(dynamixelSetpoint, 0, 1023);
    dxl.setGoalPosition(DXL_ID, (int)dynamixelSetpoint);

  } else {
    // If the deadman is not pressed, everything stops.
    desiredVelocity = 0.0f;
  }

  // CORRECTION: Send the velocity command without specifying the axis
  odrive.setVelocity(desiredVelocity);
}


void runAutomaticDemo() {
  unsigned long now = millis();

  // State machine for the demo
  switch(demoStatus) {
    case 0: // State 0: Forward and steer left
      DEBUG_SERIAL.println("DEMO: State 0 - Forward and Left");
      // CORRECTION: Calling setVelocity without specifying the axis
      odrive.setVelocity(FORWARD_VELOCITY * 0.5f); // Go forward at half speed
      dxl.setGoalPosition(DXL_ID, 200); // Steer left
      demoStateTimer = now;
      demoStatus = 1;
      break;

    case 1: // Wait 3 seconds
      if (now - demoStateTimer > 3000) {
        demoStatus = 2;
      }
      break;

    case 2: // State 2: Stop and steer right
      DEBUG_SERIAL.println("DEMO: State 2 - Stop and Right");
      // CORRECTION: Calling setVelocity without specifying the axis
      odrive.setVelocity(0.0f); // Stop
      dxl.setGoalPosition(DXL_ID, 800); // Steer right
      demoStateTimer = now;
      demoStatus = 3;
      break;

    case 3: // Wait 3 seconds
      if (now - demoStateTimer > 3000) {
        demoStatus = 0; // Restart the cycle
      }
      break;
  }
}

//  AUXILIARY FUNCTIONS (DEBUG, MODE MANAGEMENT)


void handleModeSwitch() {
  bool currentModeButtonState = digitalRead(modeSwitchPin);
  // Detect the transition from HIGH to LOW (button press)
  if (currentModeButtonState == LOW && lastModeButtonState == HIGH) {
    delay(50); // Simple debouncing

    if (currentMode == MODE_MANUAL_JOYSTICK) {
      currentMode = MODE_AUTOMATIC_DEMO;
      demoStatus = 0; // Reset the demo state
      DEBUG_SERIAL.println("\n--- Switched to AUTOMATIC DEMO Mode ---\n");
    } else {
      currentMode = MODE_MANUAL_JOYSTICK;
      // When exiting the demo, stop the motors for safety
      // CORRECTION: Calling setVelocity without specifying the axis
      odrive.setVelocity(0.0f);
      dynamixelSetpoint = dxl.getPresentPosition(DXL_ID); // Synchronize the setpoint
      DEBUG_SERIAL.println("\n--- Switched to MANUAL JOYSTICK Mode ---\n");
    }
  }
  lastModeButtonState = currentModeButtonState;
}

void handleSerialDebug() {
  if (millis() - lastSerialPrintTime >= 250) {
    lastSerialPrintTime = millis();

    if (currentMode == MODE_MANUAL_JOYSTICK) {
      DEBUG_SERIAL.print("MODE: MANUAL");
      // ODrive Info
      // CORRECTION: Calling getFeedback without specifying the axis
      ODriveFeedback feedback = odrive.getFeedback();
      DEBUG_SERIAL.print(" | ODrive Vel (Cmd/Actual): ");
      DEBUG_SERIAL.print(digitalRead(deadmanSwitchPin) == LOW && digitalRead(motorForwardPin) == LOW ? FORWARD_VELOCITY : 0.0f);
      DEBUG_SERIAL.print("/");
      DEBUG_SERIAL.print(feedback.vel);
      // Dynamixel Info
      int currentPos = dxl.getPresentPosition(DXL_ID);
      DEBUG_SERIAL.print(" | DXL Pos (Set/Actual): ");
      DEBUG_SERIAL.print((int)dynamixelSetpoint);
      DEBUG_SERIAL.print("/");
      DEBUG_SERIAL.print(currentPos);

    } else { // MODE_AUTOMATIC_DEMO
      DEBUG_SERIAL.print("MODE: DEMO");
      DEBUG_SERIAL.print(" | State: ");
      DEBUG_SERIAL.print(demoStatus);
      // CORRECTION: Calling getFeedback without specifying the axis
      ODriveFeedback feedback = odrive.getFeedback();
      DEBUG_SERIAL.print(" | ODrive Vel: ");
      DEBUG_SERIAL.print(feedback.vel);
      DEBUG_SERIAL.print(" | DXL Pos: ");
      DEBUG_SERIAL.print(dxl.getPresentPosition(DXL_ID));
    }
    DEBUG_SERIAL.println();
  }
}


//  MAIN LOOP

void loop() {
  // 1. Check if the mode needs to be changed
  handleModeSwitch();

  // 2. Execute the logic for the current mode
  if (currentMode == MODE_MANUAL_JOYSTICK) {
    runManualJoystickControl();
  } else { // MODE_AUTOMATIC_DEMO
    runAutomaticDemo();
  }

  // 3. Print debug information
  handleSerialDebug();
}
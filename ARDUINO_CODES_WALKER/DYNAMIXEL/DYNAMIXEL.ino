
  // FINAL CODE WITH NON-BLOCKING CONTROL AND MULTIPLE MODES
 
  // - MODE 1: Manual Control via Joystick.
  // - MODE 2: Automatic Demo with a state machine.
 
 //  - A button on PIN 4 allows switching between modes.
  // - All logic is managed with non-blocking timers based on millis().
 
 // - CORRECTED VERSION: Resolves the name conflict with the Dynamixel library.


//  NECESSARY LIBRARIES 
#include <Dynamixel2Arduino.h>
#include <AS5600.h>
#include <Wire.h>

//  GENERAL CONFIGURATION 
#define DXL_SERIAL   Serial1
#define DEBUG_SERIAL Serial

//  DYNAMIXEL SERVO CONFIGURATION 
#define AX_12A
#define DXL_BAUDRATE 1000000
const int DXL_DIR_PIN = 2;
const int DXL_RX_PIN  = 26;
const int DXL_TX_PIN  = 27;
const uint8_t DXL_ID = 1;
const float DXL_PROTOCOL_VERSION = 1.0;
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;

//  AS5600 ANGULAR SENSOR CONFIGURATION 
AS5600 as5600;
const int ZERO_OFFSET = 477;

//  INPUT PIN CONFIGURATION 
const int deadmanSwitchPin = 13;
const int motorForwardPin  = 25;
const int moveLeftPin      = 12;
const int moveRightPin     = 14;
const int modeSwitchPin    = 4;

//  OPERATING MODES DEFINITION 
// CORRECTION: Renamed from "OperatingMode" to "ControlMode" to avoid conflicts.
enum ControlMode {
  MODE_MANUAL_JOYSTICK,
  MODE_AUTOMATIC_DEMO
};
ControlMode currentMode = MODE_MANUAL_JOYSTICK; // Use the new type

//  VARIABLES FOR CONTROL LOGIC (MANUAL MODE) 
double setpoint = 515; // Target position in ticks (0-1023)
double input = 0.0;    // Current position read from the sensor

//  VARIABLES FOR DEMO LOGIC (AUTOMATIC MODE) 
int demoStatus = 0;
typedef struct {
  int currentPosition;
  int targetPosition;
  unsigned long timestamp;
  uint16_t timeInterval;
  bool isUpdating;
} DynamixelControl_t;
DynamixelControl_t servoDemo;

//  VARIABLES FOR NON-BLOCKING TIME MANAGEMENT 
unsigned long lastSerialPrintTime = 0;
bool lastModeButtonState = HIGH;



//  SETUP

void setup() {
  DEBUG_SERIAL.begin(115200);
  while (!DEBUG_SERIAL);

  pinMode(deadmanSwitchPin, INPUT_PULLUP);
  pinMode(motorForwardPin, INPUT_PULLUP);
  pinMode(moveLeftPin, INPUT_PULLUP);
  pinMode(moveRightPin, INPUT_PULLUP);
  pinMode(modeSwitchPin, INPUT_PULLUP);

  Wire.begin(21, 22);
  as5600.begin();
  if (!as5600.isConnected()) {
    DEBUG_SERIAL.println("FATAL ERROR: AS5600 sensor not found.");
    while (1);
  }

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

  servoDemo.currentPosition = dxl.getPresentPosition(DXL_ID);
  servoDemo.targetPosition = servoDemo.currentPosition;
  servoDemo.timeInterval = 10;
  servoDemo.timestamp = millis();
  servoDemo.isUpdating = false;

  DEBUG_SERIAL.println("\n--- System ready. Initial mode: MANUAL ---");
}


//  OPERATING MODE FUNCTIONS

void runManualJoystickControl() {
  bool deadmanPressed = (digitalRead(deadmanSwitchPin) == LOW);
  bool leftPressed    = (digitalRead(moveLeftPin) == LOW);
  bool rightPressed   = (digitalRead(moveRightPin) == LOW);

  if (deadmanPressed) {
    Serial.println("Button PRESSED: Deadman Switch  |");
    if (leftPressed && !rightPressed) {
      setpoint -= 2;
      Serial.println(" |Button PRESSED: Left Switch  |");
    } else if (rightPressed && !leftPressed) {
      setpoint += 2;
      Serial.println(" |Button PRESSED: right Switch  |");
    } else if (leftPressed && rightPressed) {
      Serial.println(" |Button PRESSED: Reset |");
      setpoint = 447;
    }

    setpoint = constrain(setpoint, 0, 1023);
    dxl.setGoalPosition(DXL_ID, (int)setpoint);
  }
}

void runAutomaticDemo() {
  if(demoStatus == 0) {
    demoStatus = 1;
    servoDemo.targetPosition = 100;
    servoDemo.isUpdating = true;
  } else if(demoStatus == 1) {
    if(servoDemo.isUpdating == false) {
      servoDemo.targetPosition = 900;
      servoDemo.isUpdating = true;
      demoStatus = 2;
    }
  } else if(demoStatus == 2) {
    if(servoDemo.isUpdating == false) {
      demoStatus = 0;
    }
  }

  if(servoDemo.isUpdating) {
    if(millis() - servoDemo.timestamp >= servoDemo.timeInterval) {
      if(servoDemo.currentPosition > servoDemo.targetPosition)
        servoDemo.currentPosition--;
      else if(servoDemo.currentPosition < servoDemo.targetPosition)
        servoDemo.currentPosition++;
      else
        servoDemo.isUpdating = false;

      dxl.setGoalPosition(DXL_ID, servoDemo.currentPosition);
      servoDemo.timestamp = millis();
    }
  }
}

void handleSerialDebug() {
  if (millis() - lastSerialPrintTime >= 200) {
    // CORRECTION: Use the new enum names
    if (currentMode == MODE_MANUAL_JOYSTICK) {
      DEBUG_SERIAL.print("MODE: MANUAL | Setpoint: ");
      DEBUG_SERIAL.print((int)setpoint);
      int currentPos = dxl.getPresentPosition(DXL_ID);
      DEBUG_SERIAL.print(" | Current Position: ");
      DEBUG_SERIAL.print(currentPos);
    } else { // MODE_AUTOMATIC_DEMO
      DEBUG_SERIAL.print("MODE: DEMO | Target: ");
      DEBUG_SERIAL.print(servoDemo.targetPosition);
      DEBUG_SERIAL.print(" | Current Command: ");
      DEBUG_SERIAL.print(servoDemo.currentPosition);
      DEBUG_SERIAL.print(" | Moving: ");
      DEBUG_SERIAL.print(servoDemo.isUpdating ? "YES" : "NO");
    }
    DEBUG_SERIAL.println();
    lastSerialPrintTime = millis();
  }
}



//  MAIN LOOP

void loop() {
  bool currentModeButtonState = digitalRead(modeSwitchPin);
  if (currentModeButtonState == LOW && lastModeButtonState == HIGH) {

    // CORRECTION: Use the new enum names
    if (currentMode == MODE_MANUAL_JOYSTICK) {
      currentMode = MODE_AUTOMATIC_DEMO;
      demoStatus = 0;
      servoDemo.currentPosition = dxl.getPresentPosition(DXL_ID);
      DEBUG_SERIAL.println("\n--- Switched to AUTOMATIC DEMO Mode ---\n");
    } else {
      currentMode = MODE_MANUAL_JOYSTICK;
      setpoint = dxl.getPresentPosition(DXL_ID);
      DEBUG_SERIAL.println("\n--- Switched to MANUAL JOYSTICK Mode ---\n");
    }
  }
  lastModeButtonState = currentModeButtonState;

  // CORRECTION: Use the new enum names
  if (currentMode == MODE_MANUAL_JOYSTICK) {
    runManualJoystickControl();
  } else { // MODE_AUTOMATIC_DEMO
    runAutomaticDemo();
  }

  handleSerialDebug();
}
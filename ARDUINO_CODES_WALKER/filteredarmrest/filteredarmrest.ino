
//MANUAL CONTROL+ARMREST SECURITY FILTERED

#include <ODriveUART.h>
#include <HardwareSerial.h>
#include <Dynamixel2Arduino.h>


//   1: HARDWARE CONFIGURATION
// ===================================================================
HardwareSerial odrive_serial(1);
const int ODRIVE_RX_PIN = 18, ODRIVE_TX_PIN = 19;
ODriveUART odrive(odrive_serial);

#define DXL_SERIAL Serial2
const int DXL_DIR_PIN = 2, DXL_RX_PIN = 26, DXL_TX_PIN = 27;
const uint8_t DXL_ID = 1;
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;

const int deadmanSwitchPin = 13, motorForwardPin = 25, moveLeftPin = 14, moveRightPin = 12;

const int hallPin1 = 34; // sx
const int hallPin2 = 32; // dx
const int hallPin3 = 35; // vtc

//   2: PARAMETRI DEL MODELLO E CONTROLLO

const float COMMANDED_FORWARD_VELOCITY = 0.5;
const int   DYNAMIXEL_SETPOINT_STEP = 10;
const float SOGLIA_APPOGGIO_MINIMO_N = 2.6;

const float L0_CM = 1.0, E_N_CM2 = 37.1;
const float R_CM = 0.12;
const float A_CM2_SINGLE = 3.14159 * R_CM * R_CM;
const float A_CM2_DOUBLE = 2 * A_CM2_SINGLE;

const float dist_calib_sens_cm[] = {2.0, 1.75, 1.65, 1.5, 1.35, 1.25, 1.15, 1.05, 0.95, 0.87, 0.8, 0.74, 0.68, 0.62, 0.58, 0.55, 0.5, 0.46, 0.42, 0.39, 0.36, 0.33, 0.3, 0.26, 0.23, 0.2, 0.15};
const float voltage1_calib_V[] = {1.52, 1.53, 1.54, 1.55, 1.57, 1.58, 1.59, 1.6, 1.65, 1.67, 1.72, 1.76, 1.8, 1.84, 1.93, 1.94, 2.09, 2.11, 2.25, 2.3, 2.42, 2.56, 2.6, 2.6, 2.6, 2.6, 2.6};
const float voltage2_calib_V[] = {1.53, 1.53, 1.55, 1.56, 1.57, 1.57, 1.59, 1.61, 1.64, 1.66, 1.7, 1.73, 1.79, 1.85, 1.87, 1.9, 1.99, 2.06, 2.12, 2.2, 2.32, 2.5, 2.58, 2.59, 2.61, 2.62, 2.63};
const float voltage3_calib_V[] = {1.54, 1.55, 1.56, 1.56, 1.58, 1.59, 1.59, 1.6, 1.63, 1.65, 1.66, 1.69, 1.72, 1.75, 1.8, 1.82, 1.87, 1.9, 1.97, 2.01, 2.05, 2.13, 2.19, 2.43, 2.61, 2.62, 2.63};
const int num_calib_points = sizeof(dist_calib_sens_cm) / sizeof(float);


//   3: MEDIAN FILTER AND CONTROL VARIABLES 
enum SystemState { IDLE, MOVING_FORWARD };
SystemState currentState = IDLE;

int dynamixelSetpoint = 512;
bool odrive_is_active = true;

// Variabili per il filtro mediano
float v1_history[3] = {0};
float v2_history[3] = {0};
float v3_history[3] = {0};
int history_index = 0;


//  SETUP
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\nAvvio sketch con sicurezza bracciolo e filtro mediano (v1.2)...");

    pinMode(deadmanSwitchPin, INPUT_PULLUP);
    pinMode(motorForwardPin, INPUT_PULLUP);
    pinMode(moveLeftPin, INPUT_PULLUP);
    pinMode(moveRightPin, INPUT_PULLUP);
    
    odrive_serial.begin(115200, SERIAL_8N1, ODRIVE_RX_PIN, ODRIVE_TX_PIN);
    delay(500);
    if (odrive.getParameterAsFloat("vbus_voltage") < 5.0) { while(1); }
    odrive.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
    
    DXL_SERIAL.begin(1000000, SERIAL_8N1, DXL_RX_PIN, DXL_TX_PIN);
    dxl.begin(1000000);
    dxl.setPortProtocolVersion(1.0);
    if (!dxl.ping(DXL_ID)) { while(1); }
    dxl.torqueOff(DXL_ID);
    dxl.setOperatingMode(DXL_ID, OP_POSITION);
    dxl.writeControlTableItem(MOVING_SPEED, DXL_ID, 150);
    dxl.torqueOn(DXL_ID);
    dynamixelSetpoint = dxl.getPresentPosition(DXL_ID);

    Serial.println("\nSistema pronto.");
}


//  AUSILIARY FUNCTIONS

//Helper function for median filter
float getMedian(float history[3]) {
    float sorted[3];
    for(int i=0; i<3; i++) sorted[i] = history[i];
    
    if (sorted[0] > sorted[1]) { float temp = sorted[0]; sorted[0] = sorted[1]; sorted[1] = temp; }
    if (sorted[1] > sorted[2]) { float temp = sorted[1]; sorted[1] = sorted[2]; sorted[2] = temp; }
    if (sorted[0] > sorted[1]) { float temp = sorted[0]; sorted[0] = sorted[1]; sorted[1] = temp; }
    
    return sorted[1]; // Ritorna il valore centrale
}

float interpolate(float x, const float x_map[], const float y_map[], int size) {
    if (x <= x_map[0]) return y_map[0];
    if (x >= x_map[size - 1]) return y_map[size - 1];
    for (int i = 0; i < size - 1; ++i) {
        if (x >= x_map[i] && x <= x_map[i + 1]) {
            float t = (x - x_map[i]) / (x_map[i + 1] - x_map[i]);
            return y_map[i] * (1 - t) + y_map[i + 1] * t;
        }
    }
    return y_map[0];
}

//  MAIN LOOP 
void loop() {
    // 1.  INPUT ACQUISITION
    bool deadmanPressed = (digitalRead(deadmanSwitchPin) == LOW);
    bool forwardPressed = (digitalRead(motorForwardPin) == LOW);
    bool leftPressed = (digitalRead(moveLeftPin) == LOW);
    bool rightPressed = (digitalRead(moveRightPin) == LOW);

    // 1.1 MEDIAN SENSORS FILTERING 

    float raw_v1 = analogRead(hallPin1) * (3.3 / 4095.0);
    float raw_v2 = analogRead(hallPin2) * (3.3 / 4095.0);
    float raw_v3 = analogRead(hallPin3) * (3.3 / 4095.0);

    v1_history[history_index] = raw_v1;
    v2_history[history_index] = raw_v2;
    v3_history[history_index] = raw_v3;
    history_index = (history_index + 1) % 3;

    float v1 = getMedian(v1_history);
    float v2 = getMedian(v2_history);
    float v3 = getMedian(v3_history);

    // 2. PHYSICAL MODEL 
    float d1 = interpolate(v1, voltage1_calib_V, dist_calib_sens_cm, num_calib_points);
    float d2 = interpolate(v2, voltage2_calib_V, dist_calib_sens_cm, num_calib_points);
    float d3 = interpolate(v3, voltage3_calib_V, dist_calib_sens_cm, num_calib_points);
    float F1 = (E_N_CM2 * A_CM2_SINGLE / L0_CM) * max(0.0f, L0_CM - d1);
    float F2 = (E_N_CM2 * A_CM2_SINGLE / L0_CM) * max(0.0f, L0_CM - d2);
    float F3 = (E_N_CM2 * A_CM2_DOUBLE  / L0_CM) * max(0.0f, L0_CM - d3);
    float Ftot_attuale = F1 + F2 + F3;
    bool isArmRested = (Ftot_attuale > SOGLIA_APPOGGIO_MINIMO_N);

    // 3. CONTROL LOGIC
    switch (currentState) {
      case IDLE:
        if (deadmanPressed && forwardPressed && isArmRested) {
          currentState = MOVING_FORWARD;
        }
        break;
      case MOVING_FORWARD:
        if ((deadmanPressed && !forwardPressed) || !isArmRested) {
          currentState = IDLE;
        }
        break;
    }

    // 4. MOTOR ACTION
    if (!isArmRested) {
        if (odrive_is_active) {
            odrive.setState(AXIS_STATE_IDLE);
            odrive_is_active = false;
        }
    } else {
        if (!odrive_is_active) {
            odrive.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
            odrive_is_active = true;
        }
        float desiredVelocity = 0.0f;
        if (currentState == MOVING_FORWARD) {
            desiredVelocity = COMMANDED_FORWARD_VELOCITY;
        }
        odrive.setVelocity(desiredVelocity);
    }

    if (deadmanPressed) {
      if (leftPressed && !rightPressed) dynamixelSetpoint -= DYNAMIXEL_SETPOINT_STEP;
      else if (rightPressed && !leftPressed) dynamixelSetpoint += DYNAMIXEL_SETPOINT_STEP;
      else if (leftPressed && rightPressed)  dynamixelSetpoint = 722; 
      dynamixelSetpoint = constrain(dynamixelSetpoint, 0, 1023);
    }
    dxl.setGoalPosition(DXL_ID, dynamixelSetpoint);

    // 5. DEBUG AND FREQUENCY CONTROL
    static long last_debug_print = 0;
    if (millis() - last_debug_print > 250) {
      int currentPos = dxl.getPresentPosition(DXL_ID);
      Serial.print("Posizione Attuale: ");
      Serial.print(currentPos);
      Serial.print(" | Stato: ");
      Serial.print(currentState == IDLE ? "IDLE" : "MOVING_FWD");
      Serial.print(" | Braccio Appoggiato: ");
      Serial.print(isArmRested ? "SI" : "NO");
      Serial.print(" | Ftot (filtrato): ");
      Serial.print(Ftot_attuale, 2);
      Serial.print(" | ODrive Attivo: ");
      Serial.println(odrive_is_active ? "SI" : "NO");
      last_debug_print = millis();
    }
    
    delay(20);
}
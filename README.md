# Master-degree-thesis
# Smart Unilateral Walker - Mechatronic Control System

This repository contains all the code, documentation, and data related to the Master's Thesis project for the development of a smart unilateral walker. The project focuses on creating a robust and accessible Human-Robot Interaction (HRI) system to enhance the safety and usability of assistive mobility devices.

The core of the project is an ESP32-based control system that integrates a custom force-sensing armrest, a joystick for user input, a high-torque traction subsystem, and a precise steering subsystem.

---

##  M_Features

*   **Dual-Mode Control:** Implements both a "Hold-to-Go" mode for precise maneuvering and a "Cruise Control" mode for reducing user fatigue on long paths.
*   **HRI Safety System:** Utilizes a custom-built, low-cost armrest sensor to estimate applied force. The system only allows movement when the user is in a stable, supported position.
*   **Modular Actuation:** Independently controls a high-power ODrive/hoverboard motor for traction and a Dynamixel smart servo for steering.
*   **Real-time Data Logging:** Firmware is equipped to stream all relevant system data (user inputs, sensor readings, motor commands, and encoder feedback) via serial in CSV format for offline analysis.

---

## F_System a_Architecture

The system is composed of two main mechatronic subsystems supervised by a central control unit:

1.  **Traction Subsystem:**
    *   **Motor:** Brushless DC Hub Motor (repurposed from a commercial hoverboard).
    *   **Controller:** ODrive v3.6, operating in velocity control mode.
2.  **Steering Subsystem:**
    *   **Actuator:** Dynamixel AX-12A smart servo, operating in position control mode.
    *   **Feedback Sensor:** AS5600 absolute magnetic encoder for precise angular position feedback.
3.  **Central Control Unit (ECU):**
    *   **Microcontroller:** ESP32-WROVER-E module.
    *   **Responsibilities:** Reading sensor data, processing the HRI safety logic, and sending real-time commands to the motor controllers.

---

## G_Getting s_Started

### Hardware Requirements

*   **Microcontroller:** ESP32-WROVER-E Development Board
*   **Traction:** ODrive v3.6, Brushless Hub Motor (e.g., from a hoverboard)
*   **Steering:** Dynamixel AX-12A Servo, AS5600 Magnetic Encoder
*   **HRI Armrest:** 3x SS49E Hall-effect sensors, 3x cylindrical EVA foam supports, 3x permanent magnets.
*   **Electronics:**
    *   SN74LS241N buffer (for Dynamixel half-duplex communication).
    *   2x Adjustable Buck Converters (DC-DC Step-Down) for 12V and 5V rails.
    *   Perfboard, wires, and connectors.
*   **Power:** 24V LiPo Battery (or similar, depending on ODrive configuration).

### Software Requirements

*   **IDE:** Arduino IDE (version 2.x recommended).
*   **Board Support:** ESP32 board package for Arduino IDE.
*   **Libraries:**
    *   `Dynamixel2Arduino`
    *   `ODriveArduino`
    *   `AS5600`
    *   `Wire`

### H_Hardware c_Connections

A detailed overview of the electronic connections:

*   **ODrive (Traction):**
    *   `GND`: Common Ground
    *   `GPIO1 (TX)`: ESP32 Pin 18 (RX for `Serial1`)
    *   `GPIO2 (RX)`: ESP32 Pin 19 (TX for `Serial1`)

*   **Dynamixel (Steering):**
    *   `GND`: Common Ground
    *   `VCC`: 12V Power Supply
    *   `DATA`: Connected via the SN74LS241N half-duplex buffer circuit to:
        *   ESP32 Pin 27 (TX for `Serial2`)
        *   ESP32 Pin 26 (RX for `Serial2`)
        *   ESP32 Pin 2 (Direction Pin)

*   **AS5600 (Steering Feedback):**
    *   `GND`: Common Ground
    *   `VCC`: 3.3V from ESP32
    *   `SDA`: ESP32 Pin 21
    *   `SCL`: ESP32 Pin 22

*   **Armrest Hall Sensors (HRI):**
    *   `GND`: Common Ground
    *   `VCC`: 5V Power Supply
    *   `Vout (Sensor 1)`: ESP32 Pin 34
    *   `Vout (Sensor 2)`: ESP32 Pin 32
    *   `Vout (Sensor 3)`: ESP32 Pin 35

---


## P_How to r_Run the s_System

1.  **Hardware Setup:** Assemble all hardware components according to the connection diagram and schematics.
2.  **ODrive & Dynamixel Configuration:** Ensure both the ODrive and Dynamixel have been pre-configured (motor calibration, setting IDs, baud rates, and control modes) using their respective software tools (`odrivetool` for ODrive, a setup sketch for Dynamixel).
3.  **Upload Firmware:** Open the main sketch located in `CODE/Final_Firmware/` with the Arduino IDE.
4.  **Select Board and Port:** Make sure you have the correct ESP32 board (`ESP32 Wrover Module`) and COM port selected.
5.  **Compile and Upload:** Upload the firmware to the ESP32.
6.  **Operation:** The system will initialize. Use the joystick to control the walker. To record data for analysis, open the Arduino IDE's Serial Monitor, set the baud rate to **115200**, and copy the CSV output into a text file.

---

## A_Author 
[NICOLETTA CINARDI] 
Master’s Thesis Project – Automation Engineering  
University of Catania

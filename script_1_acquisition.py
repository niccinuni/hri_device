import serial
import time
import csv
from datetime import datetime


# %% 0. ACQUISITION PARAMETERS

SERIAL_PORT = "COM3"
BAUD_RATE = 115200

# Measurement campaign
# Phase A: Holding the arm on the armrest for 30 seconds
ACQUISITION_DURATION_SEC = 30
MEASUREMENT_TYPE = "still_arm"  
#MEASUREMENT_TYPE = "dynamic_movements"

# Phase B: Dynamic movements
# ACQUISITION_DURATION_SEC = 30 
# MEASUREMENT_TYPE = "dynamic_movements" 

LOG_FILE_NAME = f"log_{MEASUREMENT_TYPE}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
PAUSE_DURATION = 0.001 # Minimum pause to avoid overloading the CPU

# %% 1. DATA ACQUISITION LOOP
continue_loop = True
ser = None

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    ser.flushInput()
    print(f'Serial connection on {SERIAL_PORT} established.')
except serial.SerialException as e:
    print(f'Serial error: {e}')
    continue_loop = False

#  Log File Setup 
log_file = open(LOG_FILE_NAME, "w", newline="")
csv_writer = csv.writer(log_file)
header = ['timestamp', 'v_left', 'v_right', 'v_vertical']
csv_writer.writerow(header)

input(f"\nReady for '{MEASUREMENT_TYPE}' acquisition. Press Enter to start...")
print(f"\nStarting acquisition for {ACQUISITION_DURATION_SEC} seconds... Press CTRL+C to stop.")
if MEASUREMENT_TYPE == "still_arm":
    print("Keep the arm resting and still.")
else:
    print("Perform slow and fast movements (forward, left, right, etc.).")

start_time = time.time()
sample_count = 0

try:
    while continue_loop and (time.time() - start_time < ACQUISITION_DURATION_SEC):
        if ser and ser.in_waiting > 0:
            try:
                line = ser.readline().decode('utf-8').strip()
                if line and len(line.split(',')) == 3:
                    timestamp = time.time()
                    # Order of arrival from Arduino: LEFT, RIGHT, VERTICAL
                    v_left, v_right, v_vertical = map(float, line.split(','))
                    
                    # Save the raw data
                    csv_writer.writerow([timestamp, v_left, v_right, v_vertical])
                    sample_count += 1
                    
                    # Print to screen
                    print(f"Samples: {sample_count} | Time left: {ACQUISITION_DURATION_SEC - (time.time() - start_time):.1f}s", end='\r')
            
            except (ValueError, UnicodeDecodeError):
                pass # Silently ignore corrupted lines
            except Exception as e:
                print(f"\nGeneric error in loop: {e}")
        
        time.sleep(PAUSE_DURATION)

except KeyboardInterrupt:
    print("\n\nAcquisition interrupted by user.")
finally:
    print('\nClosing...')
    if ser and ser.is_open:
        ser.close()
    if 'log_file' in locals():
        log_file.close()
        # Calculate effective sampling frequency
        if sample_count > 1:
            effective_fs = sample_count / ACQUISITION_DURATION_SEC
            print(f"Acquisition complete. {sample_count} samples saved.")
            print(f"Effective sampling frequency: {effective_fs:.2f} Hz")
        print(f"Data saved to: {LOG_FILE_NAME}")
    print('Acquisition script finished.')


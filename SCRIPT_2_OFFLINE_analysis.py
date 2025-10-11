import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt

#  Matplotlib Plotting Style Settings 
plt.rc('font', size=20)          
plt.rc('axes', titlesize=18)     
plt.rc('axes', labelsize=20)     
plt.rc('xtick', labelsize=14)    
plt.rc('ytick', labelsize=14)    
plt.rc('legend', fontsize=20)    
plt.rc('figure', titlesize=20)   


# %% 0. ANALYSIS SETTINGS
# !!! THE FILENAME MUST BE CHANGED HERE DEPENDING ON WHICH DATA TO ANALYZE !!!
DATA_FILE_NAME = "log_still_arm_20250912_185252.csv" # Example
# DATA_FILE_NAME = "log_dynamic_movements_20250912_185525.csv"

# Choose which signal to analyze: 'v_left', 'v_right', or 'v_vertical'
SIGNAL_TO_ANALYZE = 'v_right'


# Filter parameters, to be adjusted after the FFT analysis
CUTOFF_FREQUENCY_HZ = 8.0  # Try with 10Hz, 8Hz, 5Hz...
FILTER_ORDER = 4           # An order of 2 is a great compromise


# %% 1. DATA LOADING
try:
    df = pd.read_csv(DATA_FILE_NAME)
    print(f"Loaded {len(df)} samples from '{DATA_FILE_NAME}'.")
except FileNotFoundError:
    print(f"ERROR: File '{DATA_FILE_NAME}' not found.")
    exit()

#  Signal and Time Axis Preparation 
timestamps = df['timestamp'].to_numpy()
Fs = 1.0 / np.mean(np.diff(timestamps))
print(f"Estimated actual sampling frequency: {Fs:.2f} Hz")

signal = df[SIGNAL_TO_ANALYZE].to_numpy()
N = len(signal)
time_axis = (timestamps - timestamps[0])


# %% 2. SPECTRUM ANALYSIS (FFT)

print("\nPerforming FFT analysis...")
signal_for_fft = signal - np.mean(signal) # Remove the DC component
fft_vals = np.fft.rfft(signal_for_fft)
freqs = np.fft.rfftfreq(N, d=1/Fs)

#  Plot Frequency Spectrum 
plt.figure(figsize=(12, 6))
plt.plot(freqs, np.abs(fft_vals))
plt.title(f'Frequency Spectrum (FFT) of signal {SIGNAL_TO_ANALYZE}')
plt.xlabel("Frequency [Hz]"); plt.ylabel("Amplitude")
plt.grid(True)
plt.xlim(0, Fs / 2)
plt.axvline(CUTOFF_FREQUENCY_HZ, color='r', linestyle='--', label=f'Cutoff at {CUTOFF_FREQUENCY_HZ} Hz')
plt.legend()
plt.suptitle("Use this plot to choose the cutoff frequency")
print(f"FFT plot generated. Observe where the energy is concentrated and where the noise begins.")


# %% 3. FILTER APPLICATION AND COMPARISON

#  Design the Butterworth filter 
b, a = butter(FILTER_ORDER, CUTOFF_FREQUENCY_HZ, btype='low', analog=False, fs=Fs)
print(f"\nButterworth filter (Order={FILTER_ORDER}, Cutoff={CUTOFF_FREQUENCY_HZ} Hz) designed.")

#  Apply the filter 
filtered_signal = filtfilt(b, a, signal)

#  Comparison Plot: Raw vs. Filtered 
plt.figure(figsize=(15, 7))
plt.plot(time_axis, signal, 'b-', alpha=0.5, label='Raw Signal')
plt.plot(time_axis, filtered_signal, 'r-', linewidth=2, label='Filtered Signal')
plt.title(f'Raw vs. Filtered Signal ({SIGNAL_TO_ANALYZE})')
plt.xlabel("Time [s]"); plt.ylabel("Voltage [V]")
plt.legend(); plt.grid(True)
plt.show()

print("\nAnalysis complete.")
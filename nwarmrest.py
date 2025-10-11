import serial
import time
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import PchipInterpolator
from scipy.signal import butter, lfilter
import collections
import math
import csv
from datetime import datetime


# %% 0. CONTROL PARAMETERS AND GEOMETRY

PORTA_SERIALE = "COM3"; BAUD_RATE = 115200
P1_POS_CM = np.array([0.0, 0.0]); P2_POS_CM = np.array([2.59, 0.0]); P3_POS_CM = np.array([1.30, 14.94])
SOGLIA_APPOGGIO_MINIMO_N = 1.0; MOTOR_PWM_STOP = 128; MOTOR_PWM_MAX_FORWARD = 255
SOGLIA_VELOCITA_COP_CRITICA = 20.0; DEAD_ZONE_RAGGIO_CM = 5.5
CALIB_APPOGGIO_DURATION_SEC = 5; SOGLIA_AUMENTO_FTOT_PER_AVANTI_N = 0.5
SOGLIA_AUMENTO_F3_PER_AVANTI_N = 0.3; PLOT_BUFFER_SIZE = 50; PAUSE_DURATION = 0.01
SAMPLING_RATE_HZ = 50.0; CUTOFF_FREQUENCY_HZ = 8.0; BUTTERWORTH_ORDER = 2
MODO_ACQUISIZIONE = False
NOME_FILE_LOG = f"log_dati_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"


# %% 1. CALIBRATION AND FILTER PARAMETERS

masse_g = np.array([0, 180, 382, 550, 753, 954, 1156, 1324, 1525])
forza_calib_N = masse_g / 1000 * 9.81
tensione_calib_sx = np.array([1.71, 1.76, 1.87, 1.996, 1.999, 2.004, 2.01, 2.02, 2.024])
tensione_calib_dx = np.array([1.60, 1.64, 1.7, 1.76, 1.87, 1.999, 2.000, 2.01, 2.019])
tensione_calib_vtc = np.array([1.72, 1.87, 1.96, 2.05, 2.18, 2.21, 2.23, 2.24, 2.249])

def create_force_interpolator(t_calib, f_calib):
    t_unique, idx = np.unique(t_calib, return_index=True); f_for_interp = f_calib[idx]
    sort_indices = np.argsort(t_unique); t_sorted = t_unique[sort_indices]; f_sorted = f_for_interp[sort_indices]
    return PchipInterpolator(t_sorted, f_sorted, extrapolate=True)

force_interpolator1 = create_force_interpolator(tensione_calib_sx, forza_calib_N) # P1 è SX
force_interpolator2 = create_force_interpolator(tensione_calib_dx, forza_calib_N) # P2 è DX
force_interpolator3 = create_force_interpolator(tensione_calib_vtc, forza_calib_N) # P3 è VTC

V_MIN_CALIBRATED = 1.60; V_MAX_CALIBRATED = 2.25
def get_force_from_voltage(v, interpolator, v_min, v_max):
    v_clamped = np.clip(v, v_min, v_max); force = interpolator(v_clamped)
    return max(0, force)

b, a = butter(BUTTERWORTH_ORDER, CUTOFF_FREQUENCY_HZ, btype='low', analog=False, fs=SAMPLING_RATE_HZ)
print(f"Filtro Butterworth di ordine {BUTTERWORTH_ORDER} progettato con cutoff a {CUTOFF_FREQUENCY_HZ} Hz.")


# %% 2. INITIALIZATIONA ND MAIN LOOP
continue_loop = True; ser = None
F1_appoggio, F2_appoggio, F3_appoggio, Ftot_appoggio = 0.0, 0.0, 0.0, 0.0
CoP_X_appoggio, CoP_Y_appoggio = np.nan, np.nan

try:
    ser = serial.Serial(PORTA_SERIALE, BAUD_RATE, timeout=0.1)
    ser.flushInput(); ser.flushOutput(); print(f'Connessione seriale su {PORTA_SERIALE} stabilita.')
except serial.SerialException as e: print(f'Errore seriale: {e}'); ser = None

def calibra_stato_appoggio(ser_conn, duration_sec):
    global F1_appoggio, F2_appoggio, F3_appoggio, Ftot_appoggio, CoP_X_appoggio, CoP_Y_appoggio
    print(f"\nINIZIO CALIBRAZIONE: Appoggiare il braccio normalmente per {duration_sec} sec...")
    temp_f1, temp_f2, temp_f3, temp_ftot, temp_copx, temp_copy = [], [], [], [], [], []
    start_time = time.time()
    ser_conn.flushInput()
    while time.time() - start_time < duration_sec:
        if ser_conn.in_waiting > 0:
            try:
                line = ser_conn.readline().decode('utf-8').strip()
                if len(line.split(',')) == 3:
                    # Ordine di arrivo: SX, DX, VTC
                    v_sx, v_dx, v_vtc = map(float, line.split(','))
                    
                    f1 = get_force_from_voltage(v_sx, force_interpolator1, V_MIN_CALIBRATED, V_MAX_CALIBRATED)
                    f2 = get_force_from_voltage(v_dx, force_interpolator2, V_MIN_CALIBRATED, V_MAX_CALIBRATED)
                    f3 = get_force_from_voltage(v_vtc, force_interpolator3, V_MIN_CALIBRATED, V_MAX_CALIBRATED)
                    ftot = f1 + f2 + f3
                    if ftot > 0.01:
                        temp_f1.append(f1); temp_f2.append(f2); temp_f3.append(f3); temp_ftot.append(ftot)
                        temp_copx.append((P1_POS_CM[0]*f1+P2_POS_CM[0]*f2+P3_POS_CM[0]*f3)/ftot)
                        temp_copy.append((P1_POS_CM[1]*f1+P2_POS_CM[1]*f2+P3_POS_CM[1]*f3)/ftot)
                print(f"Calibrazione... {int(duration_sec-(time.time()-start_time))}s", end='\r')
            except Exception as e: print(f"Errore calibrazione: {e}")
        time.sleep(0.01)
    if temp_ftot:
        F1_appoggio=np.mean(temp_f1); F2_appoggio=np.mean(temp_f2); F3_appoggio=np.mean(temp_f3)
        Ftot_appoggio = np.mean(temp_ftot); CoP_X_appoggio = np.mean(temp_copx); CoP_Y_appoggio = np.mean(temp_copy)
        print("\nCALIBRAZIONE COMPLETATA.")
        print(f"  Ftot App: {Ftot_appoggio:.2f}N, CoP App (x,y): ({CoP_X_appoggio:.2f}, {CoP_Y_appoggio:.2f})cm")
    else: print("\nERRORE: No dati validi in calibrazione.")

if ser: calibra_stato_appoggio(ser, CALIB_APPOGGIO_DURATION_SEC)
else: print("Seriale non attiva, salto calibrazione.")

plt.ion()
fig, ax_cop = plt.subplots(figsize=(8,6)); fig.canvas.manager.set_window_title('Monitoraggio CoP e Controllo Motore')
def on_close_simple(event): global continue_loop; continue_loop = False; print("Chiusura.")
fig.canvas.mpl_connect('close_event', on_close_simple)
BB_MIN_X=min(P1_POS_CM[0],P2_POS_CM[0],P3_POS_CM[0]); BB_MAX_X=max(P1_POS_CM[0],P2_POS_CM[0],P3_POS_CM[0])
BB_MIN_Y=min(P1_POS_CM[1],P2_POS_CM[1],P3_POS_CM[1]); BB_MAX_Y=max(P1_POS_CM[1],P2_POS_CM[1],P3_POS_CM[1])
ax_cop.plot([P1_POS_CM[0],P2_POS_CM[0],P3_POS_CM[0],P1_POS_CM[0]], [P1_POS_CM[1],P2_POS_CM[1],P3_POS_CM[1],P1_POS_CM[1]], 'k-', label='Area Sensori')
if not np.isnan(CoP_X_appoggio):
    dead_zone_circle = plt.Circle((CoP_X_appoggio, CoP_Y_appoggio), DEAD_ZONE_RAGGIO_CM, color='m', fill=False, linestyle='--', label='Dead Zone')
    ax_cop.add_patch(dead_zone_circle)
cop_trace, = ax_cop.plot([],[],'b.-',label='Traccia CoP'); cop_point, = ax_cop.plot([],[],'ro',ms=10,label='CoP Attuale')
ax_cop.set_xlabel('X (cm)'); ax_cop.set_ylabel('Y (cm)'); ax_cop.set_title('CoP e Stato del Sistema'); ax_cop.grid(True); ax_cop.axis('equal'); ax_cop.set_xlim(BB_MIN_X-2, BB_MAX_X+2); ax_cop.set_ylim(BB_MIN_Y-2, BB_MAX_Y+2)
ax_cop.legend(loc='best')
text_handle = ax_cop.text(0.02, 0.98, 'In attesa...', transform=ax_cop.transAxes, fontsize=9, va='top', family='monospace', bbox=dict(boxstyle='round', fc='wheat', alpha=0.8))
plot_cop_x = collections.deque(maxlen=PLOT_BUFFER_SIZE); plot_cop_y = collections.deque(maxlen=PLOT_BUFFER_SIZE)
last_pwm = -1; prev_time = time.time()
prev_filt_x = np.nan; prev_filt_y = np.nan
buffer_len = BUTTERWORTH_ORDER * 3
buffer_x_raw = collections.deque(maxlen=buffer_len); buffer_y_raw = collections.deque(maxlen=buffer_len)

def clamp_to_bounding_box(cop_x, cop_y, min_x_bb, max_x_bb, min_y_bb, max_y_bb):
    if np.isnan(cop_x) or np.isnan(cop_y): return np.nan, np.nan
    return np.clip(cop_x, min_x_bb, max_x_bb), np.clip(cop_y, min_y_bb, max_y_bb)

print(f'\nAvvio monitoraggio con filtro Butterworth...')
log_file = open(NOME_FILE_LOG, "w", newline=""); csv_writer = csv.writer(log_file)
csv_writer.writerow(['timestamp', 'v_sx', 'v_dx', 'v_vtc', 'F1_sx', 'F2_dx', 'F3_vtc', 'F_tot', 'CoP_X_filt', 'CoP_Y_filt', 'PWM_out', 'Status'])

try:
    while continue_loop:
        if not ser or not ser.is_open: time.sleep(1); continue
        now = time.time(); dt = now - prev_time
        if dt < 1e-6: dt = PAUSE_DURATION
        
        if ser.in_waiting > 0:
            try:
                line = ser.readline().decode('utf-8').strip()
                if len(line.split(',')) == 3:
                    # Ordine di arrivo: SX, DX, VTC
                    v_sx, v_dx, v_vtc = map(float, line.split(','))
                    
                    F1 = get_force_from_voltage(v_sx, force_interpolator1, V_MIN_CALIBRATED, V_MAX_CALIBRATED)
                    F2 = get_force_from_voltage(v_dx, force_interpolator2, V_MIN_CALIBRATED, V_MAX_CALIBRATED)
                    F3 = get_force_from_voltage(v_vtc, force_interpolator3, V_MIN_CALIBRATED, V_MAX_CALIBRATED)
                    F_tot = F1 + F2 + F3

                    cop_x_raw, cop_y_raw = np.nan, np.nan
                    if F_tot > 0.05:
                        cop_x_raw = (P1_POS_CM[0]*F1 + P2_POS_CM[0]*F2 + P3_POS_CM[0]*F3) / F_tot
                        cop_y_raw = (P1_POS_CM[1]*F1 + P2_POS_CM[1]*F2 + P3_POS_CM[1]*F3) / F_tot
                    
                    cx, cy = clamp_to_bounding_box(cop_x_raw, cop_y_raw, BB_MIN_X, BB_MAX_X, BB_MIN_Y, BB_MAX_Y)
                    
                    fx, fy = np.nan, np.nan
                    buffer_x_raw.append(cx if not np.isnan(cx) else (list(buffer_x_raw)[-1] if buffer_x_raw else 0))
                    buffer_y_raw.append(cy if not np.isnan(cy) else (list(buffer_y_raw)[-1] if buffer_y_raw else 0))

                    if len(buffer_x_raw) >= buffer_len:
                        x_filtered_series = lfilter(b, a, list(buffer_x_raw)); fx = x_filtered_series[-1]
                    if len(buffer_y_raw) >= buffer_len:
                        y_filtered_series = lfilter(b, a, list(buffer_y_raw)); fy = y_filtered_series[-1]
                    
                    print(f"V(sx,dx,vtc):({v_sx:.2f},{v_dx:.2f},{v_vtc:.2f})|F(1,2,3):({F1:.1f},{F2:.1f},{F3:.1f})|CoP Raw:({cop_x_raw:.2f},{cop_y_raw:.2f})|CoP Filt:({fx:.2f},{fy:.2f})")
                    
                    vx = (fx - prev_filt_x)/dt if not np.isnan(fx) and not np.isnan(prev_filt_x) else np.nan
                    vy = (fy - prev_filt_y)/dt if not np.isnan(fy) and not np.isnan(prev_filt_y) else np.nan
                    prev_filt_x, prev_filt_y = fx, fy
                    
                    pwm, status = MOTOR_PWM_STOP, ""
                    is_jerk = (not np.isnan(vy) and abs(vy) > SOGLIA_VELOCITA_COP_CRITICA)
                    
                    if F_tot < SOGLIA_APPOGGIO_MINIMO_N: status = "(No App.)"
                    elif is_jerk: status = "(Mov. Brusco)"
                    elif np.isnan(fx) or np.isnan(CoP_X_appoggio): status = "(CoP/Appoggio NaN)"
                    else:
                        dx_cop = fx - CoP_X_appoggio; dy_cop = fy - CoP_Y_appoggio
                        dist_spostamento = math.sqrt(dx_cop**2 + dy_cop**2)
                        dF_tot = F_tot - Ftot_appoggio; dF3 = F3 - F3_appoggio
                        is_moving_away = dist_spostamento > DEAD_ZONE_RAGGIO_CM
                        is_pushing = (dF_tot > SOGLIA_AUMENTO_FTOT_PER_AVANTI_N) or (dF3 > SOGLIA_AUMENTO_F3_PER_AVANTI_N)
                        if is_moving_away and is_pushing:
                            pwm = MOTOR_PWM_MAX_FORWARD; status = "AVANTI (ON)"
                        else: status = "(Appoggio/Neutro)"
                    
                    if is_jerk: pwm = MOTOR_PWM_STOP
                    if not MODO_ACQUISIZIONE:
                        if pwm != last_pwm: ser.write(f"M{int(pwm)}\n".encode('utf-8')); last_pwm = pwm
                    else: pwm = MOTOR_PWM_STOP; status = f"(ACQUISIZIONE) {status}"

                    if not np.isnan(fx): plot_cop_x.append(fx); plot_cop_y.append(fy)
                    cop_trace.set_data(list(plot_cop_x), list(plot_cop_y)); cop_point.set_data([fx], [fy])
                    text_handle.set_text(f"Stato: {status}\n\nFtot:{F_tot:.1f}N\n"
                                         f"CoP (x,y): ({fx:.2f}, {fy:.2f})cm\nVel Y:{vy:.1f}cm/s\n\n"
                                         f"PWM: {pwm}")
                    fig.canvas.draw_idle()

                    csv_writer.writerow([now, v_sx, v_dx, v_vtc, F1, F2, F3, F_tot, fx, fy, pwm, status])
            except Exception as e: print(f"Err ciclo: {e}, '{line}'")
        
        prev_time = now
        plt.pause(PAUSE_DURATION)
except KeyboardInterrupt: print("Interruzione.")
finally:
    print('Chiusura...');
    if ser and ser.is_open:
        try: ser.write(f"M{MOTOR_PWM_STOP}\n".encode('utf-8')); ser.close()
        except: pass
    if 'log_file' in globals():
        log_file.close(); print(f"File di log salvato: {NOME_FILE_LOG}")
    if 'fig' in globals() and plt.fignum_exists(fig.number): plt.close(fig)
    print('Terminato.')
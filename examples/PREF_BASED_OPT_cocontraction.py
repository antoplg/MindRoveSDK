import csv
import sys
import os
import time
import glob
import serial
import threading
import argparse
from pathlib import Path
import numpy as np
import pyqtgraph as pg
from scipy.stats import norm
from serial.tools import list_ports
from pyqtgraph.Qt import QtCore, QtWidgets
from scipy.signal import butter, iirnotch, filtfilt, lfilter, lfilter_zi
from mindrove.board_shim import BoardShim, MindRoveInputParams, BoardIds
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF, ConstantKernel as C
from NMF import compute_synergy_metrics, Perform_NMF, trim_percent

# ======= Parametri generali =======
BAUD_RATE       = 115200
SERIAL_TIMEOUT  = 0.01     # s, lettura non bloccante
TX_HEADER       = b'V'     # header per inviare velocità (float32)
PRINT_EVERY     = 0.2      # s, quanto spesso stampare feedback a video

NUM_ITERATIONS  = 10
RANGE_A         = (0.0, 1.0)
MIN_DISTANCE    = 0.15     # distanza minima tra campioni di 'a'
N_SAMPLES       = 1000     # candidati per la policy di acquisizione

INVERSION   = True
SERIAL_COM  = True
baud_rate   = 115200

if SERIAL_COM:
    port_list = list(list_ports.comports(include_links=False))
    port_arduino = port_list[0].device
    print("[INFO] Using serial port:", port_arduino)
    ser = serial.Serial(port_arduino, baud_rate)

# ───── Qt App ───────────────────────────────────────────────
app = QtWidgets.QApplication(sys.argv)

# ───── Start MindRove streaming ─────────────────────────────
BoardShim.enable_dev_board_logger()
params   = MindRoveInputParams()
board_id = BoardIds.MINDROVE_WIFI_BOARD
board    = BoardShim(board_id, params)
board.prepare_session()
board.start_stream()

# ───── Prepare channels & buffers ──────────────────────────
emg_ch      = BoardShim.get_emg_channels(board_id)
sr          = BoardShim.get_sampling_rate(board_id)
window_s    = 10
N           = int(window_s * sr)
window_s2   = 1
N2          = int(window_s2 * sr)

# DESIGN FILTRI EMG
b_notch, a_notch = iirnotch(50.0/(sr/2), Q=30.0)
b_hp, a_hp       = butter(4, 20.0/(sr/2), btype='highpass')
zi_hp            = lfilter_zi(b_hp, a_hp)

# Buffer circolare per cocontrazione
cocon_buf = np.zeros(N2)

# GP (preferenze binarie → y=1 vince, y=0 perde; qui usiamo surrogate regression)
kernel = C(1.0, (1e-2, 1e2)) * RBF(length_scale=0.4, length_scale_bounds=(1e-2, 1e1))
gp     = GaussianProcessRegressor(kernel=kernel, alpha=1e-2, normalize_y=True)

def compute_trimmed_minmax(data, percent=10):
    """Calcola min e max robusti scartando percent% in alto e in basso."""
    data = np.sort(np.array(data).flatten())
    n = len(data)
    k = int(n * percent / 100.0)
    if n <= 2 * k:
        return np.min(data), np.max(data)
    trimmed = data[k:-k]
    return np.min(trimmed), np.max(trimmed)

def load_calibration_cocon(csv_path="cocontraction_session.csv", percent=10):
    """
    Carica il file CSV di calibrazione della cocontrazione,
    e calcola i valori min e max robusti scartando percent% in alto e in basso.
    """
    p = Path(csv_path)
    if not p.exists():
        print(f"[WARN] Calibration file {csv_path} not found. Using defaults 0.0–1.0")
        return 0.0, 1.0

    data = np.loadtxt(p, delimiter=',', skiprows=1)  # assume header
    cocon_min, cocon_max = compute_trimmed_minmax(data, percent)
    print(f"[INFO] Calibration cocontrazione: min={cocon_min:.3f}, max={cocon_max:.3f}")
    return cocon_min, cocon_max

COCON_MIN, COCON_MAX = load_calibration_cocon("cocontraction_session.csv", percent=trim_percent)

# ───── Build pyqtgraph window ──────────────────────────────
# pg.setConfigOption('background', 'w')
# pg.setConfigOption('foreground', 'k')
win   = pg.GraphicsLayoutWidget(title="Live Cocontraction", show=True)
# p     = win.addPlot(row=0, col=0, title="Cocontraction (normalized)")
# curve = p.plot(pen=pg.mkPen(color=(0, 150, 0), width=2))
# p.showGrid(x=True, y=True, alpha=0.3)

# Second plot: mapping function
p_map   = win.addPlot(row=1, col=0, title="Mapping cocontraction → velocity")
curve_map = p_map.plot(pen=pg.mkPen(color=(200, 0, 0), width=2))
p_map.showGrid(x=True, y=True, alpha=0.3)
p_map.setLabel('bottom', 'Cocontraction')
p_map.setLabel('left', 'Velocity')

# Fissa i range e impedisci autoscale
p_map.setXRange(0, 1)
p_map.setYRange(0, 1)
p_map.enableAutoRange(axis='x', enable=False)
p_map.enableAutoRange(axis='y', enable=False)

# ======= Mappa velocità =======
def velocity_from_cocon(cocon: float, a: float) -> float:
    """
    f(x) = (exp((a*3.9 - 0.9)*x) - 1) / (3.9*a - 0.9)
    - Coccon atteso ~ [0,1] (non clampiamo >1: passa così com'è)
    - 'a' in [0,1]
    Gestione caso denominatore ~ 0 (a ≈ 0.23077): uso limite ~ x
    """
    k = 3.9*a - 0.9
    x = float(cocon)
    if abs(k) < 1e-6:
        # limite di (e^{k x}-1)/k per k->0 è x
        return x
    return (np.exp(k*x) - 1.0) / k

def plot_mapping_function(a_value):
    x_vals = np.linspace(0, 1, 200)   # cocontraction range
    y_vals = [velocity_from_cocon(x, a_value) for x in x_vals]
    curve_map.setData(x_vals, y_vals)


# ======= Carica dati da file =======

def load_data(path):
    ext = os.path.splitext(path)[1].lower()
    if ext == '.npy':
        return np.load(path)
    elif ext in ('.csv', '.txt'):
        return np.loadtxt(path, delimiter=',', skiprows=1)
    else:
        raise ValueError(f"Unsupported extension {ext}")

def find_default_file():
    for ext in ('npy','csv','txt'):
        matches = glob.glob(f"emg_session*.{ext}")
        if matches:
            return matches[0]
    return None

parser = argparse.ArgumentParser(description="EMG → NMF (cocontraction only)")
parser.add_argument('file', nargs='?', help="Path a emg_session.npy/.csv (default: primo emg_session.*)")
parser.add_argument('--fs', type=float, default=250, help="Frequenza di campionamento (Hz), default=250")
parser.add_argument('--rms-win', type=float, default=0.2, help="Lunghezza finestra RMS (s), default 0.2")
args = parser.parse_args()

fp = args.file or find_default_file()
if not fp:
    print("Nessun file trovato emg_session.*", file=sys.stderr)
    sys.exit(1)

data_offline        = load_data(fp)
ns, nc              = data_offline.shape
data_offline_abs    = np.abs(data_offline)
data_offline_rms    = np.zeros_like(data_offline_abs)

win_samps = int(args.rms_win * args.fs)
if win_samps < 1:
    win_samps = 1
kernel = np.ones(win_samps) / win_samps
for ch in range(nc):
    sq   = data_offline_abs[:, ch] ** 2
    mov  = np.convolve(sq, kernel, mode='same')
    data_offline_rms[:, ch] = np.sqrt(mov)

H, W = Perform_NMF(data_offline_rms)
k_ext_max, k_flex_max, k_ext_min, k_flex_min = compute_synergy_metrics(H, trim_percent)
W_pinv = np.linalg.pinv(W)

# ======= Utility: thread per attendere Invio =======
class EnterWaiter:
    def __init__(self):
        self._event = threading.Event()
        self._thread = threading.Thread(target=self._waiter, daemon=True)

    def _waiter(self):
        try:
            input("\n  ▶ Premi INVIO quando vuoi passare al valore successivo...\n")
        except EOFError:
            pass
        self._event.set()

    def start(self):
        self._thread.start()

    def is_set(self):
        return self._event.is_set()

    def wait(self):
        self._event.wait()

# ───── Update callback ─────────────────────────────────────
RMS_WINDOW_S                            = 0.3
rms_win_samps                           = int(RMS_WINDOW_S * sr)
if rms_win_samps < 1: rms_win_samps     = 1
rms_kernel                              = np.ones(rms_win_samps) / rms_win_samps
cocon_smooth                            = 0.0
last_print                              = 0.0  # global to control console feedback

def update(a_value):
    global cocon_buf, cocon_smooth, last_print

    count = board.get_board_data_count()
    if count <= 0:
        return

    data_raw = board.get_board_data(N)

    if data_raw.shape[1] < 20:   # 20 > padlen(15) safe margin
        return
    
    # Filtra EMG
    data_filt = np.zeros_like(data_raw)
    for ch in emg_ch:
        tmp = filtfilt(b_notch, a_notch, data_raw[ch])
        data_filt[ch] = filtfilt(b_hp, a_hp, tmp)
        # data_filt[ch], zi_hp = lfilter(b_hp, a_hp, data_raw[ch], zi=zi_hp)

    # RMS
    rect    = np.abs(data_filt)
    rms     = np.array([
        np.sqrt(np.convolve(rect[ch]**2, rms_kernel, mode='same'))
        for ch in emg_ch
    ])
    x_new   = rms[:, -1]

    # Attivazioni NMF
    h_new       = x_new.dot(W_pinv)
    flex_norm   = max(0.0, (h_new[1] - k_flex_min) / (k_flex_max - k_flex_min))
    ext_norm    = max(0.0, (h_new[0] - k_ext_min)  / (k_ext_max  - k_ext_min))

    # Cocontrazione grezza
    cocon_raw   = min(flex_norm, ext_norm)

    # EMA smoothing
    alpha           = 0.4
    cocon_smooth    = (1 - alpha) * cocon_smooth + alpha * cocon_raw

    # Normalized cocontraction
    if COCON_MAX > COCON_MIN:
        cocon_norm = max(0.0, (cocon_smooth - COCON_MIN) / (COCON_MAX - COCON_MIN))
    else:
        cocon_norm = 0.0

    vel = velocity_from_cocon(cocon_norm, a_value)

    # Serial transmission
    if SERIAL_COM:
        try:
            ser.write(b"C")
            ser.write(f"{vel:.6f}\n".encode())
        except Exception as e:
            print("Errore seriale:", e)


    # # Aggiorna buffer e plot
    # cocon_buf[:-1]  = cocon_buf[1:]
    # cocon_buf[-1]   = cocon_norm
    # curve.setData(cocon_buf)

    # # Print velocity occasionally
    # now = time.time()
    # if now - last_print >= PRINT_EVERY:
    #     print(f"[INFO] cocon={cocon_norm:.3f}, vel={vel:.3f}")
    #     last_print = now


# ======= Streaming per valutare un candidato 'a' =======
def stream_candidate(a_value: float, max_time_s: float = 30.0, update_hz: float = 20.0):
    """
    Stream cocontraction & velocity in real-time:
    - cocontraction is plotted continuously
    - velocity is printed every PRINT_EVERY seconds
    - stops when user presses Enter or after max_time_s
    """
    print(f"\n=== Test parameter a = {a_value:.3f} ===")
    print("Contract muscles to feel the real-time response.")

    # Show the mapping function for this a_value
    plot_mapping_function(a_value)

    waiter = EnterWaiter()
    waiter.start()

    # Create a QTimer that calls update() at fixed rate
    timer = QtCore.QTimer()
    timer.timeout.connect(lambda: update(a_value))
    timer.start(int(1000 / update_hz))  # e.g. 20 Hz

    # Stop timer and quit Qt loop after timeout or Enter
    def stop_stream():
        if timer.isActive():
            timer.stop()
        app.quit()

    # Kill after max_time_s
    # QtCore.QTimer.singleShot(int(max_time_s * 1000), stop_stream)

    # Also stop if Enter is pressed (check in background)
    check_timer = QtCore.QTimer()
    check_timer.timeout.connect(lambda: stop_stream() if waiter.is_set() else None)
    check_timer.start(200)  # check every 200 ms

    # Run the Qt event loop until quit
    app.exec_()

    print("⏹️  Trial ended.")


# ======= Scelta utente tra due candidati =======
def get_user_choice():
    while True:
        c = input("Choose: [1] first, [2] second, [r] repeat, [q] quit: ").strip().lower()
        if c in ('1', '2', 'r', 'q'):
            return c
        print("Input is not valid.")

# ======= Selezione del prossimo candidato con GP (UCB penalizzato) =======
def select_next_candidate(gp, X_train, previous_best, min_distance=MIN_DISTANCE):
    samples = np.linspace(RANGE_A[0], RANGE_A[1], N_SAMPLES).reshape(-1, 1)
    tested_vals = np.array(X_train).flatten() if len(X_train) else np.array([])

    def far_enough(val, tested, d=min_distance):
        return not len(tested) or np.all(np.abs(tested - val) >= d)

    filtered = np.array([s for s in samples if far_enough(s[0], tested_vals)])
    if len(filtered) < 10:
        filtered = samples

    mu, sigma = gp.predict(filtered, return_std=True)
    beta = 1.5
    lam  = 0.3
    ucb  = mu + beta * sigma
    if previous_best is not None:
        distances = np.abs(filtered.flatten() - previous_best)
        ucb -= lam * distances

    return float(filtered[np.argmax(ucb)][0])

# ======= Main loop: preference-based optimization su 'a' =======
def run_pbo_on_a():
    print("\n--- Preference-based optimization on parameter 'a' (0..1) ---")
    X_train = []   # [[a], ...]
    y_train = []   # preferenze binarie (1 vince, 0 perde)
    previous_best = None

    for it in range(NUM_ITERATIONS):
        print(f"\nIteration {it+1}/{NUM_ITERATIONS}")

        if it == 0:
            a1 = 0.20
            a2 = 0.80
        else:
            gp.fit(np.array(X_train), np.array(y_train))
            a2 = select_next_candidate(gp, X_train, previous_best)
            a1 = previous_best

        # --- Valutazione interattiva del PRIMO candidato ---
        print(f"\nTrial [1] a = {a1:.3f}")
        stream_candidate(a1)

        # --- Valutazione interattiva del SECONDO candidato ---
        print(f"\nTrial [2] a = {a2:.3f}")
        stream_candidate(a2)

        # --- Scelta utente ---
        while True:
            ch = get_user_choice()
            if ch == '1':
                X_train.append([a1]); y_train.append(1)
                X_train.append([a2]); y_train.append(0)
                previous_best = a1
                break
            elif ch == '2':
                X_train.append([a1]); y_train.append(0)
                X_train.append([a2]); y_train.append(1)
                previous_best = a2
                break
            elif ch == 'r':
                print("Repeating the iteration...")
                # ripeti streaming degli stessi due candidati
                print(f"\nTrial [1] a = {a1:.3f}")
                stream_candidate(a1)
                print(f"\nTrial [2] a = {a2:.3f}")
                stream_candidate( a2)
            elif ch == 'q':
                print("Stopped by user.")
                return previous_best

    print(f"\Best 'a' found: {previous_best:.3f}")
    return previous_best

# ======= Avvio =======

best_a = run_pbo_on_a()
# ---- Save to CSV ----
with open("a_value.csv", mode='w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow([ f"{best_a:.6f}" if best_a is not None else ""])

print("\nCalibration completed. Results saved to 'a_value.csv'.")

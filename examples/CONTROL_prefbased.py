import sys
import os
import csv
import datetime
import serial
import glob
import argparse
import numpy as np
import pyqtgraph as pg
from SerialCom import SerialCommunication
from pathlib import Path
from scipy.signal import butter, iirnotch, filtfilt, lfilter, lfilter_zi
from pyqtgraph.Qt import QtCore, QtWidgets
from mindrove.board_shim import BoardShim, MindRoveInputParams, BoardIds
from NMF import compute_synergy_metrics, Perform_NMF, trim_percent
from PREF_BASED_OPT_cocontraction import velocity_from_cocon, TX_HEADER
from serial.tools import list_ports

baud_rate   = 115200
INVERSION   = True
SERIAL_COM  = True
FUZZY       = False
FLEXEXT     = False  # disabilita flex/ext, solo cocontrazione

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

# ──── Costanti fuzzy ──────────────────────────
DIFF_DEADBAND        = 0.10
DIFF_DEADBAND_WIDTH  = 0.15
CO_MIN               = 0.15
CO_GATE_WIDTH        = 0.20
DIFF_MAX_FOR_SRL     = 0.15
DIFF_SRL_WIDTH       = 0.15

G_SMALL              = 0.15
G_MED                = 0.50
G_LARGE              = 0.75


# ────── fuzzy helpers ──────────────────────────

def tri(x, a, b, c):
    if x <= a or x >= c:
        return 0.0
    if x == b:
        return 1.0
    return (x - a) / (b - a) if x < b else (c - x) / (c - b)

def grade(x, a, b):
    if x <= a:
        return 0.0
    if x >= b:
        return 1.0
    return (x - a) / (b - a)

def rgrade(x, a, b):
    if x <= a:
        return 1.0
    if x >= b:
        return 0.0
    return (b - x) / (b - a)

# ───── Fuzzy controller ─────────────────────────────────────

def fuzzy_mapping(flex, ext, cocontraction, best_grip, best_srl):
    d   = ext - flex             # -1..1
    a   = abs(d)                 # |d|
    c   = cocontraction          # 0..1
    d01 = 0.5 * (d + 1.0)

    # Membership su d
    mu_neg  = tri(d01, 0.00, 0.00, 0.50)
    mu_zero = tri(d01, 0.25, 0.50, 0.75)
    mu_pos  = tri(d01, 0.50, 1.00, 1.00)

    # Membership su |d|
    mu_a_small = rgrade(a, DIFF_MAX_FOR_SRL, DIFF_MAX_FOR_SRL + DIFF_SRL_WIDTH)

    # Membership su co-contraction
    mu_c_low  = rgrade(c, CO_MIN, CO_MIN + CO_GATE_WIDTH)
    mu_c_med  = tri(c, CO_MIN, CO_MIN + CO_GATE_WIDTH, CO_MIN + 2*CO_GATE_WIDTH)
    mu_c_high = grade(c, CO_MIN + CO_GATE_WIDTH, CO_MIN + 2*CO_GATE_WIDTH)

    # Consequents SRL
    srl_slow = (best_srl - 0.2) 
    srl_med  = best_srl 
    srl_fast = (best_srl + 0.2) 

    # Membership velocità gripper
    mu_d_small = rgrade(a, G_SMALL, G_MED)
    mu_d_med   = tri(a, G_SMALL, G_MED, G_LARGE)
    mu_d_large = grade(a, G_MED, G_LARGE)

    # Consequents gripper
    g_slow = (best_grip - 0.2) 
    g_med  = best_grip 
    g_fast = (best_grip + 0.2) 

    # Magnitudo pesata gripper
    Wmag = mu_d_small + mu_d_med + mu_d_large or 1.0
    g_mag = (mu_d_small*g_slow + mu_d_med*g_med + mu_d_large*g_fast) / Wmag

    # Direzione
    Wdir = mu_pos + mu_neg or 1.0
    dir_sign = (mu_pos - mu_neg) / Wdir

    # Comando gripper + deadband
    gripper_cmd = dir_sign * g_mag
    g_gate = grade(a, DIFF_DEADBAND, DIFF_DEADBAND + DIFF_DEADBAND_WIDTH)
    g_cmd = gripper_cmd * g_gate

    # SRL fuzzy aggregation
    w_srl_slow = min(mu_a_small, mu_c_low)
    w_srl_med  = min(mu_a_small, mu_c_med)
    w_srl_fast = min(mu_a_small, mu_c_high)
    Wsrl = w_srl_slow + w_srl_med + w_srl_fast or 1.0
    srl_cmd_mag = (w_srl_slow*srl_slow + w_srl_med*srl_med + w_srl_fast*srl_fast) / Wsrl

    # Gate SRL
    co_gate       = grade(c, CO_MIN, CO_MIN + CO_GATE_WIDTH)
    diff_gate_srl = rgrade(a, DIFF_MAX_FOR_SRL, DIFF_MAX_FOR_SRL + DIFF_SRL_WIDTH)
    srl_gate      = min(co_gate, diff_gate_srl)

    g_cmd = np.clip(g_cmd, -1, 1)
    srl_cmd = np.clip(srl_cmd_mag * srl_gate, 0, 1)

    # ---- output ----
    return g_cmd, srl_cmd

# ───── Build pyqtgraph window ──────────────────────────────
pg.setConfigOption('background', 'w')
pg.setConfigOption('foreground', 'k')
win   = pg.GraphicsLayoutWidget(title="Live Cocontraction", show=True)
p     = win.addPlot(row=0, col=0, title="Cocontraction (normalized)")
curve = p.plot(pen=pg.mkPen(color=(0, 150, 0), width=2))
p.showGrid(x=True, y=True, alpha=0.3)

# ───── Carica dati calibrazione cocontrazione ──────────────
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

# ───── Compute NMF offline ─────────────────────────────────
def load_calibration_values(csv_path="calibration_results.csv"):
    """
    Supports both formats:
    A) rows of (variable,value)
       variable,value
       gripper_velocity,0.42
       srl_velocity,0.77

    B) single row
       gripper_velocity,srl_velocity
       0.42,0.77
    """
    p = Path(csv_path)
    if not p.exists():
        print(f"[WARN] Calibration file not found: {csv_path}. Using defaults 0.0, 0.0")
        return 0.0, 0.0

    with p.open("r", newline="") as f:
        reader = csv.reader(f)
        rows = [r for r in reader if r]

    # Empty or header only
    if len(rows) < 2:
        print(f"[WARN] Calibration file seems empty: {csv_path}. Using defaults 0.0, 0.0")
        return 0.0, 0.0

    # Detect format
    header = [h.strip().lower() for h in rows[0]]

    # Format B: single row with two columns
    if ("gripper_velocity" in header) and ("srl_velocity" in header) and len(rows) >= 2:
        try:
            idx_g   = header.index("gripper_velocity")
            idx_s   = header.index("srl_velocity")
            vals    = rows[1]
            best_g  = float(vals[idx_g])
            best_s  = float(vals[idx_s])
            return best_g, best_s
        except Exception as e:
            print(f"[WARN] Failed to parse single-row calibration CSV: {e}. Using defaults.")
            return 0.0, 0.0

    # Format A: variable,value pairs
    try:
        best_g, best_s = 0.0, 0.0
        for r in rows[1:]:
            if len(r) < 2:
                continue
            var = r[0].strip().lower()
            try:
                val = float(r[1])
            except:
                continue
            if var in ("gripper_velocity", "gripper", "best_gripper"):
                best_g = val
            elif var in ("srl_velocity", "srl", "best_srl"):
                best_s = val
        return best_g, best_s
    except Exception as e:
        print(f"[WARN] Failed to parse calibration CSV: {e}. Using defaults.")
        return 0.0, 0.0    
    
BEST_GRIPPER, BEST_SRL = load_calibration_values("calibration_results.csv")

def load_data(path):
    ext = os.path.splitext(path)[1].lower()
    if ext == '.npy':
        return np.load(path)
    elif ext in ('.csv', '.txt'):
        # prova a leggere come singolo valore
        with open(path, "r") as f:
            content = f.read().strip()
        try:
            return float(content)
        except ValueError:
            # fallback: usa loadtxt se è una tabella
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

a_value             = load_data("a_value.csv")
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

# ───── Update callback ─────────────────────────────────────
RMS_WINDOW_S  = 0.3
rms_win_samps = int(RMS_WINDOW_S * sr)
if rms_win_samps < 1:
    rms_win_samps = 1
rms_kernel    = np.ones(rms_win_samps) / rms_win_samps

cocon_smooth = 0.0

def update():
    global cocon_buf, cocon_smooth

    count = board.get_board_data_count()
    if count <= 0:
        return

    data_raw = board.get_board_data(N)

    if data_raw.shape[1] < 20:
        return

    # Filtra EMG
    data_filt = np.zeros_like(data_raw)
    for ch in emg_ch:
        tmp = filtfilt(b_notch, a_notch, data_raw[ch])
        data_filt[ch] = filtfilt(b_hp, a_hp, tmp)
        # data_filt[ch], zi_hp = lfilter(b_hp, a_hp, data_raw[ch], zi=zi_hp)

    # RMS
    rect = np.abs(data_filt)
    rms = np.array([
        np.sqrt(np.convolve(rect[ch]**2, rms_kernel, mode='same'))
        for ch in emg_ch
    ])
    x_new = rms[:, -1]

    # Attivazioni NMF
    h_new = x_new.dot(W_pinv)
    flex_norm = max(0.0, (h_new[1] - k_flex_min) / (k_flex_max - k_flex_min))
    ext_norm  = max(0.0, (h_new[0] - k_ext_min)  / (k_ext_max  - k_ext_min))

    # Cocontrazione grezza
    cocon_raw = min(flex_norm, ext_norm)

    # EMA smoothing
    alpha = 0.4
    cocon_smooth = (1 - alpha) * cocon_smooth + alpha * cocon_raw

    # Normalizzazione con valori da calibrazione
    if COCON_MAX > COCON_MIN:
        cocon_norm = (cocon_smooth - COCON_MIN) / (COCON_MAX - COCON_MIN)
        # cocon_norm = np.clip(cocon_norm, 0.0, 1.0)
        cocon_norm = max(cocon_norm, 0.0)
    else:
        cocon_norm = 0.0

    cocon1 = velocity_from_cocon(cocon_norm, a_value)
    cocon2 = 0.0    # riservato per futuro uso 

    if not FLEXEXT:
        flex_norm = 0.0
        ext_norm  = 0.0

    # ───── Fuzzy control ──────────────────────────
    if FUZZY:
        flexext_cmd, cocon1_mag = fuzzy_mapping(flex_norm, ext_norm, cocon_norm, BEST_GRIPPER, BEST_SRL)
        command = f"{flexext_cmd:.6f} {cocon1_mag:.6f} {cocon2:.6f}\n"
    else:
        command = f"{flex_norm-ext_norm:.6f} {cocon1:.6f} {cocon2:.6f}\n"

    # Serial transmission
    if SERIAL_COM:
        try:
            ser.write(TX_HEADER)
            ser.write(command.encode())
        except Exception as e:
            print("Errore seriale:", e)

    # Aggiorna buffer e plot
    cocon_buf[:-1] = cocon_buf[1:]
    cocon_buf[-1]  = cocon_norm
    curve.setData(cocon_buf)
    p.enableAutoRange(axis='y', enable=True)

# timer
timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(60)

# ───── Start GUI loop ──────────────────────────────────────
if __name__ == '__main__':
    sys.exit(app.exec_())

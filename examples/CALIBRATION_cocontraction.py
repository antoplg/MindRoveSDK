import sys
import os
import glob
import argparse
import numpy as np
import pyqtgraph as pg
from pathlib import Path
from scipy.signal import butter, iirnotch, filtfilt
from pyqtgraph.Qt import QtCore, QtWidgets
from mindrove.board_shim import BoardShim, MindRoveInputParams, BoardIds
from NMF import compute_synergy_metrics, Perform_NMF, trim_percent
import datetime
import csv

INVERSION   = True

# ───── 1) Qt App ───────────────────────────────────────────────
app = QtWidgets.QApplication(sys.argv)

# ───── 2) Start MindRove streaming ─────────────────────────────
BoardShim.enable_dev_board_logger()
params   = MindRoveInputParams()
board_id = BoardIds.MINDROVE_WIFI_BOARD
board    = BoardShim(board_id, params)
board.prepare_session()
board.start_stream()

# ───── 3) Prepare channels & buffers ──────────────────────────
emg_ch   = BoardShim.get_emg_channels(board_id)
sr       = BoardShim.get_sampling_rate(board_id)
window_s = 10
N        = int(window_s * sr)

# DESIGN FILTRI EMG
b_notch, a_notch = iirnotch(50.0/(sr/2), Q=30.0)
b_hp, a_hp       = butter(4, 20.0/(sr/2), btype='highpass')

# Buffer circolare per cocontrazione
buf_len    = int(1 * sr)  # 1 secondo di buffer
cocon_buf  = np.zeros(buf_len)

# ───── 4) Build pyqtgraph window ──────────────────────────────
pg.setConfigOption('background', 'w')
pg.setConfigOption('foreground', 'k')
win        = pg.GraphicsLayoutWidget(title="Cocontraction", show=True)
p          = win.addPlot(row=0, col=0, title="Cocontraction (min flex, ext)")
curve      = p.plot(pen=pg.mkPen(color=(0, 100, 0), width=2))
p.showGrid(x=True, y=True, alpha=0.3)

# ───── 5) Load data for calibration ───────────────────────────
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
parser.add_argument('file', nargs='?', help="Path a emg_session.npy/.csv")
parser.add_argument('--fs', type=float, default=250, help="Frequenza di campionamento (Hz)")
parser.add_argument('--rms-win', type=float, default=0.2, help="Finestra RMS (s)")
args = parser.parse_args()

fp = args.file or find_default_file()
if not fp:
    print("Nessun file trovato emg_session.*", file=sys.stderr)
    sys.exit(1)

data_offline     = load_data(fp)
data_offline_abs = np.abs(data_offline)
data_offline_rms = np.zeros_like(data_offline_abs)

win_samps = int(args.rms_win * args.fs)
if win_samps < 1:
    win_samps = 1
kernel = np.ones(win_samps) / win_samps

for ch in range(data_offline.shape[1]):
    sq   = data_offline_abs[:, ch] ** 2
    mov  = np.convolve(sq, kernel, mode='same')
    data_offline_rms[:, ch] = np.sqrt(mov)

H, W = Perform_NMF(data_offline_rms)
k_ext_max, k_flex_max, k_ext_min, k_flex_min = compute_synergy_metrics(H, trim_percent)
W_pinv = np.linalg.pinv(W)

# ───── 6) File CSV per salvare cocontrazione ──────────────────
log_file  = "cocontraction_session.csv"

with open(log_file, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["cocontraction"])   # solo una colonna

print(f"[INFO] Logging cocontrazione su {log_file}")

# ───── 7) Update callback ─────────────────────────────────────
RMS_WINDOW_S  = 0.3                 # lunghezza finestra RMS (s)
rms_win_samps = int(RMS_WINDOW_S * sr)
if rms_win_samps < 1:
    rms_win_samps = 1
rms_kernel    = np.ones(rms_win_samps) / rms_win_samps   # DEFINIZIONE QUI

def update():
    count = board.get_board_data_count()
    if count <= 0:
        return

    data_raw = board.get_board_data(N)

    # --- Filtra EMG ---
    data_filt = np.zeros_like(data_raw)
    for ch in emg_ch:
        tmp = filtfilt(b_notch, a_notch, data_raw[ch])
        data_filt[ch] = filtfilt(b_hp, a_hp, tmp)

    # --- RMS per canale ---
    rect = np.abs(data_filt)
    rms = np.array([
        np.sqrt(np.convolve(rect[ch]**2, rms_kernel, mode='same'))
        for ch in emg_ch
    ])
    x_new = rms[:, -1]

    # --- Attivazioni NMF ---
    h_new = x_new.dot(W_pinv)

    flex_norm = max(0.0, (h_new[1] - k_flex_min) / (k_flex_max - k_flex_min))
    ext_norm  = max(0.0, (h_new[0] - k_ext_min)  / (k_ext_max  - k_ext_min))

    # --- Cocontrazione semplice ---
    cocon_raw = min(flex_norm, ext_norm)

    # --- EMA leggero per stabilizzare ---
    global cocon_smooth
    try:
        cocon_smooth
    except NameError:
        cocon_smooth = 0.0

    alpha = 0.25 # più alto = più reattivo, meno liscio
    cocon_smooth = (1 - alpha) * cocon_smooth + alpha * cocon_raw
    cocon = cocon_smooth

    # --- Aggiorna buffer ---
    global cocon_buf
    cocon_buf[:-1] = cocon_buf[1:]
    cocon_buf[-1]  = cocon

    # --- Plot ---
    curve.setData(cocon_buf)
    p.enableAutoRange(axis='y', enable=True)

    # --- Salva solo cocontrazione ---
    with open(log_file, "a", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([cocon])
        
print("\nCalibration completed. Results saved to 'cocontraction_results.csv'.")

# timer
timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(40)

# ───── 8) Start GUI loop ──────────────────────────────────────
if __name__ == '__main__':
    sys.exit(app.exec_())

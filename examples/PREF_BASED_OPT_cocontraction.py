import csv
import sys
import os
import time
import glob
import struct
import threading
import argparse
import numpy as np
from scipy.stats import norm
from serial.tools import list_ports
from pyqtgraph.Qt import QtCore, QtWidgets
from scipy.signal import butter, iirnotch, filtfilt
from mindrove.board_shim import BoardShim, MindRoveInputParams, BoardIds
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF, ConstantKernel as C
from NMF import compute_synergy_metrics, Perform_NMF, trim_percent
from CALIBRATION_cocontraction import load_data, find_default_file
from CONTROL import compute_trimmed_minmax, load_calibration_cocon, update
import serial

# ======= Parametri generali =======
BAUD_RATE       = 115200
SERIAL_TIMEOUT  = 0.01     # s, lettura non bloccante
TX_HEADER       = b'V'     # header per inviare velocità (float32)
PRINT_EVERY     = 0.2      # s, quanto spesso stampare feedback a video

NUM_ITERATIONS  = 10
RANGE_A         = (0.0, 1.0)
MIN_DISTANCE    = 0.15     # distanza minima tra campioni di 'a'
N_SAMPLES       = 1000     # candidati per la policy di acquisizione

# GP (preferenze binarie → y=1 vince, y=0 perde; qui usiamo surrogate regression)
kernel = C(1.0, (1e-2, 1e2)) * RBF(length_scale=0.4, length_scale_bounds=(1e-2, 1e1))
gp     = GaussianProcessRegressor(kernel=kernel, alpha=1e-2, normalize_y=True)
COCON_MIN, COCON_MAX = load_calibration_cocon("cocontraction_session.csv", percent=trim_percent)

# ======= Carica dati da file =======

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

# ======= Avvio =======

best_a = run_pbo_on_a()
# ---- Save to CSV ----
with open("a_value.csv", mode='w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow([ f"{best_a:.6f}" if best_a is not None else ""])

print("\nCalibration completed. Results saved to 'a_value.csv'.")

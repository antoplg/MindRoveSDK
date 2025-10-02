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
from scipy.signal import butter, iirnotch, filtfilt
from pyqtgraph.Qt import QtCore, QtWidgets
from mindrove.board_shim import BoardShim, MindRoveInputParams, BoardIds
from NMF import compute_synergy_metrics, Perform_NMF, trim_percent
from serial.tools import list_ports

INVERSION   = True
SERIAL_COM  = True  # Set to True if you want to use serial communication
baud_rate   = 115200

if SERIAL_COM:
    port_list = list(list_ports.comports(include_links=False))
    port_arduino = port_list[0].device
    print(port_arduino)  
    ser = serial.Serial(port_arduino, baud_rate)


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
emg_ch      = BoardShim.get_emg_channels(board_id)
acc_ch      = BoardShim.get_accel_channels(board_id)
gyro_ch     = BoardShim.get_gyro_channels(board_id)
imu_ch      = acc_ch + gyro_ch
sr          = BoardShim.get_sampling_rate(board_id)
window_s    = 10
N           = int(window_s * sr)
window_s2   = 1
N2          = int(window_s2 * sr)

# DESIGN FILTRI EMG
b_notch, a_notch = iirnotch(50.0/(sr/2), Q=30.0)
b_hp, a_hp       = butter(4, 20.0/(sr/2), btype='highpass')

# Buffer circolari
emg_bufs = [np.zeros(N2) for _ in range(2)] 
imu_bufs = [np.zeros(N) for _ in imu_ch]

session_emg = []
session_imu = []

# ───── 4) Build TWO pyqtgraph windows ────────────────────────
# EMG window
pg.setConfigOption('background', 'w')  # sfondo bianco
pg.setConfigOption('foreground', 'k')  # testo e assi neri
win         = pg.GraphicsLayoutWidget(title="Live Synergy Activations", show=True)
emg_plots   = []
emg_curves  = []
directions  = ["Flexion", "Extension"]

for i in range(2):
    p = win.addPlot(row=i, col=0, title=f"{directions[i]}")
    # colore più scuro e linea più spessa
    dark_color = (30, 30, 180) if i == 0 else (180, 30, 30)  
    c = p.plot(pen=pg.mkPen(color=dark_color, width=2))  
    p.setAutoVisible(y=True)
    p.showGrid(x=True, y=True, alpha=0.3)
    emg_plots.append(p)
    emg_curves.append(c)

# IMU window
# win_imu = pg.GraphicsLayoutWidget(show=True, title="Live IMU")
# imu_plots = [] 
# imu_curves = []
# for i, ch in enumerate(acc_ch):
#     p = win_imu.addPlot(row=i, col=0, title=f"Accel Ch {ch}")
#     c = p.plot(pen=pg.intColor(i, hues=len(acc_ch)))
#     p.setAutoVisible(y=True)
#     imu_plots.append(p)
#     imu_curves.append(c)
# for i, ch in enumerate(gyro_ch):
#     p = win_imu.addPlot(row=len(acc_ch)+i, col=0, title=f"Gyro Ch {ch}")
#     c = p.plot(pen=pg.intColor(i, hues=len(gyro_ch)))
#     p.setAutoVisible(y=True)
#     imu_plots.append(p)
#     imu_curves.append(c)

# ───── 5) Compute NMF and metrics ─────────────────────────────

def load_data(path):
    ext = os.path.splitext(path)[1].lower()
    if ext == '.npy':
        return np.load(path)         # (samples, channels)
    elif ext in ('.csv', '.txt'):
        return np.loadtxt(path,
                           delimiter=',',
                           skiprows=1)  # assume header
    else:
        raise ValueError(f"Unsupported extension {ext}")
    

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


def find_default_file():
    for ext in ('npy','csv','txt'):
        matches = glob.glob(f"emg_session*.{ext}")
        if matches:
            return matches[0]
    return None

# Load calibration (best) values once at startup
BEST_GRIPPER, BEST_SRL = load_calibration_values("calibration_results.csv")
print(f"[INFO] Loaded calibration: best_gripper={BEST_GRIPPER:.6f}, best_srl={BEST_SRL:.6f}")

parser = argparse.ArgumentParser(description="EMG → NMF (flexion/extension) senza filtri")
parser.add_argument('file', nargs='?',
                    help="Path a emg_session.npy/.csv (default: primo emg_session.*)")
parser.add_argument('--fs', type=float, default=250,
                    help="Frequenza di campionamento (Hz), default=250")
parser.add_argument('--rms-win', type=float, default=0.2,
                    help="Lunghezza finestra RMS (s), default 0.2")
args = parser.parse_args()

fp = args.file or find_default_file()
if not fp:
    print("Nessun file trovato emg_session.*", file=sys.stderr)
    sys.exit(1)

data_offline        = load_data(fp)              # shape = (ns, nc)
ns, nc              = data_offline.shape
data_offline_abs    = np.abs(data_offline)
data_offline_rms    = np.zeros_like(data_offline_abs)

# Calcola RMS sulla finestra di lunghezza args.rms_win
win_samps       = int(args.rms_win * args.fs)
if win_samps < 1:
    win_samps   = 1
kernel          = np.ones(win_samps) / win_samps

# Per ciascuna colonna (canale) facciamo la media mobile dei quadrati
for ch in range(nc):
    sq                      = data_offline_abs[:, ch] ** 2
    movavg                  = np.convolve(sq, kernel, mode='same')
    data_offline_rms[:, ch] = np.sqrt(movavg)

# Esegui NMF
H, W    = Perform_NMF(data_offline_rms)

# Scaling dei valori di H per le metriche
k_ext_max, k_flex_max, k_ext_min, k_flex_min = compute_synergy_metrics(H, trim_percent)

# costruisci la pseudo‐inversa di W (nc×2)
W_pinv = np.linalg.pinv(W)

# ───── 5) Update callback ─────────────────────────────────────
RMS_WINDOW_S        = 0.3             # finestra RMS in secondi (es. 100 ms)
rms_win_samps       = int(RMS_WINDOW_S * sr)
if rms_win_samps < 1:
    rms_win_samps   = 1
rms_kernel          = np.ones(rms_win_samps) / rms_win_samps

def update():

    count = board.get_board_data_count()
    if count <= 0:
        return

    data_raw = board.get_board_data(N)  # (n_ch, N)

    # FILTRA EMG
    data_filt = np.zeros_like(data_raw)
    for ch in emg_ch:
        tmp = filtfilt(b_notch, a_notch, data_raw[ch])
        data_filt[ch] = filtfilt(b_hp, a_hp, tmp)

    # RMS
    # rettifica + RMS su ciascun canale
    rect = np.abs(data_filt)
    # costruisci un array (n_ch, N) di RMS usando list comprehension
    rms = np.array([
        np.sqrt(np.convolve(rect[ch]**2, rms_kernel, mode='same'))
        for ch in emg_ch
    ])
    x_new = rms[:, -1]                 # vettore shape (n_ch,)

    # ACTIVATION
    h_new = x_new.dot(W_pinv)          # vettore shape (2,)

    # NORMALIZE
    flex_norm   = (h_new[1] - k_flex_min) / (k_flex_max - k_flex_min)
    if flex_norm < 0:
        flex_norm = 0.0
    ext_norm    = (h_new[0] - k_ext_min)  / (k_ext_max  - k_ext_min)
    if ext_norm < 0:
        ext_norm = 0.0

    if SERIAL_COM:
        try:
            # Build: C best_gripper best_srl flex ext\n
            # (User requested order: flex then ext)
            if INVERSION:
                command = f"{BEST_SRL:.6f} {BEST_GRIPPER:.6f} {ext_norm:.6f} {flex_norm:.6f}\n"
            else:
                command = f"{BEST_SRL:.6f} {BEST_GRIPPER:.6f} {flex_norm:.6f} {ext_norm:.6f}\n"
            target = 'V'

            ser.write(target.encode())  # send header
            ser.write(command.encode())
            # print(buf)
        except Exception as e:
            print("Errore seriale:", e)

    # UPDATE EMG
    for buf, val in zip(emg_bufs, (ext_norm, flex_norm)):
        buf[:-1] = buf[1:]
        buf[-1]  = val

    for curve, buf in zip(emg_curves, emg_bufs):
        curve.setData(buf)

    for p in emg_plots:
        p.enableAutoRange(axis='y', enable=True)

    # UPDATE IMU
    # for buf, curve, ch, plot in zip(imu_bufs, imu_curves, imu_ch, imu_plots):
    #     seg = data_raw[ch]
    #     if len(seg) < N:
    #         buf[:-len(seg)] = buf[len(seg):]
    #         buf[-len(seg):] = seg
    #     else:
    #         buf[:] = seg[-N:]
    #     curve.setData(buf)
    #     plot.enableAutoRange(axis='y', enable=True)

    if SERIAL_COM and ser.in_waiting > 0:
        try:
            data = ser.readline().decode("utf-8", errors="ignore").strip()
            if data:
                print("RX:", data)
        except Exception as e:
            print("Errore lettura seriale:", e)

# timer
timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(40)

# ───── 6) Print Serial ────────────────────────────────────────

class SerialData2Csv:
    """
    Class for writing serial data to csv file.
    """

    def __init__(self, file_name, baudrate=115200):
        # get the port name automatically
        port_list = list(list_ports.comports(include_links=False))
        port_arduino = port_list[0].device
        print(port_arduino)  

        # Establish serial communication with Arduino
        self.ser_com = SerialCommunication(port_arduino, baudrate)

        # Data frame for saving the data
        self.file_headers = ["datetime", "timestamp", "flexion", "extension"]
        self.file_name = file_name

        # Initialize csv file with header
        with open(self.file_name, mode='w') as file:
            # Create a csv writer object
            writer = csv.writer(file, dialect="excel")
            writer.writerow(self.file_headers)

            file.close()

    # Extract pressure sensor readings and desired pressure from data string
    def edit_data(self, data):
        # Split the data into the single parts
        split_data = data.split(",")

        # Define datetime and timestamp
        ct = datetime.datetime.now()
        ts = ct.timestamp()

        # Split data string into translation and rotation data of base and end-effector
        flexion = float(split_data[0][1:])
        extension = float(split_data[1])

        return ct, ts, flexion, extension
    
    def run(self):
        # Run serial communication
        try:
            while True:
                # Get data from serial
                data = self.ser_com.receive_data_from_arduino()
                ct, ts, flexion, extension = self.edit_data(data)
                print(f"Data received: {ct}, {ts}, {flexion}, {extension}")

        except KeyboardInterrupt:
            self.ser_com.ser.close()

# ───── 6) Start GUI loop ─────────────────────────────────────
if __name__ == '__main__':
    # data_writer = SerialData2Csv(file_name="date_name_raw.csv", baudrate=115200)
    # data_writer.run()
    sys.exit(app.exec_())
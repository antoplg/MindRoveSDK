import sys
import os
import struct
import datetime
import glob
import argparse
import numpy as np
import pyqtgraph as pg
from SerialCom import SerialCommunication
from scipy.signal import butter, iirnotch, filtfilt
from pyqtgraph.Qt import QtCore, QtWidgets
from mindrove.board_shim import BoardShim, MindRoveInputParams, BoardIds
from NMF import compute_synergy_metrics, Perform_NMF, trim_percent
from serial.tools import list_ports
import serial

baud_rate=115200
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
win = pg.GraphicsLayoutWidget(title="Live Synergy Activations", show=True)
emg_plots = []
emg_curves = []
directions = ["Flexion", "Extension"]

for i in range(2):
    p = win.addPlot(row=i, col=0, title=f"{directions[i]}")
    c = p.plot(pen=pg.intColor(i, hues=2))
    p.setAutoVisible(y=True)
    emg_plots.append(p)
    emg_curves.append(c)

# IMU window
win_imu = pg.GraphicsLayoutWidget(show=True, title="Live IMU")
imu_plots = [] 
imu_curves = []
for i, ch in enumerate(acc_ch):
    p = win_imu.addPlot(row=i, col=0, title=f"Accel Ch {ch}")
    c = p.plot(pen=pg.intColor(i, hues=len(acc_ch)))
    p.setAutoVisible(y=True)
    imu_plots.append(p)
    imu_curves.append(c)
for i, ch in enumerate(gyro_ch):
    p = win_imu.addPlot(row=len(acc_ch)+i, col=0, title=f"Gyro Ch {ch}")
    c = p.plot(pen=pg.intColor(i, hues=len(gyro_ch)))
    p.setAutoVisible(y=True)
    imu_plots.append(p)
    imu_curves.append(c)

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


def find_default_file():
    for ext in ('npy','csv','txt'):
        matches = glob.glob(f"emg_session*.{ext}")
        if matches:
            return matches[0]
    return None

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
RMS_WINDOW_S = 0.2               # finestra RMS in secondi (es. 200 ms)
rms_win_samps = int(RMS_WINDOW_S * sr)
if rms_win_samps < 1:
    rms_win_samps = 1
rms_kernel = np.ones(rms_win_samps) / rms_win_samps

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
    flex_norm = round((h_new[0] - k_flex_min) / (k_flex_max - k_flex_min), 2)
    ext_norm  = round((h_new[1] - k_ext_min)  / (k_ext_max  - k_ext_min), 2)

    cocontraction = min(flex_norm, ext_norm)

    try:
        command = str(flex_norm) + ' ' + str(ext_norm) + ' \r\n'
        ser.reset_output_buffer()
        ser.write(command.encode())
        # print(buf)
    except Exception as e:
        print("Errore seriale:", e)


    # UPDATE EMG
    for buf, val in zip(emg_bufs, (flex_norm, ext_norm)):
        buf[:-1] = buf[1:]
        buf[-1]  = val

    for curve, buf in zip(emg_curves, emg_bufs):
        curve.setData(buf)

    for p in emg_plots:
        p.enableAutoRange(axis='y', enable=True)

    # UPDATE IMU
    for buf, curve, ch, plot in zip(imu_bufs, imu_curves, imu_ch, imu_plots):
        seg = data_raw[ch]
        if len(seg) < N:
            buf[:-len(seg)] = buf[len(seg):]
            buf[-len(seg):] = seg
        else:
            buf[:] = seg[-N:]
        curve.setData(buf)
        plot.enableAutoRange(axis='y', enable=True)


    ser.reset_input_buffer()
    message = ""
    while True:
        # Get data from TCP server
        data = ser.readline()
        data = data.decode("utf-8")

        message = message + data

        # Search for unique first element of message "<" (find returns -1 if element is not found)
        position_first_element = message.find("<")

        # Check if the unique first element is included in the data and cut everything before if so
        if position_first_element == -1:
            continue
        else:
            message = message[position_first_element:]

        # Search for the unique last element of message ">"
        position_last_element = message.find(">")

        # Check if the unique last element is included in the data and cut everything after if so
        if position_last_element == -1:
            continue
        else:
            message = message[:position_last_element+1]
            break
    print(message)

# timer
timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(50)

# ───── 6) Print Serial ────────────────────────────────────────

class SerialData2Csv:
    """
    Class for writing serial data to csv file.
    """

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

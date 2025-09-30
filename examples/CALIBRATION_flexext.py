#!/usr/bin/env python3
import sys
import signal
import numpy as np
import pyqtgraph as pg
from scipy.signal import butter, iirnotch, filtfilt
from pyqtgraph.Qt import QtCore, QtWidgets
from mindrove.board_shim import BoardShim, MindRoveInputParams, BoardIds

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
print(f"Sampling rate: {sr} Hz, Buffer size: {N} samples ({window_s} seconds)")

# DESIGN FILTRI EMG
b_notch, a_notch = iirnotch(50.0/(sr/2), Q=30.0)
b_hp, a_hp       = butter(4, 20.0/(sr/2), btype='highpass')

# Buffer circolari
emg_bufs = [np.zeros(N) for _ in emg_ch]
imu_bufs = [np.zeros(N) for _ in imu_ch]

session_emg = []
session_imu = []

# ───── 4) Build TWO pyqtgraph windows ────────────────────────
# EMG window
pg.setConfigOption('background', 'w')  # sfondo bianco
pg.setConfigOption('foreground', 'k')  # testo e assi neri
win_emg = pg.GraphicsLayoutWidget(show=True, title="Live EMG")
emg_plots = []; emg_curves = []
for i, ch in enumerate(emg_ch):
    p = win_emg.addPlot(row=i, col=0, title=f"EMG Ch {ch}")
    c = p.plot(pen=pg.intColor(i, hues=len(emg_ch)))
    p.setAutoVisible(y=True)
    p.showGrid(x=True, y=True, alpha=0.3)  # griglia attivata
    emg_plots.append(p)
    emg_curves.append(c)

# IMU window
win_imu = pg.GraphicsLayoutWidget(show=True, title="Live IMU")
imu_plots = []; imu_curves = []
for i, ch in enumerate(acc_ch):
    p = win_imu.addPlot(row=i, col=0, title=f"Accel Ch {ch}")
    c = p.plot(pen=pg.intColor(i, hues=len(acc_ch)))
    p.setAutoVisible(y=True)
    p.showGrid(x=True, y=True, alpha=0.3)  # griglia attivata
    imu_plots.append(p)
    imu_curves.append(c)
for i, ch in enumerate(gyro_ch):
    p = win_imu.addPlot(row=len(acc_ch)+i, col=0, title=f"Gyro Ch {ch}")
    c = p.plot(pen=pg.intColor(i, hues=len(gyro_ch)))
    p.setAutoVisible(y=True)
    p.showGrid(x=True, y=True, alpha=0.3)  # griglia attivata
    imu_plots.append(p)
    imu_curves.append(c)

# ───── 5) Update callback ─────────────────────────────────────
def update():

    count = board.get_board_data_count()
    if count <= 0:
        return
    
    data_raw = board.get_board_data(N)  # (n_ch, N)

    # FILTRA EMG
    data_emg = np.zeros_like(data_raw)
    for ch in emg_ch:
        tmp = filtfilt(b_notch, a_notch, data_raw[ch])
        data_emg[ch] = filtfilt(b_hp, a_hp, tmp)

    # ACCUMULA
    session_emg.append(data_emg[emg_ch].T)
    session_imu.append(data_raw[imu_ch].T)

    # UPDATE EMG
    for buf, curve, ch, plot in zip(emg_bufs, emg_curves, emg_ch, emg_plots):
        seg = data_emg[ch]
        if len(seg) < N:
            buf[:-len(seg)] = buf[len(seg):]
            buf[-len(seg):] = seg
        else:
            buf[:] = seg[-N:]
        curve.setData(buf)
        plot.enableAutoRange(axis='y', enable=True)

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

# timer
timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(50)

# ───── 6) Save on exit ────────────────────────────────────────
def save_and_quit(*args):
    # EMG
    if session_emg:
        all_emg = np.vstack(session_emg)
        np.save("emg_session.npy", all_emg)
        hdr = ",".join(f"emg_{ch}" for ch in emg_ch)
        np.savetxt("emg_session.csv", all_emg,
                   delimiter=",", header=hdr, comments="")
        print(f"Saved EMG {all_emg.shape[0]}×{all_emg.shape[1]}")
    else:
        print("No EMG data captured.")

    # IMU
    if session_imu:
        all_imu = np.vstack(session_imu)
        np.save("imu_session.npy", all_imu)
        hdr = [f"accel_{ax}" for ax in ('x','y','z')] + \
              [f"gyro_{ax}"  for ax in ('x','y','z')]
        np.savetxt("imu_session.csv", all_imu,
                   delimiter=",", header=",".join(hdr), comments="")
        print(f"Saved IMU {all_imu.shape[0]}×{all_imu.shape[1]}")
    else:
        print("No IMU data captured.")

    QtWidgets.QApplication.quit()

signal.signal(signal.SIGINT, save_and_quit)
app.aboutToQuit.connect(save_and_quit)

# ───── 7) Start GUI loop ─────────────────────────────────────
if __name__ == '__main__':
    sys.exit(app.exec_())

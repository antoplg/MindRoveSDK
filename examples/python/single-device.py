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

# ───── 3) Prepare EMG channels & data buffers ─────────────────

emg_ch   = BoardShim.get_emg_channels(board_id)
sr       = BoardShim.get_sampling_rate(board_id)  # es. 250 Hz
window_s = 1000
N        = int(window_s * sr)

# DESIGN DEL FILTRO:
# 1) Notch a 50 Hz
notch_freq = 50.0
notch_Q    = 30.0
b_notch, a_notch = iirnotch(notch_freq/(sr/2), notch_Q)

# 2) High-pass a 20 Hz
hp_cut = 20.0
b_hp, a_hp = butter(4, hp_cut/(sr/2), btype='highpass')

# Buffers per visualizzazione e salvataggio
disp_buffers = [np.zeros(N) for _ in emg_ch]
session_data = []  # lista di blocchi filtrati (shape=(chunk, n_ch))

# ───── 4) Build pyqtgraph window ─────────────────────────────
win    = pg.GraphicsLayoutWidget(show=True, title="Live EMG (All Channels)")
plots  = []
curves = []
for i, ch in enumerate(emg_ch):
    p     = win.addPlot(row=i, col=0, title=f"EMG Ch {ch}")
    c     = p.plot(pen=pg.intColor(i, hues=len(emg_ch)))
    p.setAutoVisible(y=True)
    plots.append(p)
    curves.append(c)

# ───── 5) Update callback ─────────────────────────────────────
def update():

    if board.get_board_data_count() >= 0:
        data_raw = board.get_current_board_data(N)

        # filtra notch → high-pass
        data_filt = np.zeros_like(data_raw)
        for ch in emg_ch:
            tmp = filtfilt(b_notch, a_notch, data_raw[ch])
            data_filt[ch] = filtfilt(b_hp, a_hp, tmp)

        # salva chunk trasposto (shape=(count, n_ch))
        session_data.append(data_filt.T)

        # aggiorna i buffer per il plot
        for buf, curve, ch, plot in zip(disp_buffers, curves, emg_ch, plots):
            seg = data_filt[ch]
            if len(seg) < N:
                buf[:-len(seg)] = buf[len(seg):]
                buf[-len(seg):] = seg
            else:
                buf[:] = seg[-N:]
            curve.setData(buf)
            plot.enableAutoRange(axis='y', enable=True)

# timer a 50 ms per chiamare update()
timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(50)

# ───── 6) Save on exit ────────────────────────────────────────
def save_and_quit(*args):
    if not session_data:
        print("\nNo data captured.")
        QtWidgets.QApplication.quit()
        return

    all_data = np.vstack(session_data)  # (tot_samples, n_ch)
    all_data = all_data[:, emg_ch]      # solo EMG

    # salva
    np.save("emg_session.npy", all_data)
    header = ",".join(f"emg_{ch}" for ch in emg_ch)
    np.savetxt("emg_session.csv", all_data,
               delimiter=",", header=header, comments="")
    print(f"\nSaved {all_data.shape[0]}×{all_data.shape[1]}")

    QtWidgets.QApplication.quit()

# cattura Ctrl+C o chiusura finestra
signal.signal(signal.SIGINT, save_and_quit)
app.aboutToQuit.connect(save_and_quit)

# ───── 7) Start GUI loop ─────────────────────────────────────
if __name__ == '__main__':
    sys.exit(app.exec_())

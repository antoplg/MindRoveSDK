#!/usr/bin/env python3
"""
plot_saved_imu.py

Load an IMU session from imu_session.npy (or CSV) and plot all channels.
If no file is specified, defaults to './imu_session.npy' (or '.csv').
"""

import os
import glob
import sys
import numpy as np
import matplotlib.pyplot as plt
import argparse

def load_data(path):
    ext = os.path.splitext(path)[1].lower()
    if ext == '.npy':
        return np.load(path)  # shape: (samples, channels)
    elif ext in ('.csv', '.txt'):
        return np.loadtxt(path, delimiter=',', skiprows=1)
    else:
        raise ValueError("Unsupported file extension: " + ext)

def find_default_file():
    for ext in ('npy','csv','txt'):
        matches = glob.glob(f"imu_session*.{ext}")
        if matches:
            return matches[0]
    return None

def main():
    parser = argparse.ArgumentParser(description="Plot saved IMU data")
    parser.add_argument('file', nargs='?',
                        help="Path to imu_session.npy or .csv (optional)")
    parser.add_argument('--fs', type=float, default=250,
                        help="Sampling rate (Hz), default=250")
    parser.add_argument('--down', type=int, default=1,
                        help="Downsample factor for plotting (default=1=no down)")
    args = parser.parse_args()

    filepath = args.file or find_default_file()
    if not filepath:
        print("Errore: nessun file fornito e non ho trovato imu_session*.npy/.csv", file=sys.stderr)
        sys.exit(1)

    data = load_data(filepath)      # shape = (samples, channels)
    ns, nc = data.shape
    t = np.arange(ns) / args.fs

    # downsampling opzionale
    if args.down > 1:
        t    = t[::args.down]
        data = data[::args.down, :]

    # nomi dei canali IMU
    channel_names = ['accel_x', 'accel_y', 'accel_z',
                     'gyro_x',  'gyro_y',  'gyro_z']
    if nc != len(channel_names):
        # in caso di numero diverso, genera etichette generiche
        channel_names = [f"Ch{i}" for i in range(nc)]

    fig, axes = plt.subplots(nc, 1, sharex=True, figsize=(10, 2*nc))
    fig.suptitle(f"IMU Session: {nc} ch, {ns} samp @ {args.fs}Hz (down×{args.down})")

    for ch in range(nc):
        ax = axes[ch]
        ax.plot(t, data[:, ch],
                linestyle='-', linewidth=0.8, marker=None)
        ax.set_ylabel(channel_names[ch], fontsize=8)
        ax.grid(True, which='both', linestyle='--', alpha=0.3)

    axes[-1].set_xlabel("Time (s)")
    plt.tight_layout(rect=[0,0,1,0.96])
    plt.show()

if __name__ == '__main__':
    main()

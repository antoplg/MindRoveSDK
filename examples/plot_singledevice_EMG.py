#!/usr/bin/env python3
"""
plot_saved_emg.py

Load an EMG session from emg_session.npy (or CSV) and plot all channels.
If no file is specified, defaults to './emg_session.npy' (or '.csv').
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
        return np.load(path)         # shape (samples, channels)
    elif ext in ('.csv', '.txt'):
        return np.loadtxt(path,       # assumes header present
                           delimiter=',',
                           skiprows=1)  # skip CSV header line
    else:
        raise ValueError("Unsupported file extension: " + ext)

def find_default_file():
    for ext in ('npy','csv','txt'):
        matches = glob.glob(f"emg_session*.{ext}")
        if matches:
            return matches[0]
    return None

def main():
    parser = argparse.ArgumentParser(description="Plot saved EMG data")
    parser.add_argument('file', nargs='?',
                        help="Path to emg_session.npy or .csv (optional)")
    parser.add_argument('--fs', type=float, default=250,
                        help="Sampling rate (Hz), default=250")
    parser.add_argument('--down', type=int, default=1,
                        help="Downsample factor for plotting (default=1=no down)")
    args = parser.parse_args()

    filepath = args.file or find_default_file()
    if not filepath:
        print("Errore: nessun file fornito e non ho trovato emg_session*.npy/.csv", file=sys.stderr)
        sys.exit(1)

    data = load_data(filepath)               # (samples, channels)
    ns, nc = data.shape
    t = np.arange(ns) / args.fs

    # eventualmente downsampling per plot
    if args.down > 1:
        t = t[::args.down]
        data = data[::args.down, :]

    fig, axes = plt.subplots(nc, 1, sharex=True, figsize=(10, 2*nc))
    fig.suptitle(f"EMG Session: {nc} ch, {ns} samp @ {args.fs}Hz (down×{args.down})")

    for ch in range(nc):
        ax = axes[ch]
        # linea continua, senza marker
        ax.plot(t, data[:, ch]/1000,
                linestyle='-', linewidth=0.8, marker=None)
        ax.set_ylabel(f"Ch {ch}", fontsize=8)
        ax.grid(True, which='both', linestyle='--', alpha=0.3)

    axes[-1].set_xlabel("Time (s)")
    plt.tight_layout(rect=[0,0,1,0.96])
    plt.show()

if __name__ == '__main__':
    main()

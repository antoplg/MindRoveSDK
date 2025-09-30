#!/usr/bin/env python3
"""
plot_emg_nmf_no_filter.py

Load an already-filtered EMG session from emg_session.npy (or CSV), apply rectification + RMS,
perform NMF with 2 components (flexion vs extension), and plot:

  1) Synergies spaziali (bar plot dei pesi sui canali)
  2) Curve di attivazione temporale delle due sinergie
"""

# Preference-informed Fuzzy Control of an Origami Supernumerary Robotic Limb via sEMG Feedback

import os
import glob
import sys
import numpy as np
import matplotlib.pyplot as plt
import argparse
from sklearn.decomposition import NMF

trim_percent        = 5
trim_percent_coco   = 5

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

def Perform_NMF(data):
    """
    Perform Non-negative Matrix Factorization (NMF) on the provided data.
    
    Parameters:
        data (np.ndarray): Input data matrix of shape (samples, channels).
        
    Returns:
        W (np.ndarray): Temporal activations of shape (samples, n_components).
        H (np.ndarray): Spatial synergies of shape (n_components, channels).
    """
    # Eseguo NMF con 2 componenti
    print("Eseguo NMF (n_components=2)…")
    model = NMF(
        n_components=2,
        init='nndsvda',
        solver='mu',
        beta_loss='frobenius',
        max_iter=1000,
        random_state=0
    )
    H = model.fit_transform(data)  # shape = (samples, 2)
    W = model.components_           # shape = (2, channels)
    return H, W

def compute_synergy_metrics(H, trim_percent):
    """
    Given:
      W    -- matrix of temporal activations, shape (ns, n_comp)
      H    -- matrix of spatial synergies,    shape (n_comp, n_ch)
      data -- raw (or preprocessed) EMG data,  shape (ns, n_ch)
      trim_percent -- percentage of highest H-values to ignore when computing maxima

    Returns:
      k_ext_max   -- max of H[0, :] after trimming top trim_percent%
      k_flex_max  -- max of H[1, :] after trimming top trim_percent%
      k_ext_min   -- 1.3 × min of H[0, :]
      k_flex_min  -- 1.3 × min of H[1, :]
    """
    # 1) trimming
    n = H.shape[0]
    n_trim = int(np.floor(n * trim_percent / 100))
    print(f"Trimming top {n_trim} values from each synergy")

    sorted_ext      = np.sort(H[:, 0])
    sorted_flex     = np.sort(H[:, 1])
    ext_trimmed     = sorted_ext[:-n_trim]
    flex_trimmed    = sorted_flex[:-n_trim]
 
    # 2) compute maxima on trimmed data
    k_ext_max  = ext_trimmed.max()
    k_flex_max = flex_trimmed.max()

    # 3) compute 1.3× minima on full data
    k_ext_min  = 1.3 * H[0, :].min()
    k_flex_min = 1.3 * H[1, :].min()

    return k_ext_max, k_flex_max, k_ext_min, k_flex_min


def main():
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

    print(f"Carico dati da {fp} …")
    data_raw    = load_data(fp)              # shape = (ns, nc)
    ns, nc  = data_raw.shape
    t       = np.arange(ns) / args.fs

    # Rettifica
    data = np.abs(data_raw)

    # Calcola RMS sulla finestra di lunghezza args.rms_win
    win_samps       = int(args.rms_win * args.fs)
    if win_samps < 1:
        win_samps   = 1
    kernel          = np.ones(win_samps) / win_samps

    # Per ciascuna colonna (canale) facciamo la media mobile dei quadrati
    for ch in range(nc):
        sq          = data[:, ch] ** 2
        movavg      = np.convolve(sq, kernel, mode='same')
        data[:, ch] = np.sqrt(movavg)

    # Esegui NMF
    H, W = Perform_NMF(data)

    # Scaling dei valori di H per le metriche
    k_ext_max, k_flex_max, k_ext_min, k_flex_min = compute_synergy_metrics(H, trim_percent)

    # Row-wise scaling:
    row0 = (H[:, 0] - k_ext_min) / (k_ext_max  - k_ext_min)
    row1 = (H[:, 1] - k_flex_min) / (k_flex_max - k_flex_min)

    print('shape(H) =', H.shape,'shape(W) =', W.shape)

    # Stack into a 2×n array:
    H_scaling = np.vstack((row0, row1)).T    # shape will be (2, n)

    # 1) Temporal activations
    plt.figure(figsize=(8, 3))
    plt.plot(t, H[:, 0], label="Flex activation", linewidth=1)
    plt.plot(t, H[:, 1], label="Ext activation", linewidth=1)
    plt.xlabel("Time (s)")
    plt.ylabel("Activation")
    plt.legend()
    plt.title("Temporal Activations")
    plt.tight_layout()

    # 2) Temporal activations scaled
    plt.figure(figsize=(8, 3))
    plt.plot(t, H_scaling[:, 0], label="Flex activation", linewidth=1)
    plt.plot(t, H_scaling[:, 1], label="Ext activation", linewidth=1)
    plt.xlabel("Time (s)")
    plt.ylabel("Activation")
    plt.legend()
    plt.title("Temporal Activations Scaled")
    plt.tight_layout()

    # 3) EMG no RMS plot
    fig, axes = plt.subplots(nc, 1, sharex=True, figsize=(10, 2*nc))
    fig.suptitle(f"EMG Session no RMS: {nc} ch, {ns} samp @ {args.fs}Hz")

    for ch in range(nc):
        ax = axes[ch]
        # linea continua, senza marker
        ax.plot(t, data_raw[:, ch]/1000,
                linestyle='-', linewidth=0.8, marker=None)
        ax.set_ylabel(f"Ch {ch}", fontsize=8)
        ax.grid(True, which='both', linestyle='--', alpha=0.3)

    axes[-1].set_xlabel("Time (s)")
    plt.tight_layout(rect=[0,0,1,0.96])

    # 4) EMG RMS plot
    fig, axes = plt.subplots(nc, 1, sharex=True, figsize=(10, 2*nc))
    fig.suptitle(f"EMG Session: {nc} ch, {ns} samp @ {args.fs}Hz")

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

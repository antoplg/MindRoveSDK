import csv
import time
import serial
import struct
import numpy as np
from scipy.stats import norm
from serial.tools import list_ports
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF, ConstantKernel as C

# ---- Serial Setup ----
baud_rate       = 115200
port_list       = list(list_ports.comports(include_links=False))
port_arduino    = port_list[0].device
print(port_arduino)  
ser             = serial.Serial(port_arduino, baud_rate)
time.sleep(2)

# ---- GP Setup ----
kernel  = C(1.0, (1e-2, 1e2)) * RBF(length_scale=0.4, length_scale_bounds=(1e-2, 1e1))
gp      = GaussianProcessRegressor(kernel=kernel, alpha=1e-2, normalize_y=True)

# ---- General Settings ----
NUM_ITERATIONS  = 10
RANGE           = (0.0, 1.0)
MIN_DISTANCE    = 0.15  # minimum allowed distance between val1 and val2
N_SAMPLES       = 1000     # number of candidates for EI sampling

# ---- Logging ----
results = []

def send_value_to_arduino(value, target):
    """
    target: 'G' for gripper, 'S' for SRL
    """
    ser.write(target.encode())  # send header
    ser.write(struct.pack('f', float(value)))  # send float
    while True:
        line = ser.readline().decode('utf-8').strip()
        if line == "OK":
            break

def get_user_choice():
    while True:
        choice = input("Choose [1] first, [2] second, [r] repeat, [q] quit: ").strip().lower()
        if choice in ['1', '2', 'r', 'q']:
            return choice
        else:
            print("Invalid input.")

def run_calibration(variable_name):
    print(f"\n--- Calibrating {variable_name.upper()} ---")
    X_train = []
    y_train = []

    previous_best = None

    for iteration in range(NUM_ITERATIONS):
        print(f"\nIteration {iteration + 1} / {NUM_ITERATIONS}")

        if iteration == 0:
            val1 = 0.1
            val2 = 0.9
        else:
            gp.fit(np.array(X_train), np.array(y_train))

            # Genera candidati ben distribuiti
            samples         = np.linspace(RANGE[0], RANGE[1], N_SAMPLES).reshape(-1, 1)
            tested_vals     = np.array(X_train).flatten()

            # Filtro: scarta valori troppo vicini a quelli già testati
            def is_far_enough(val, tested_vals, min_dist=MIN_DISTANCE):
                return np.all(np.abs(tested_vals - val) >= min_dist)

            filtered_samples = np.array([s for s in samples if is_far_enough(s[0], tested_vals)])

            # Se troppo pochi candidati validi, rilassa il filtro
            if len(filtered_samples) < 10:
                filtered_samples = samples

            mu, sigma = gp.predict(filtered_samples, return_std=True)

            # Parametri di esplorazione
            beta = 1.5       # meno esplorazione
            lambda_penalty = 0.3  # penalizzazione sulla distanza

            # Calcolo UCB
            ucb = mu + beta * sigma

            # Penalizza distanza dal valore preferito corrente (previous_best)
            if previous_best is not None:
                distances = np.abs(filtered_samples.flatten() - previous_best)
                ucb -= lambda_penalty * distances

            # Seleziona il punto con UCB penalizzato massimo
            val2 = filtered_samples[np.argmax(ucb)][0]

            # mu_sample_opt = np.max(gp.predict(np.array(X_train)))
            # imp = mu - mu_sample_opt
            # Z = imp / (sigma + 1e-9)
            # ei = imp * norm.cdf(Z) + sigma * norm.pdf(Z)
            # ei[sigma == 0.0] = 0.0

            # val2 = filtered_samples[np.argmax(ei)][0]
            val1            = previous_best

        print(f"Testing value 1: {val1:.3f}")
        send_value_to_arduino(val1, 'G' if variable_name == "gripper_velocity" else 'S')
        time.sleep(2)

        print(f"Testing value 2: {val2:.3f}")
        send_value_to_arduino(val2, 'G' if variable_name == "gripper_velocity" else 'S')
        time.sleep(2)

        while True:
            choice = get_user_choice()

            if choice == '1':
                X_train.append([val1])
                y_train.append(1)
                X_train.append([val2])
                y_train.append(0)
                previous_best = val1
                break  # esci dal while → passa a iterazione successiva
            elif choice == '2':
                X_train.append([val1])
                y_train.append(0)
                X_train.append([val2])
                y_train.append(1)
                previous_best = val2
                break
            elif choice == 'r':
                print("Repeating iteration...")
                print(f"Testing value 1: {val1:.3f}")
                send_value_to_arduino(val1, 'G' if variable_name == "gripper_velocity" else 'S')
                time.sleep(2)
                print(f"Testing value 2: {val2:.3f}")
                send_value_to_arduino(val2, 'G' if variable_name == "gripper_velocity" else 'S')
                time.sleep(2)
            elif choice == 'q':
                print("Calibration interrupted. Using previous best.")
                return previous_best

        results.append({'variable': variable_name, 'value': previous_best})

    print(f"\nBest {variable_name.upper()} value: {previous_best:.3f}")
    return previous_best

# ---- Main calibration ----
gripper_best = run_calibration("gripper_velocity")
srl_best = run_calibration("srl_velocity")

# ---- Save to CSV ----
with open("calibration_results.csv", mode='w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(["variable", "value"])
    writer.writerow(["gripper_velocity", f"{gripper_best:.6f}" if gripper_best is not None else ""])
    writer.writerow(["srl_velocity",     f"{srl_best:.6f}"     if srl_best is not None else ""])


print("\nCalibration completed. Results saved to 'calibration_results.csv'.")

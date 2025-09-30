import numpy as np
import matplotlib.pyplot as plt
from ahrs.filters import Madgwick
from scipy.integrate import cumtrapz
from scipy.signal import butter, filtfilt

def butter_lowpass_filter(data, cutoff, fs, order=2):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return filtfilt(b, a, data, axis=0)

# Supponiamo che tu abbia già i dati in queste variabili (Nx3 ciascuna)
# accelerometer = np.array([...]) # [m/s^2]
# gyroscope = np.array([...])     # [rad/s]
# timestamps = np.array([...])    # [s]

# --- ESEMPIO FINTI DATI (da sostituire con i tuoi)
imu_data = np.load("imu_session.npy")
N = len(imu_data)
print(f"Loaded IMU data with {N} samples.")
dt = 0.002  # 500 Hz
timestamps = np.linspace(0, (N) * dt, N)
accelerometer = imu_data[:, :3]*9.81  # Converti in m/s²
gyroscope = imu_data[:, 3:6]

fig, axs = plt.subplots(2, 1, figsize=(12, 6), sharex=True)

axs[0].plot(timestamps, accelerometer[:, :3])
axs[0].set_title("Accelerometer [m/s²]")
axs[0].legend(["X", "Y", "Z"])
axs[0].grid()

axs[1].plot(timestamps, gyroscope[:, :3])
axs[1].set_title("Gyroscope [rad/s]")
axs[1].legend(["X", "Y", "Z"])
axs[1].grid()

plt.tight_layout()
plt.show()

# --- 1. ORIENTATION ESTIMATION ---
madgwick = Madgwick(sampleperiod=dt)
quaternions = np.zeros((N, 4))
quaternions[0] = [1, 0, 0, 0]  # initial orientation

threshold_acc = 0.05  # m/s²
for k in range(3):
    for t in range(N):
        if abs(accelerometer[t,k]) < threshold_acc:
            accelerometer[t,k] = 0

threshold_gyr = 0.05  # rad/s
for k in range(3):
    for t in range(N):
        if abs(gyroscope[t,k]) < threshold_gyr:
            gyroscope[t,k] = 0

for t in range(1, N):
    acc_norm = np.linalg.norm(accelerometer[t])
    if acc_norm > 0:
        acc_unit = accelerometer[t] / acc_norm
    else:
        acc_unit = accelerometer[t]
    quaternions[t] = madgwick.updateIMU(quaternions[t-1], gyroscope[t], acc_unit)

# --- 2. GRAVITY COMPENSATION ---
def quaternion_conjugate(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])

def quaternion_multiply(q1, q2):
    w0, x0, y0, z0 = q1
    w1, x1, y1, z1 = q2
    return np.array([
        w0*w1 - x0*x1 - y0*y1 - z0*z1,
        w0*x1 + x0*w1 + y0*z1 - z0*y1,
        w0*y1 - x0*z1 + y0*w1 + z0*x1,
        w0*z1 + x0*y1 - y0*x1 + z0*w1
    ])

def rotate_vector_by_quaternion(v, q):
    v_quat = np.concatenate(([0], v))
    q_conj = quaternion_conjugate(q)
    return quaternion_multiply(quaternion_multiply(q, v_quat), q_conj)[1:]

acc_world = np.zeros_like(accelerometer)
g = np.array([0, 0, 9.81])

for t in range(N):
    # Rotate measured acceleration into world frame
    acc_world[t] = rotate_vector_by_quaternion(accelerometer[t], quaternions[t])
    # Remove gravity from world-frame acceleration
    acc_world[t] -= g

# Rimuovi eventuale drift iniziale se il sensore parte fermo
# bias = np.mean(acc_world[:100], axis=0)
# acc_world -= bias

# # Opzionale: ignora valori piccoli (filtro staticità)
threshold = 0.15  # m/s²
acc_magnitude = np.linalg.norm(acc_world, axis=1)
acc_world[acc_magnitude < threshold] = 0

# --- 3. VELOCITÀ E POSIZIONE ---
velocity = cumtrapz(acc_world, dx=dt, axis=0, initial=0)
position = cumtrapz(velocity, dx=dt, axis=0, initial=0)

# --- 4. PLOT TRAIETTORIA ---

def set_axes_equal(ax):
    ''' Imposta gli assi 3D con scala uguale (1:1:1) '''
    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    max_range = max(x_range, y_range, z_range)

    ax.set_xlim3d([x_middle - max_range/2, x_middle + max_range/2])
    ax.set_ylim3d([y_middle - max_range/2, y_middle + max_range/2])
    ax.set_zlim3d([z_middle - max_range/2, z_middle + max_range/2])

print(position)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(position[:, 0], position[:, 1], position[:, 2])
ax.set_title("3D Trajectory from 6DoF IMU")
ax.set_xlabel('X [m]')
ax.set_ylabel('Y [m]')
ax.set_zlabel('Z [m]')
set_axes_equal(ax)
plt.show()

# Apply high-pass filter to acc_world to reduce drift
def highpass_filter(data, cutoff, fs, order=2):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='high', analog=False)
    return filtfilt(b, a, data, axis=0)

acc_world = highpass_filter(acc_world, cutoff=0.1, fs=1/dt)

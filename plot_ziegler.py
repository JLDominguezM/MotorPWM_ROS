import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage import gaussian_filter1d

# 1. Load your data
# Make sure the filename matches your text file
# Assuming your file has a header row like "Time(ms),RPM"
df = pd.read_csv('./experiments/Experiment_1.csv')

# Extract columns
time_ms = df.iloc[:, 0].values
rpm = df.iloc[:, 1].values

# 2. Isolate the step response (the step happened at 2000ms)
step_start_ms = 2000
valid_data = time_ms >= step_start_ms

# Convert time to seconds, starting at 0
t_sec = (time_ms[valid_data] - step_start_ms) / 1000.0 
y_rpm = rpm[valid_data]

# 3. Smooth the noise out of the RPM data so the derivative math works cleanly
y_smooth = gaussian_filter1d(y_rpm, sigma=2.0)

# 4. Find Steady State (Max RPM) and Process Gain (K)
# Average the last 20% of the data to get a stable max RPM
steady_state_idx = int(len(y_rpm) * 0.8)
max_rpm = np.mean(y_rpm[steady_state_idx:])
pwm_step = 150.0 # The PWM value we sent to the motor
K = max_rpm / pwm_step

# 5. Find the inflection point (maximum slope) using a derivative
dy_dt = np.gradient(y_smooth, t_sec)
max_slope_idx = np.argmax(dy_dt)

slope = dy_dt[max_slope_idx]
t_inflect = t_sec[max_slope_idx]
y_inflect = y_smooth[max_slope_idx]

# 6. Calculate Dead Time (L) and Time Constant (T)
# We use the point-slope formula of our tangent line to find where it hits 0 and Max RPM
L = t_inflect - (y_inflect / slope)
if L < 0: L = 0.001 # Fallback for near-instant dead time

t_steady = t_inflect + ((max_rpm - y_inflect) / slope)
T = t_steady - L

# Print the results
print("--- Ziegler-Nichols Open Loop Parameters ---")
print(f"Max Steady RPM:  {max_rpm:.1f}")
print(f"Process Gain (K):{K:.3f}")
print(f"Dead Time (L):   {L:.4f} seconds")
print(f"Time Const (T):  {T:.4f} seconds")
print("--------------------------------------------")

# 7. Plot the results to visually verify the math
plt.figure(figsize=(10, 6))
plt.plot(t_sec, y_rpm, label='Raw Data', alpha=0.3)
plt.plot(t_sec, y_smooth, label='Smoothed Data', linewidth=2)

# Draw tangent line
t_tangent = np.array([L, t_steady])
y_tangent = np.array([0, max_rpm])
plt.plot(t_tangent, y_tangent, 'r--', label='Tangent Line', linewidth=2)

plt.axhline(max_rpm, color='g', linestyle=':', label=f'Max RPM ({max_rpm:.0f})')
plt.axvline(L, color='purple', linestyle=':', label=f'Dead Time L ({L:.3f}s)')

plt.xlabel('Time after step (seconds)')
plt.ylabel('RPM')
plt.title('Ziegler-Nichols Step Response Analysis')
plt.legend()
plt.grid(True)
plt.show()

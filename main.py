import matplotlib
matplotlib.use("TkAgg")

import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

PORT     = "/dev/ttyACM0"
BAUDRATE = 115200
MAX_POINTS = 100

raw_data      = deque([0.0] * MAX_POINTS, maxlen=MAX_POINTS)
filtered_data = deque([0.0] * MAX_POINTS, maxlen=MAX_POINTS)
velocity_data = deque([0.0] * MAX_POINTS, maxlen=MAX_POINTS)

ser = serial.Serial(PORT, BAUDRATE, timeout=1)

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6))
fig.suptitle("HC-SR04 + 2-State Kalman Filter")

line_raw,      = ax1.plot([], [], label="Raw (cm)",      color="red",   alpha=0.5)
line_filtered, = ax1.plot([], [], label="Filtered (cm)", color="blue",  linewidth=2)
line_velocity, = ax2.plot([], [], label="Velocity (cm/s)", color="green", linewidth=2)

ax1.set_ylim(0, 200)
ax1.set_xlim(0, MAX_POINTS)
ax1.set_ylabel("Distance (cm)")
ax1.legend(loc="upper right")
ax1.grid(True, alpha=0.3)
ax1.axhline(y=0, color="black", linewidth=0.5)

ax2.set_ylim(-50, 50)
ax2.set_xlim(0, MAX_POINTS)
ax2.set_ylabel("Velocity (cm/s)")
ax2.set_xlabel("Samples")
ax2.legend(loc="upper right")
ax2.grid(True, alpha=0.3)
ax2.axhline(y=0, color="black", linewidth=0.5)

def update(frame):
    try:
        line = ser.readline().decode("utf-8").strip()

        # Print every line to the terminal so you can see what arrives
        print(repr(line))

        parts = {}
        for token in line.split():
            if ":" in token:
                key, val = token.split(":")
                parts[key] = float(val)

        if "RAW" in parts and "DIST" in parts and "VEL" in parts:
            raw_data.append(parts["RAW"])
            filtered_data.append(parts["DIST"])
            velocity_data.append(parts["VEL"])

            x = range(MAX_POINTS)
            line_raw.set_data(x, list(raw_data))
            line_filtered.set_data(x, list(filtered_data))
            line_velocity.set_data(x, list(velocity_data))

    except Exception as e:
        print(f"Parse error: {e}")

    return line_raw, line_filtered, line_velocity

ani = animation.FuncAnimation(fig, update, interval=60, blit=True, cache_frame_data=False)
plt.tight_layout()
plt.show()

ser.close()
import sys
import serial
import matplotlib.pyplot as plt

if len(sys.argv) < 2:
    print("Usage: python3 position_sample_plot.py <serial_port>")
    sys.exit(1)

port = sys.argv[1]
ser = serial.Serial(port, 115200)

pos = []
vel_esc = []
vel_calc = []

print("[INFO] Collecting samples...")

while True:
    line = ser.readline().decode("utf-8").strip()
    if not line:
        continue
    if line == "END":
        break

    try:
        p, vesc, vcalc = map(float, line.split(","))
        pos.append(p)
        vel_esc.append(vesc)
        vel_calc.append(vcalc)
    except ValueError:
        continue

print(f"[INFO] Got {len(pos)} samples.")

# Plot results
fig, axs = plt.subplots(2, 1, sharex=True, figsize=(10,6))

axs[0].plot(pos, label="Position [rad]")
axs[0].set_ylabel("Position (rad)")
axs[0].legend()
axs[0].grid(True)

axs[1].plot(vel_esc, label="ESC Velocity [rad/s]")
axs[1].plot(vel_calc, label="Calc Velocity [rad/s]", linestyle="--")
axs[1].set_ylabel("Velocity (rad/s)")
axs[1].set_xlabel("Sample Index")
axs[1].legend()
axs[1].grid(True)

plt.show()

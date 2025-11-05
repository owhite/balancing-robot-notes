#!/usr/bin/env python3

import sys, json, serial, math
import argparse
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Button
import numpy as np

def send_run_command(event, ser):
    """Handle Run button click → tell Teensy to enter balance mode."""
    msg = {"cmd": "run_balance"}
    print("TX:", msg)
    ser.write((json.dumps(msg) + "\n").encode("utf-8"))

def main():
    parser = argparse.ArgumentParser(description="Two-Wheeled Robot live IMU plotter.")
    parser.add_argument("-p", "--port", required=True, help="Serial port (e.g. /dev/ttyACM0)")
    args = parser.parse_args()

    ser = serial.Serial(args.port, 115200, timeout=0.05)
    print(f"✅ Connected to {args.port}")

    # --- Setup plots ---
    fig, (ax_roll, ax_rate) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    plt.subplots_adjust(left=0.1, right=0.75, top=0.92, bottom=0.1)

    line_roll, = ax_roll.plot([], [], color="tab:orange", label="Roll [deg]")
    ax_roll.set_title("IMU Roll Angle")
    ax_roll.set_ylabel("Angle (deg)")
    ax_roll.grid(True)
    ax_roll.legend()
    ax_roll.set_ylim(-2, 2)   # <-- fixed limits

    line_rate, = ax_rate.plot([], [], color="tab:blue", label="Roll Rate [deg/s]")
    ax_rate.set_title("IMU Roll Rate")
    ax_rate.set_xlabel("Time (ms)")
    ax_rate.set_ylabel("Rate (deg/s)")
    ax_rate.grid(True)
    ax_rate.legend()
    ax_rate.set_ylim(-2, 2)   # <-- fixed limits

    # --- UI Button ---
    axbutton = plt.axes([0.8, 0.85, 0.15, 0.08])
    button = Button(axbutton, "Run")
    button.on_clicked(lambda event: send_run_command(event, ser))

    # --- Data buffers ---
    t_vals, roll_vals, rate_vals = [], [], []
    max_points = 2000

    def update(frame):
        nonlocal t_vals, roll_vals, rate_vals
        while ser.in_waiting:
            try:
                line = ser.readline().decode("utf-8").strip()
                if not line:
                    continue
                data = json.loads(line)
                if all(k in data for k in ("t", "roll", "roll_rate")):
                    t_vals.append((data["t"] - (t_vals[0] if t_vals else data["t"])) / 1000.0)
                    roll_vals.append(data["roll"])
                    rate_vals.append(data["roll_rate"])

                    # Trim to buffer size
                    if len(t_vals) > max_points:
                        t_vals, roll_vals, rate_vals = (
                            t_vals[-max_points:], roll_vals[-max_points:], rate_vals[-max_points:]
                        )
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue

        if t_vals:
            line_roll.set_data(t_vals, roll_vals)
            line_rate.set_data(t_vals, rate_vals)

            # Dynamic axes
            ax_roll.set_xlim(min(t_vals), max(t_vals))
            # for ax, vals in [(ax_roll, roll_vals), (ax_rate, rate_vals)]:
                # ymin, ymax = min(vals), max(vals)
                # margin = 0.1 * (ymax - ymin) if ymax > ymin else 1.0
                # ax.set_ylim(ymin - margin, ymax + margin)

        return (line_roll, line_rate)

    ani = animation.FuncAnimation(fig, update, interval=100, blit=False, cache_frame_data=False)
    plt.show()

if __name__ == "__main__":
    main()

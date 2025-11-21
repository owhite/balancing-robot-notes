#!/usr/bin/env python3
import sys, json, serial, argparse
import matplotlib.pyplot as plt
import matplotlib.animation as animation

def main():
    parser = argparse.ArgumentParser(description="Plot a_mag from serial JSON")
    parser.add_argument("-p", "--port", required=True, help="Serial port")
    args = parser.parse_args()

    # Open serial port
    ser = serial.Serial(args.port, 115200, timeout=0.05)
    print(f"Connected to {args.port}")

    # Matplotlib setup
    fig, ax = plt.subplots(figsize=(10,5))
    ax.set_title("a_mag over time (last 200 samples)")
    ax.set_xlabel("Sample index")
    ax.set_ylabel("a_mag")
    ax.grid(True)

    line_mag, = ax.plot([], [], label="a_mag")
    ax.legend()

    # Buffers
    amag_vals = []

    WINDOW = 200

    def update(frame):
        nonlocal amag_vals

        # Read whatever is in the buffer
        while ser.in_waiting:
            try:
                line = ser.readline().decode("utf-8").strip()
                if not line:
                    continue

                data = json.loads(line)

                if "a_mag" in data:
                    amag_vals.append(data["a_mag"])

                    # keep only last 200 values
                    if len(amag_vals) > WINDOW:
                        amag_vals = amag_vals[-WINDOW:]

            except (UnicodeDecodeError, json.JSONDecodeError):
                continue

        if len(amag_vals) > 1:
            x = list(range(len(amag_vals)))
            y = amag_vals
            line_mag.set_data(x, y)

            ax.set_xlim(0, WINDOW-1)
            ax.set_ylim(min(y) - 0.5, max(y) + 0.5)

        return line_mag,

    ani = animation.FuncAnimation(
        fig,
        update,
        interval=50,
        blit=False
    )

    plt.show()

if __name__ == "__main__":
    main()

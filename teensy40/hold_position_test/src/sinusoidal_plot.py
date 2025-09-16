import serial
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtCore
import sys
import numpy as np
from collections import deque

# ----- CONFIG -----
BAUD = 115200
HISTORY_SEC = 5.0       # seconds of data to keep in scrolling plots
LOOP_HZ = 1000          # expected control loop frequency
BUF_LEN = int(HISTORY_SEC * LOOP_HZ)

# ----- SERIAL PORT FROM ARGUMENT -----
if len(sys.argv) < 2:
    print("Usage: python realtime_plot.py <serial_port>")
    sys.exit(1)

PORT = sys.argv[1]
print(f"[INFO] Connecting to {PORT} at {BAUD} baud...")

try:
    ser = serial.Serial(PORT, BAUD, timeout=1)
except serial.SerialException as e:
    print(f"[ERROR] Could not open port {PORT}: {e}")
    sys.exit(1)

# ----- DATA BUFFERS -----
time_buf = deque(maxlen=BUF_LEN)
hold_buf = deque(maxlen=BUF_LEN)
pos_buf  = deque(maxlen=BUF_LEN)
err_buf  = deque(maxlen=BUF_LEN)

latest_stats = {}

# ----- PLOTTING SETUP -----
app = QtGui.QApplication([])
win = pg.GraphicsLayoutWidget(show=True, title="Teensy Control System Telemetry")
win.resize(1000,600)

# Plot 1: Reference vs Measured
p1 = win.addPlot(title="Reference vs Measured Position")
curve_hold = p1.plot(pen="r", name="hold_pos")
curve_pos  = p1.plot(pen="y", name="pos")

# Plot 2: Error
win.nextRow()
p2 = win.addPlot(title="Position Error")
curve_err = p2.plot(pen="c", name="pos_err")

# Text label for stats
stats_label = pg.LabelItem(justify="left")
win.addItem(stats_label, row=2, col=0)

# ----- UPDATE LOOP -----
def update():
    global latest_stats
    line = ser.readline().decode("utf-8").strip()
    if not line or "," not in line:
        return

    try:
        (t_us, hold, pos, err, kspring, b, dt, min_dt, max_dt, avg_dt, exec_us) = map(float, line.split(","))
    except ValueError:
        return  # skip malformed lines

    t_s = t_us / 1e6

    # append to buffers
    time_buf.append(t_s)
    hold_buf.append(hold)
    pos_buf.append(pos)
    err_buf.append(err)

    # update curves
    curve_hold.setData(time_buf, hold_buf)
    curve_pos.setData(time_buf, pos_buf)
    curve_err.setData(time_buf, err_buf)

    # update stats
    latest_stats = {
        "min_dt": min_dt,
        "max_dt": max_dt,
        "avg_dt": avg_dt,
        "exec": exec_us,
        "dt": dt,
        "kspring": kspring,
        "b": b
    }
    stats_text = "\n".join([f"{k}: {v:.3f}" for k, v in latest_stats.items()])
    stats_label.setText(stats_text)

# Timer for GUI updates
timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(10)   # update every 10 ms

# Run
if __name__ == "__main__":
    if (sys.flags.interactive != 1) or not hasattr(QtCore, "PYQT_VERSION"):
        QtGui.QApplication.instance().exec_()

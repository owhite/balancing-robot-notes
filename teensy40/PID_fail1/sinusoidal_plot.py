import sys
import serial
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets
import collections
import time

if len(sys.argv) < 2:
    print("Usage: python sinusoidal_plot.py <serial_port>")
    sys.exit(1)

port = sys.argv[1]

# Serial setup
print(f"[INFO] Connecting to {port} at 115200 baud...")
ser = serial.Serial(port, 921600, timeout=0.1)

# Buffers
BUF_LEN = 1000
time_buf = collections.deque(maxlen=BUF_LEN)
hold_buf = collections.deque(maxlen=BUF_LEN)
pos_buf = collections.deque(maxlen=BUF_LEN)
err_buf = collections.deque(maxlen=BUF_LEN)
vel_buf = collections.deque(maxlen=BUF_LEN)
vel_sign_buf = collections.deque(maxlen=BUF_LEN)
torque_buf = collections.deque(maxlen=BUF_LEN)
torque_raw_buf = collections.deque(maxlen=BUF_LEN)

# PyQtGraph setup
app = QtWidgets.QApplication([])
win = pg.GraphicsLayoutWidget(show=True, title="Sinusoidal Tracking Telemetry")
win.resize(1000, 800)

# Plot 1: Position tracking
p1 = win.addPlot(title="Position Tracking")
p1.setLabel('left', "Position [rad]")
p1.setLabel('bottom', "Time [s]")
p1.setYRange(0, 2*3.14159)  # lock y-axis to [0, 2π]
curve_hold = p1.plot(pen='r', name="Hold Position")
curve_pos = p1.plot(pen='y', name="Measured Position")
win.nextRow()

# Plot 2: Error
p2 = win.addPlot(title="Position Error")
p2.setLabel('left', "Error [rad]")
p2.setLabel('bottom', "Time [s]")
curve_err = p2.plot(pen='w', name="Error")
win.nextRow()

# Plot 3: Velocity + sign
p3 = win.addPlot(title="Velocity and Sign")
p3.setLabel('left', "Velocity [rad/s]")
p3.setLabel('bottom', "Time [s]")
curve_vel = p3.plot(pen='c', name="Velocity")
curve_vel_sign = p3.plot(pen='m', name="Velocity Sign")
win.nextRow()

# Plot 4: Torque
p4 = win.addPlot(title="Torque Command")
p4.setLabel('left', "Torque [Nm]")
p4.setLabel('bottom', "Time [s]")
curve_torque = p4.plot(pen='g', name="Clamped Torque")
curve_torque_raw = p4.plot(pen='r', name="Raw Torque")

last_data_time = time.time()

def update():
    global latest_stats, last_data_time
    line = ser.readline().decode("utf-8").strip()
    if not line or "," not in line:
        return
    try:
        (t_us, hold, pos, err, vel, vel_sign, torque_raw, torque) = line.split(",")
        t_us = float(t_us)
        hold = float(hold)
        pos = float(pos)
        err = float(err)
        vel = float(vel)
        vel_sign = int(vel_sign)
        torque_raw = float(torque_raw)
        torque = float(torque)
 


    except ValueError:
        return

    t_s = t_us / 1e6
    last_data_time = time.time()

    # Append
    time_buf.append(t_s)
    hold_buf.append(hold)
    pos_buf.append(pos)
    err_buf.append(err)
    vel_buf.append(vel)
    vel_sign_buf.append(vel_sign)
    torque_buf.append(torque)
    torque_raw_buf.append(torque_raw)

    # Update plots
    curve_hold.setData(time_buf, hold_buf)
    curve_pos.setData(time_buf, pos_buf)
    curve_err.setData(time_buf, err_buf)
    curve_vel.setData(time_buf, vel_buf)
    curve_vel_sign.setData(time_buf, vel_sign_buf)
    curve_torque.setData(time_buf, torque_buf)
    curve_torque_raw.setData(time_buf, torque_raw_buf)


timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(20)

# Handle Ctrl-C gracefully
def handle_sigint(*args):
    print("\n[INFO] Exiting...")
    app.quit()

import signal
signal.signal(signal.SIGINT, handle_sigint)

if __name__ == '__main__':
    QtWidgets.QApplication.instance().exec_()

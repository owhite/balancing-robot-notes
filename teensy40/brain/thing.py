import serial
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets

ser = serial.Serial('/dev/cu.usbmodem178888901', 921600, timeout=1)

app = QtWidgets.QApplication([])
win = pg.GraphicsLayoutWidget(show=True, title="Sinusoidal Tracking Debug")
win.resize(1200, 800)

# Plots
p1 = win.addPlot(title="Position Tracking")
p2 = win.addPlot(title="Position Error")
p3 = win.addPlot(title="Velocity and Sign")
p4 = win.addPlot(title="Torque Command")

win.nextRow()

curve_hold = p1.plot(pen='r')
curve_pos = p1.plot(pen='y')
curve_err = p2.plot(pen='w')
curve_vel = p3.plot(pen='c')
curve_velsign = p3.plot(pen='m')
curve_torque = p4.plot(pen='g')
curve_torque_raw = p4.plot(pen='b')  # NEW raw torque curve

# Buffers
N = 2000
data_hold = []
data_pos = []
data_err = []
data_vel = []
data_velsign = []
data_torque = []
data_torque_raw = []

def update():
    global data_hold, data_pos, data_err, data_vel, data_velsign, data_torque, data_torque_raw
    line = ser.readline().decode("utf-8").strip()
    if not line or "," not in line:
        return

    try:
        (t_us, hold, pos, err, vel, vel_sign,
         torque, torque_raw, kspring, b,
         dt, min_dt, max_dt, avg_dt, exec_us) = line.split(",")

        hold = float(hold)
        pos = float(pos)
        err = float(err)
        vel = float(vel)
        vel_sign = int(vel_sign)
        torque = float(torque)
        torque_raw = float(torque_raw)

        # append
        data_hold.append(hold)
        data_pos.append(pos)
        data_err.append(err)
        data_vel.append(vel)
        data_velsign.append(vel_sign * 50)  # scaled for visibility
        data_torque.append(torque)
        data_torque_raw.append(torque_raw)

        # keep buffers trimmed
        for arr in [data_hold, data_pos, data_err, data_vel, data_velsign, data_torque, data_torque_raw]:
            if len(arr) > N:
                arr.pop(0)

        # update curves
        curve_hold.setData(data_hold)
        curve_pos.setData(data_pos)
        curve_err.setData(data_err)
        curve_vel.setData(data_vel)
        curve_velsign.setData(data_velsign)
        curve_torque.setData(data_torque)
        curve_torque_raw.setData(data_torque_raw)

    except ValueError:
        # skip malformed line
        return

timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(0)

if __name__ == '__main__':
    QtWidgets.QApplication.instance().exec_()

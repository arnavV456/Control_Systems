import sys
import serial
import time
from collections import deque

import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets, QtCore

# ---------- CONFIG ----------
PORT = "/dev/ttyUSB0"   # change if needed
BAUD = 115200
BUFFER_SIZE = 500       # samples in plot window

# ---------- SERIAL ----------
ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)

# ---------- DATA BUFFERS ----------
roll_buf  = deque(maxlen=BUFFER_SIZE)
pitch_buf = deque(maxlen=BUFFER_SIZE)
x_buf     = deque(maxlen=BUFFER_SIZE)

t0 = time.time()

# ---------- QT APP ----------
app = QtWidgets.QApplication(sys.argv)
win = pg.GraphicsLayoutWidget(title="ISM330DHCX Roll & Pitch (Real-Time)")
win.show()

plot = win.addPlot(title="Roll & Pitch")
plot.setLabel('left', 'Angle (deg)')
plot.setLabel('bottom', 'Time (s)')
plot.addLegend()
plot.showGrid(x=True, y=True)

roll_curve  = plot.plot(pen=pg.mkPen('r', width=2), name="Roll")
pitch_curve = plot.plot(pen=pg.mkPen('b', width=2), name="Pitch")

plot.setYRange(-90, 90)

# ---------- UPDATE FUNCTION ----------
def update():
    while ser.in_waiting:
        line = ser.readline().decode(errors="ignore").strip()

        if not line.startswith("ROLL"):
            continue

        try:
            # ROLL:x,PITCH:y
            parts = line.replace("ROLL:", "").replace("PITCH:", "").split(",")
            roll  = float(parts[0])
            pitch = float(parts[1])
        except ValueError:
            return

        t = time.time() - t0

        roll_buf.append(roll)
        pitch_buf.append(pitch)
        x_buf.append(t)

    roll_curve.setData(x_buf, roll_buf)
    pitch_curve.setData(x_buf, pitch_buf)

# ---------- TIMER ----------
timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(20)   # 50 Hz GUI refresh (perfect for humans)

# ---------- START ----------
sys.exit(app.exec_())

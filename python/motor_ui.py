import sys
import time
import numpy as np
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg

class MotorUI(QtWidgets.QWidget):
    def __init__(self, motor, time_window=2.0):
        super().__init__()
        self.motor = motor
        self.time_window = time_window
        self.init_ui()
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(30)  # 30ms refresh for smooth UI
        # Dedicated buffer for plotting (only stores data within the time window)
        self.plot_buffer = []

    def init_ui(self):
        self.setWindowTitle("Motor Monitor UI (PyQtGraph)")
        layout = QtWidgets.QVBoxLayout(self)

        # Plot widgets
        self.plot_q = pg.PlotWidget(title="Motor Position (q)")
        self.plot_dq = pg.PlotWidget(title="Motor Velocity (dq)")
        self.curve_q = self.plot_q.plot(pen='b')
        self.curve_dq = self.plot_dq.plot(pen='y')
        layout.addWidget(self.plot_q)
        layout.addWidget(self.plot_dq)

        # Controls
        ctrl_layout = QtWidgets.QHBoxLayout()
        self.time_entry = QtWidgets.QLineEdit(str(self.time_window))
        self.apply_btn = QtWidgets.QPushButton("Apply")
        self.apply_btn.clicked.connect(self.apply_time_window)
        self.collision_label = QtWidgets.QLabel("Collision: None")
        self.collision_label.setStyleSheet("background-color: gray")
        self.collision_count_label = QtWidgets.QLabel("Collision Count: 0")
        self.reset_collision_btn = QtWidgets.QPushButton("Reset Collision")
        self.reset_collision_btn.clicked.connect(self.reset_collision)
        ctrl_layout.addWidget(QtWidgets.QLabel("Time Window (s):"))
        ctrl_layout.addWidget(self.time_entry)
        ctrl_layout.addWidget(self.apply_btn)
        ctrl_layout.addWidget(self.collision_label)
        ctrl_layout.addWidget(self.collision_count_label)
        ctrl_layout.addWidget(self.reset_collision_btn)
        layout.addLayout(ctrl_layout)

    def apply_time_window(self):
        try:
            val = float(self.time_entry.text())
            if val > 0.1:
                self.time_window = val
        except ValueError:
            pass

    def reset_collision(self):
        self.motor.reset_collision_flag()

    def update_plot(self):
        buffer = self.motor.get_data_buffer()
        with buffer.lock:
            data = list(buffer.data)
        if not data:
            return
        now = time.time()
        min_time = now - self.time_window
        # Only keep data within the time window in the dedicated plot buffer
        self.plot_buffer = [d for d in data if d[0] >= min_time]
        if not self.plot_buffer:
            return
        times, qs, dqs, collisions = zip(*self.plot_buffer)
        times = np.array(times)
        qs = np.array(qs)
        dqs = np.array(dqs)
        collisions = np.array(collisions)
        times = times - times[0]
        self.curve_q.setData(times, qs)
        self.curve_dq.setData(times, dqs)
        self.plot_q.setXRange(0, self.time_window)
        self.plot_dq.setXRange(0, self.time_window)
        if len(qs) > 0:
            min_q, max_q = np.min(qs), np.max(qs)
            if min_q == max_q:
                min_q -= 0.1
                max_q += 0.1
            self.plot_q.setYRange(min_q, max_q)
        if len(dqs) > 0:
            min_dq, max_dq = np.min(dqs), np.max(dqs)
            if min_dq == max_dq:
                min_dq -= 0.1
                max_dq += 0.1
            self.plot_dq.setYRange(min_dq, max_dq)
        # Collision indicator
        collision_now = self.motor.collision_latched
        if collision_now:
            self.collision_label.setText("Collision: Yes")
            self.collision_label.setStyleSheet("background-color: red")
        else:
            self.collision_label.setText("Collision: None")
            self.collision_label.setStyleSheet("background-color: gray")
        self.collision_count_label.setText(f"Collision Count: {self.motor.collision_count}")

    def closeEvent(self, event):
        self.timer.stop()
        event.accept()

# Usage in main:
if __name__ == "__main__":
    from goM8010_6_motor_collision_detection import Motor
    app = QtWidgets.QApplication(sys.argv)
    motor0 = Motor("/dev/ttyUSB0", 0)
    motor0.start()
    ui = MotorUI(motor0)
    ui.show()
    app.exec_()
    motor0.stop() 
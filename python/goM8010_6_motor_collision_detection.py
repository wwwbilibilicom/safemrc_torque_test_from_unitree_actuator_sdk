import time
from PyQt5 import QtWidgets
from motor_ui import MotorUI
import sys
sys.path.append('../lib')
from unitree_actuator_sdk import *
from scipy.signal import butter, lfilter, lfilter_zi
import threading
import tkinter as tk
import numpy as np

class BandpassFilter:
    def __init__(self, lowcut, highcut, fs, order=4):
        self.lowcut = lowcut
        self.highcut = highcut
        self.fs = fs
        self.order = order
        nyq = 0.5 * fs
        low = lowcut / nyq
        high = highcut / nyq
        self.b, self.a = butter(order, [low, high], btype='bandpass')
        self.zi = lfilter_zi(self.b, self.a)  # 初始化滤波器状态

    def update(self, data):
        y, self.zi = lfilter(self.b, self.a, [data], zi=self.zi)
        return y[0]

class DataBuffer:
    def __init__(self):
        self.data = []
        self.lock = threading.Lock()
        self.collision = False

    def add(self, q, dq, collision, timestamp):
        with self.lock:
            self.data.append((timestamp, q, dq, collision))
            if collision:
                self.collision = True

    def get_batch(self, batch_size):
        with self.lock:
            batch = self.data[:batch_size]
            self.data = self.data[batch_size:]
            collision = self.collision
            self.collision = False
            return batch, collision

class Motor:
    def __init__(self, serial_number, id):
        self.id = id
        self.cmd = MotorCmd()
        self.data = MotorData()
        self.serial = SerialPort(serial_number)
        self.position = 0.0

        self.cmd.motorType = MotorType.GO_M8010_6
        self.cmd.mode = MotorMode.FOC
        self.gearRatio = queryGearRatio(MotorType.GO_M8010_6)
        self.cmd.q = 0.0
        self.cmd.dq = 0.0
        self.cmd.kp = 0.0
        self.cmd.kd = 0.0
        self.cmd.tau = 0.0
        self.cmd.id = self.id

        self.data.motorType = MotorType.GO_M8010_6
        self.data.q = 0.0
        self.data.dq = 0.0
        self.data.tau = 0.0

        self.bandpass_filter = BandpassFilter(30, 150, 1000)
        self.collision_flag = False
        self.collision_latched = False  # Latch for UI reset
        self.data_buffer = DataBuffer()
        self.collision_count = 0  # collision count
        self._running = False
        self._thread = None

        # Zero offset calibration
        self.zero_offset = 0.0
        self._calibrate_zero_offset()
        self.cmd.kp = 25.0
        self.cmd.kd = 0.1

    def _calibrate_zero_offset(self):
        print("Calibrating zero offset... Please keep the motor still.")
        q_sum = 0.0
        n = 1000
        for i in range(n):
            self.exchange_data()
            print(self.data.q)
            q_sum += self.data.q
            time.sleep(0.005)  # 5ms between reads
        self.zero_offset = q_sum / n
        print(f"Zero offset calibrated: {self.zero_offset:.5f}")

    def exchange_data(self):
        self.cmd.q = self.position*self.gearRatio + self.zero_offset
        self.serial.sendRecv(self.cmd, self.data)
        # When reading, offset data.q by zero_offset
        self.data.q = self.data.q

    def collision_detection(self):
        # If a collision has been latched, keep it True until reset
        if abs(self.bandpass_filter.update(self.data.dq)) > 2.0:
            if not self.collision_flag:
                self.collision_count += 1
            self.collision_flag = True
            self.collision_latched = True
            return True
        else:
            if self.collision_latched:
                return True
            self.collision_flag = False
            return False

    def reset_collision_flag(self):
        self.collision_latched = False
        self.collision_flag = False
        self.cmd.kp = 25.0
        self.cmd.kd = 0.1
        
    def run(self):
        self._running = True
        while self._running:
            self.exchange_data()
            q = self.data.q
            dq = self.data.dq
            collision = self.collision_detection()
            if self.collision_latched:
                self.cmd.kp = 0.0
                self.cmd.kd = 0.01
                self.zero_offset = self.data.q
            timestamp = time.time()
            self.data_buffer.add(q, dq, collision, timestamp)
            print(f"motor {self.id}: cmd.q={self.cmd.q:>8.3f} | q={q:>10.5f} | dq={dq:>10.5f} | collision={str(collision):>5} | collision_count={self.collision_count:>3}")
            time.sleep(0.001)  # 10ms定时

    def start(self):
        if self._thread is None:
            self._thread = threading.Thread(target=self.run, daemon=True)
            self._thread.start()

    def stop(self):
        self._running = False
        if self._thread is not None:
            self._thread.join()
            self._thread = None

    def get_data_buffer(self):
        return self.data_buffer

if __name__ == "__main__":
    motor0 = Motor("/dev/ttyUSB0", 0)
    motor0.start()
    app = QtWidgets.QApplication(sys.argv)
    ui = MotorUI(motor0)
    ui.show()
    app.exec_()
    motor0.stop()


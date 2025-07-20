import time
import sys
sys.path.append('../lib')
from unitree_actuator_sdk import *
from scipy.signal import butter, lfilter
import threading
from motor_ui import MotorUI
import tkinter as tk

class BandpassFilter:
    def __init__(self, lowcut, highcut, fs, order=4):
        self.lowcut = lowcut
        self.highcut = highcut
        self.fs = fs
        self.order = order
        self.b, self.a = butter(order, [lowcut, highcut], btype='bandpass')

    def update(self, data):
        return lfilter(self.b, self.a, data)

class DataBuffer:
    def __init__(self):
        self.data = []
        self.lock = threading.Lock()
        self.collision = False

    def add(self, q, dq, collision):
        with self.lock:
            self.data.append((q, dq, collision))
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
    def __init__(self, serial, id):
        self.serial = serial
        self.id = id
        self.cmd = MotorCmd()
        self.data = MotorData()

        self.cmd.motorType = MotorType.GO_M8010_6
        self.cmd.mode = MotorMode.FOC
        self.cmd.q = 0.0
        self.cmd.dq = 6.28*queryGearRatio(MotorType.GO_M8010_6)
        self.cmd.kp = 0.0
        self.cmd.kd = 0.01
        self.cmd.tau = 0.0
        self.cmd.id = self.id

        self.data.id = self.id
        self.data.motorType = MotorType.GO_M8010_6
        self.data.q = 0.0
        self.data.dq = 0.0
        self.data.kp = 0.0
        self.data.kd = 0.0
        self.data.tau = 0.0

        self.bandpass_filter = BandpassFilter(30, 150, 1000)
        self.collision_flag = False

    def exchange_data(self):
        self.serial.sendRecv(self.cmd, self.data)

    def collision_detection(self):
        if(abs(self.bandpass_filter.update(self.data.dq)) > 2.0):
            self.collision_flag = True
            return True
        else:
            self.collision_flag = False
            return False

def motor_worker(motor, data_buffer, ui):
    while True:
        motor.exchange_data()
        q = motor.data.q
        dq = motor.data.dq
        collision = motor.collision_detection()
        data_buffer.add(q, dq, collision)
        if collision:
            ui.root.after_idle(ui.force_update)
        time.sleep(0.01)  # 10ms定时

class BatchMotorUI(MotorUI):
    def __init__(self, motor, data_buffer):
        self.data_buffer = data_buffer
        super().__init__(motor)

    def update_ui(self):
        batch, collision = self.data_buffer.get_batch(100)
        if batch or collision:
            # 这里只显示最新一个点
            if batch:
                q, dq, col = batch[-1]
                self.label_q.config(text=f"角度: {q:.2f}")
                self.label_dq.config(text=f"角速度: {dq:.2f}")
                color = "red" if col else "green"
                self.canvas.itemconfig(self.signal_light, fill=color)
            elif collision:
                self.canvas.itemconfig(self.signal_light, fill="red")
        self.root.after(100, self.update_ui)

    def force_update(self):
        self.update_ui()

if __name__ == "__main__":
    data_buffer = DataBuffer()
    motor0 = Motor("/dev/ttyUSB0", 0)
    ui = BatchMotorUI(motor0, data_buffer)
    threading.Thread(target=motor_worker, args=(motor0, data_buffer, ui), daemon=True).start()
    ui.run()


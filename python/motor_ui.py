import tkinter as tk

class MotorUI:
    def __init__(self, motor):
        self.motor = motor
        self.root = tk.Tk()
        self.root.title("电机状态监控")
        self.root.geometry("300x200")

        self.label_q = tk.Label(self.root, text="角度: 0.0", font=("Arial", 16))
        self.label_q.pack(pady=10)

        self.label_dq = tk.Label(self.root, text="角速度: 0.0", font=("Arial", 16))
        self.label_dq.pack(pady=10)

        self.canvas = tk.Canvas(self.root, width=50, height=50)
        self.canvas.pack(pady=10)
        self.signal_light = self.canvas.create_oval(10, 10, 50, 50, fill="green")

        self.update_ui()

    def update_ui(self):
        # 采集最新数据
        self.motor.exchange_data()
        q = self.motor.data.q
        dq = self.motor.data.dq
        collision = self.motor.collision_detection()

        self.label_q.config(text=f"角度: {q:.2f}")
        self.label_dq.config(text=f"角速度: {dq:.2f}")
        color = "red" if collision else "green"
        self.canvas.itemconfig(self.signal_light, fill=color)

        # 100ms后再次刷新
        self.root.after(100, self.update_ui)

    def run(self):
        self.root.mainloop()
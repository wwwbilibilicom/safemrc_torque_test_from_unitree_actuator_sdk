#!/usr/bin/env python3
import sys
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
from matplotlib.gridspec import GridSpec
from scipy.signal import savgol_filter

def filter_data(data, window_length=100, polyorder=4):
    """使用Savitzky-Golay滤波器进行滤波"""
    return savgol_filter(data, window_length, polyorder)

def plot_data(filename):
    # 确保文件名包含.csv扩展名
    if not filename.endswith('.csv'):
        filename = filename + '.csv'
    
    # 如果文件名不包含路径，假设它在data目录下
    if not os.path.dirname(filename):
        filename = os.path.join('data', filename)
    
    # 读取CSV文件
    df = pd.read_csv(filename)
    
    # 对数据进行滤波处理
    df['a_Torque(Nm)'] = filter_data(df['a_Torque(Nm)'].values)
    df['Velocity(rad/s)'] = filter_data(df['Velocity(rad/s)'].values)
    df['Power(W)'] = filter_data(df['Power(W)'].values)
    
    # 创建图形
    plt.style.use('seaborn')
    
    # 创建一个大的图形窗口，包含所有子图
    fig = plt.figure(figsize=(15, 10))
    gs = GridSpec(3, 2, figure=fig)
    
    # 1. 绘制扭矩图
    ax1 = fig.add_subplot(gs[0, 0])
    ax1.plot(df['Time(s)'], df['d_Torque(Nm)'], label='Desired Torque', linestyle='--')
    ax1.plot(df['Time(s)'], df['a_Torque(Nm)'], label='Actual Torque (Filtered)')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Torque (Nm)')
    ax1.set_title('Torque vs Time')
    ax1.legend()
    ax1.grid(True)
    
    # 2. 绘制位置图
    ax2 = fig.add_subplot(gs[0, 1])
    ax2.plot(df['Time(s)'], df['Desired_Position(rad)'], label='Desired Position', linestyle='--')
    ax2.plot(df['Time(s)'], df['Position(rad)'], label='Actual Position')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Position (rad)')
    ax2.set_title('Position vs Time')
    ax2.legend()
    ax2.grid(True)
    
    # 3. 绘制扭矩-位置图
    ax3 = fig.add_subplot(gs[1, 0])
    ax3.plot(df['Position(rad)'], df['a_Torque(Nm)'], label='Torque vs Position (Filtered)')
    ax3.set_xlabel('Position (rad)')
    ax3.set_ylabel('Torque (Nm)')
    ax3.set_title('Torque vs Position')
    ax3.legend()
    ax3.grid(True)
    
    # 4. 绘制功率图
    ax4 = fig.add_subplot(gs[1, 1])
    ax4.plot(df['Time(s)'], df['Power(W)'], label='Power (Filtered)')
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Power (W)')
    ax4.set_title('Power vs Time')
    ax4.legend()
    ax4.grid(True)
    
    # 5. 绘制角速度图
    ax5 = fig.add_subplot(gs[2, :])
    ax5.plot(df['Time(s)'], df['Velocity(rad/s)'], label='Angular Velocity (Filtered)')
    ax5.set_xlabel('Time (s)')
    ax5.set_ylabel('Angular Velocity (rad/s)')
    ax5.set_title('Angular Velocity vs Time')
    ax5.legend()
    ax5.grid(True)
    
    # 调整子图之间的间距
    plt.tight_layout()
    
    # 显示图形
    plt.show()

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python3 plot_data.py <csv_filename>")
        sys.exit(1)
    
    filename = sys.argv[1]
    plot_data(filename) 
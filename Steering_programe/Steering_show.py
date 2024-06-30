import tkinter as tk
from tkinter import ttk
import pygame
import csv
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
from matplotlib.transforms import Transform
from matplotlib.ticker import FuncFormatter

# 初始化Pygame和方向盘
pygame.init()
pygame.mixer.init()
try:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
except Exception as e:
    print(f"没有外接方向盘: {e}")
    exit()

# 初始化CSV文件
csv_file = f'steering_data.csv'
with open(csv_file, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["Timestamp", "SteeringAngle"])


# 自定义非线性变换
class NonLinearScale(Transform):
    input_dims = output_dims = 1

    def transform_non_affine(self, values):
        values = np.asarray(values)
        return np.where((values > -0.25) & (values < 0.25),
                        values * 4,  # 放大-0.25到0.25区间
                        np.sign(values) * (np.abs(values) ** 0.5) * 2)  # 对其余部分进行开方变换以压缩

    def inverted(self):
        return InvertedNonLinearScale()


class InvertedNonLinearScale(Transform):
    input_dims = output_dims = 1

    def transform_non_affine(self, values):
        values = np.asarray(values)
        return np.where((values > -1) & (values < 1),
                        values / 4,  # 缩小-1到1区间
                        np.sign(values) * (np.abs(values) / 2) ** 2)  # 对其余部分进行平方变换


def non_linear_formatter(x, pos):
    return f'{x:.2f}'


# 初始化图表
plt.style.use('fivethirtyeight')
fig, ax = plt.subplots()
x_data, y_data = [], []
ln, = plt.plot([], [], 'b-')

# 应用自定义的非线性变换
ax.set_yscale('function', functions=(NonLinearScale().transform, NonLinearScale().inverted().transform))
ax.yaxis.set_major_formatter(FuncFormatter(non_linear_formatter))


def init():
    ax.set_xlim(0, 10)
    ax.set_ylim(-1, 1)
    return ln,


def update(frame):
    timestamp = time.time()

    # 获取方向盘转角数据
    pygame.event.pump()  # 刷新Pygame事件队列
    steering = joystick.get_axis(0)

    # 存储数据到CSV
    with open(csv_file, mode='a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow([timestamp, steering])

    # 更新绘图数据
    x_data.append(timestamp)
    y_data.append(steering)
    ln.set_data(x_data, y_data)

    # 动态调整x轴范围
    if len(x_data) > 10:
        ax.set_xlim(x_data[-10], x_data[-1])
    else:
        ax.set_xlim(x_data[0], x_data[-1])

    return ln,


def animate():
    ani = FuncAnimation(fig, update, frames=None, init_func=init, blit=True, interval=100, cache_frame_data=False)
    canvas.draw()


# 创建Tkinter界面
root = tk.Tk()
root.title("Real-time Steering Angle Data")

# 创建一个框架以包含图表
frame = ttk.Frame(root)
frame.pack(fill=tk.BOTH, expand=True)

# 将Matplotlib图表嵌入到Tkinter中
canvas = FigureCanvasTkAgg(fig, master=frame)
canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

# 在Tkinter主循环中调用动画函数
root.after(100, animate)

# 启动Tkinter主循环
root.mainloop()

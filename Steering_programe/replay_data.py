import tkinter as tk
from tkinter import ttk
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.transforms import Transform
from matplotlib.ticker import FuncFormatter
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np

# 读取CSV文件
data = pd.read_csv('steering_data.csv')

# 提取时间戳和方向盘转角数据
timestamps = data['Timestamp']
steering_angles = data['SteeringAngle']

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
ax.set_yscale('function', functions=(NonLinearScale().transform, NonLinearScale().inverted().transform))
ax.yaxis.set_major_formatter(FuncFormatter(non_linear_formatter))
ln, = plt.plot([], [], 'b-')

# 更新绘图数据
def update_plot(frame):
    ax.clear()
    ax.set_yscale('function', functions=(NonLinearScale().transform, NonLinearScale().inverted().transform))
    ax.yaxis.set_major_formatter(FuncFormatter(non_linear_formatter))
    ax.plot(timestamps[:frame+1], steering_angles[:frame+1], 'b-')
    ax.set_xlim(timestamps.iloc[0], timestamps.iloc[-1])
    ax.set_ylim(-1, 1)
    canvas.draw()

# 创建Tkinter界面
root = tk.Tk()
root.title("Steering Angle Visualization")

# 创建一个框架以包含图表
frame = ttk.Frame(root)
frame.pack(fill=tk.BOTH, expand=True)

# 将Matplotlib图表嵌入到Tkinter中
canvas = FigureCanvasTkAgg(fig, master=frame)
canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

# 创建一个滑动条
slider = tk.Scale(root, from_=0, to=len(timestamps)-1, orient=tk.HORIZONTAL, length=800, command=lambda x: update_plot(int(x)))
slider.pack(fill=tk.X, padx=10, pady=10)

# 启动Tkinter主循环
root.mainloop()

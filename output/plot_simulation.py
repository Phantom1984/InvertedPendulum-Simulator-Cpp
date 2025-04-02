# -*- coding: utf-8 -*-
# plot_simulation.py
import matplotlib.pyplot as plt
import pandas as pd

data = pd.read_csv("simulation.csv")

# 创建两个图形窗口
fig1 = plt.figure("Combined Plot", figsize=(12, 6))
fig2 = plt.figure("Separated Plots", figsize=(12, 8))

# ----------------------
# 第一个图形：合并显示
# ----------------------
ax1 = fig1.add_subplot(111)
ax1.plot(data['time'], data['theta'], label='Angle (rad)')
ax1.plot(data['time'], data['x'], label='Cart Position (m)')
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Value')
ax1.legend(loc='upper right')  
ax1.grid(True)
fig1.tight_layout()

# ----------------------
# 第二个图形：分开显示
# ----------------------
# 上方角度图
ax2_top = fig2.add_subplot(211)
ax2_top.plot(data['time'], data['theta'], color='blue', label='Angle (rad)')
ax2_top.set_ylabel('Angle (rad)')
ax2_top.legend(loc='upper right') 
ax2_top.grid(True)

# 下方位置图
ax2_bottom = fig2.add_subplot(212)
ax2_bottom.plot(data['time'], data['x'], color='orange', label='Cart Position (m)')
ax2_bottom.set_xlabel('Time (s)')
ax2_bottom.set_ylabel('Cart Position (m)')
ax2_bottom.legend(loc='upper right')  
ax2_bottom.grid(True)

fig2.tight_layout()

# 同时显示两个窗口
plt.show()

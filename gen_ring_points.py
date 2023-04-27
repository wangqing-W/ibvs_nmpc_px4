import numpy as np
import matplotlib.pyplot as plt

x0, y0 = 0, 0  # 原点
r = 0.6        # 半径
angles = np.arange(0, 2*np.pi, np.pi/9)  # 角度列表
xs = x0 + r * np.cos(angles)
ys = y0 + r * np.sin(angles)

# 添加最后一个点
x_final, y_final = x0 + r * np.cos(2*np.pi), y0 + r * np.sin(2*np.pi)
xs = np.append(xs, x_final)
ys = np.append(ys, y_final)

plt.plot(xs, ys)
plt.show()

for x, y in zip(xs, ys):
    print(f'<point>{x} {y}</point>')
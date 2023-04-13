import numpy
import matplotlib.pyplot as plt
from math import cos, sin, pi

xs = []
ys = []
x0, y0 = 0, 0  # origin
r = 0.6        # radius
for angle in range(0, 360, 20):
    x = x0 + r * cos(angle * pi / 180)
    y = y0 + r * sin(angle * pi / 180)
    xs.append(x)
    ys.append(y)

xs.append(x0 + r * cos(360 * pi / 180))
ys.append(y0 + r * sin(360 * pi / 180))

# plt.plot(xs, ys)
# plt.show()

for i in range(len(xs)):
  print ('<point>' + str(xs[i]) + ' ' + str(ys[i]) + '</point>')
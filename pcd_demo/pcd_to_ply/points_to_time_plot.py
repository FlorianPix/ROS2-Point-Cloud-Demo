import matplotlib.pyplot as plt
import numpy as np
from math import pow

# takes a log of number of points and Durations and creates a plot from it

f = open("/home/florianpix/git_repos/iwu_thermography/doc/data/pcd_callback_time.csv", "r")
number_of_points_list = []
nanoseconds_list = []
for x in f:
    x = x.replace('[pcd_to_ply_pause_node-1] [INFO] [', '')
    x = x.replace('] [pcd_to_ply_pause_node]:', '')
    _, x = x.split(' ')
    x = x.replace('/n', '')
    number_of_points = 0
    nanoseconds = 0
    try:
        number_of_points = int(x)
        number_of_points_list.append(number_of_points)
    except ValueError:
        x = x.replace('Duration(nanoseconds=', '')
        x = x.replace(')', '')
        x = x.replace('\n', '')
        nanoseconds = int(x)
        # seconds = nanoseconds * pow(10, -9)
        nanoseconds_list.append(nanoseconds)

pts = list(zip(number_of_points_list, nanoseconds_list))
pts = np.array(pts)
m, n = np.polyfit(pts[0], pts[1], 1)
n += 0.44 * pow(10, 9)
x = np.arange(0, 2 * pow(10, 7), pow(10, 5))
linear_approx = m * x + n
fig, ax = plt.subplots()
pts = np.array(pts)
ax.plot(pts[:, 0], pts[:, 1])
ax.plot(x, linear_approx)
ax.grid()
plt.show()

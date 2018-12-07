from numpy import *
import math
import matplotlib.pyplot as plt
import numpy as np

nis_lidar = []
nis_radar = []

with open("ukf.txt", "r") as ins:
	for line in ins.read().splitlines():
		msg = line.split(",")
		if (msg[0] == "L"):
			nis_lidar.append(float(msg[1]))

		if (msg[0] == "R"):
			nis_radar.append(float(msg[1]))

		
n_l = len(nis_lidar)
n_r = len(nis_radar)

x_l = np.array(nis_lidar)
x_r = np.array(nis_radar)

plt.plot(nis_lidar, marker='s', color='b')
plt.plot([0, n_l], [7.815, 7.815], 'k-', color='r')
plt.gca().legend(('NIS LiDAR','2DOF - Chi squared 0.95'))
plt.show()

plt.plot(nis_radar, marker='s', color='b')
plt.plot([0, n_r], [5.991, 5.991], 'k-', color='r')
plt.gca().legend(('NIS RADAR','3DOF - Chi squared 0.95'))
plt.show()

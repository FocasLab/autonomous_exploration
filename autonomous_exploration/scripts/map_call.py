#!/usr/bin/env python3

# ros includes
import rospy
from nav_msgs.msg import OccupancyGrid

# stl includes
import numpy as np
import matplotlib.pyplot as plt

mapData = OccupancyGrid()

def mapDataCallback(msg):
	global mapData
	mapData = msg


def main():
	rospy.init_node('test_map_data')

	rospy.Subscriber('/map', OccupancyGrid, mapDataCallback)

	while(len(mapData.data) < 1):
		pass

	height = mapData.info.height
	width = mapData.info.width

	maps = np.zeros((40, 200))

	count_i = 0
	for i in range(200, 240):
		count_j = 0
		for j in range(100, 200):
			idx = width * i + j
			if mapData.data[idx] > 0:
				maps[count_i][count_j] = 1
				# print('{}, '.format(1), end='')
			else:
				# print('{}, '.format(mapData.data[idx]),  end='')
				maps[count_i][count_j] = mapData.data[idx]
			count_j += 1
		count_i += 1

	new_map = np.array(mapData.data).reshape(height, width)
	print(maps.shape)
	print(new_map.shape)

	# Visualizing the test function
	X1 = np.linspace(0, 384, 384)
	X2 = np.linspace(0, 384, 384)
	X1, X2 = np.meshgrid(X1, X2)
	X = np.hstack((X1.reshape(-1,1), X2.reshape(-1,1)))

	fig, ax = plt.subplots(figsize=(7,5))

	cont = ax.contourf(X1, X2, new_map, levels=25, cmap='GnBu');
	ax.set_xlabel(r'$x_1$', fontsize=15)
	ax.set_ylabel(r'$x_2$', fontsize=15)
	ax.set_title('Obs', fontsize=15)
	fig.colorbar(cont, ax=ax)
	plt.tight_layout()

	plt.show()

	# rospy.spin()


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
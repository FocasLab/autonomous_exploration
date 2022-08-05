#!/usr/bin/env python3

# ros includes
import rospy
from nav_msgs.msg import OccupancyGrid

# stl includes
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import ticker, cm

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

	maps = np.zeros((height, width))

	for i in range(160, 241):
		for j in range(160, 241):
			idx = width * i + j
			if mapData.data[idx] > 0:
				maps[i][j] = 1
				print('{}, '.format(1), end='')
			else:
				print('{}, '.format(mapData.data[idx]),  end='')
				maps[i][j] = mapData.data[idx]

	# rospy.spin()


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
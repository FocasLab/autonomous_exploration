#!/usr/bin/env python3

# ros includes
import rospy
from nav_msgs.msg import OccupancyGrid

import numpy as np
import math
import time

begin = time.time()
# filename = 'mapgazho.txt' # name of latest /map file

def get_frontiers(maps, width, height, target_window=6, boundary_window=1):
	# Getting the map grid data from the ./map topic
	visited_targets = [] #To store all the visited targets
	target_window = 6 # Can vary the target size based on the size of the bot
	boundary_window = 1 # Can vary depending on how far the unexplored area can be from the target area
	control_op = 1 # needs to be changes, currently indicates if the controller has been able to plan the path to the target

	# Chaning the obtained grid to a 2D matrix
	maps = np.array(maps).reshape(width, height)

	# Dividing the map into 4 submaps and obtaining their sums
	sums = [] # Stores the sums of each submap
	s = len(maps[0]) # Length of the map
	submaps = [] # Stores the submaps
	for i in range(2):
		submaps.append(maps[len(maps)-1][i*s//2:(i+1)*s//2, 0:s//2]) # Dividing the map in half horizontally
		sums.append(sum(sum(submaps[len(submaps)-1]))) # Getting the sum
		submaps.append(maps[len(maps)-1][i*s//2:(i+1)*s//2, s//2:s])
		sums.append(sum(sum(submaps[len(submaps)-1])))

	print(sums)
	sum_iter = sums # Storing the sums in another list to iterate repeatedly

	while sum(sum_iter)!=-399996: # terminating condition
		most_explored = sum_iter.index(max(sum_iter)) # Finding the most explored submap
		ind = 0
		print("Most explored target:", most_explored + 1)
		indices = [] # Stores the indices of all possible targets

		# Getting the indices of all the zeros in the most explored submap
		if any(0 in n for n in submaps[most_explored]):
			indices = np.array(np.argwhere(submaps[most_explored] == 0))

		def neighbors(radius, row_number, column_number):
			if row_number < len(submaps[most_explored]) - radius and column_number < len(submaps[most_explored]) - radius:
				return submaps[most_explored][row_number:row_number + radius, column_number:column_number + radius]
			elif row_number >= len(submaps[most_explored]) - radius and column_number < len(submaps[most_explored]) - radius:
				return submaps[most_explored][row_number - radius:row_number, column_number:column_number + radius]
			elif row_number < len(submaps[most_explored]) - radius and column_number >= len(submaps[most_explored]) - radius:
				return submaps[most_explored][row_number:row_number + radius, column_number - radius:column_number]
			else:
				return submaps[most_explored][row_number - radius:row_number, column_number - radius:column_number]

		# Checking the surrounding points (surrounding area) of each index to find boundary points
		def boundary(window, row, column):
			return submaps[most_explored][row - window:row + window + 1, column - window:column + window + 1]

		# Function to obtain all the possible points that can be set as targets
		def possiblepoints(indices, b_window):
			neighborhood = [] # Stores the possible target areas
			boundary_points = [] # To store all the boundary points only
			boundary_area = [] # Store the boundary area of ecach index
			ind = [] # Temporarily stores the possible boundary points.
			# Store all the boundary areas of the index in a list
			for i in range(len(indices)):
				r, c = indices[i] # Stores the x and y coordinate of the indices
				if r > 0 and r < len(submaps[most_explored]) - boundary_window - 1 and c > 0 and c < len(submaps[most_explored]) - boundary_window - 1:
					boundary_area.append(boundary(boundary_window, r, c))
					ind.append(indices[i])
				else:
					pass
			indices = np.array(ind)

			# Store all the boundary points as long as there are -1s in the vicinity
			for i in range(len(boundary_area)):
				r, c = indices[i]
				if not any(1 in n for n in boundary_area[i]):
					if any(-1 in n for n in boundary_area[i]):
						neighborhood.append(neighbors(target_window, r, c))
						boundary_points.append(indices[i])

			boundary_points = np.array(boundary_points)
			target_places = [] # Stores all the possible target areas
			id = [] # Stores all the possible targets (boundary points)

			# Store all the possible targets (boundary points) as long there are no 1s or -1s in the vicinity
			for i in range(len(neighborhood)):
				if not any(1 in n for n in neighborhood[i]):
					if not any(-1 in n for n in neighborhood[i]):
						target_places.append(neighborhood[i])
						id.append(boundary_points[i])

			target_places = np.array(target_places)
			boundary_points = np.array(id)
			return boundary_points, target_places

		# Function to get the coordinates of the possible targets
		def coordinates(point):
			r, c = point
			#target_place = target_places[0]
			target = [0, 0] # Set initial value of target to (0, 0)
			if most_explored < 2:
				target[0] = r
				if most_explored > 0: target[1] = s // 2 + c
				else: target[1] = c
			else:
				target[0] = s // 2 + r
				if most_explored > 2: target[1] = s // 2 + c
				else: target[1] = c
			return target

		# Function to store all the targets points
		def Targets(boundary_points):
			targets = []
			for i in range(len(boundary_points)):
				t = coordinates(boundary_points[i])
				targets.append([(t[0], t[1]), (t[0] + target_window, t[1] + target_window)])
			print("Possible Targets:", (targets))
			print("------------------------------------------------------------------------------------------------------------------------")
			print("Target:", targets[0])
			print("Target Place \n", target_places[0])
			print("------------------------------------------------------------------------------------------------------------------------")
			return targets

		# Only if there are zeros in the current submap continue
		if len(indices) != 0:
			boundary_points, target_places = possiblepoints(indices, boundary_window) # Get all the possible boundary points
			print("No. of possible targets:", len(boundary_points))
			print("------------------------------------------------------------------------------------------------------------------------")
			#targets = []

			# Only if there are possible points continue
			if len(boundary_points) != 0:
				targets = Targets(boundary_points)
				end = time.time()
				print("Time taken:", (end - begin))

				# If the controller isn't able to plan a path to the given targets then increase boundary window and check for the new targets
				while control_op == 0:
					boundary_window += 1
					boundary_points, target_places = possiblepoints(indices, boundary_window) # Get all the possible boundary points
					print("No. of possible targets:", len(boundary_points))
					print("------------------------------------------------------------------------------------------------------------------------")
					if len(boundary_points) != 0:
						targets = Targets(boundary_points)
						end = time.time()
						print("Time taken:", (end - begin))

				# break
				return targets

			# If no possible targets exist then replace the sum of the current submap with a least negative no. and continue
			else:
				sum_iter[most_explored] = -99999 # This submap is considered as explored
				print(sum_iter)

		# If there are no 0s found then replace the sum of the current submap with a least negative no. and continue to find the next max explored submap
		else:
			sum_iter[most_explored] = -99999
			print(sum_iter)

	print("Done")


mapData = OccupancyGrid()

def mapDataCallback(msg):
	global mapData
	mapData = msg


def main():
	rospy.init_node('get_frontiers')

	rospy.Subscriber('/map', OccupancyGrid, mapDataCallback)

	while(len(mapData.data) < 1):
		pass

	target_data = get_frontiers(mapData.data, mapData.info.width, mapData.info.height)

	print(target_data)

	rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
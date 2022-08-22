#!/usr/bin/env python3

# ros includes
import rospy
import actionlib
from autonomous_exploration.msg import autoExplAction
from autonomous_exploration.msg import autoExplGoal
from autonomous_exploration.msg import Target
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Empty, EmptyResponse

# other
import numpy as np
import threading
from typing import final


class targetFinder:
	"""docstring for targetFinder"""
	def __init__(self, resolution=0.05, target_window=8, clearance=0.2, frontier_clearance=4, robot_dimensions=[0.2, 0.2]):

		assert isinstance(target_window, (int)), "Target window must be integer, Recieved type %r" % type(target_window).__name__
		assert isinstance(frontier_clearance, (int)), "Frontier clearance must be integer, Recieved type %r" % type(frontier_clearance).__name__

		assert round(robot_dimensions[0] * robot_dimensions[1], 2) <= round((resolution * target_window) ** 2, 2), "Robot size(area) must not be bigger than target window(area), given robot size is %r m*m, while target window is %r m*m" %(robot_dimensions[0] * robot_dimensions[1], (resolution * target_window) ** 2)
		assert target_window >= 9, "Target window must be greater than 9 (0.4*0.4 m*m), Recieved %r" % target_window
		assert target_window % 2 != 0, "Target window must be an odd number, Recieved %r" % target_window
		assert clearance > resolution, "Clearance must not be less than resolution, Recieved clearance is %r, and resolution is %r" % (clearance, resolution)

		self.resolution = resolution					# Size of each grid in meters
		self.robot_max_length = robot_dimensions[0]		# Manimum length of the robot in meters
		self.robot_max_width = robot_dimensions[1]		# Miximum width of the robot in meters
		self.target_window = target_window				# size of the target window in pixel (to convert meters => target_window * resolution), note that it is a square and must always be an odd number
		self.clearance = clearance						# The inflation of the robot for safety so it is not very close to the frontier or the obstacle
		self.frontier_clearance = frontier_clearance	# The amount of spacing between robot and frontier, it must always be even
	
	def get_targets(self, maps, width, height):
		"""
		This function finds out all potential targets in the map
		If a point is near to a frontier while also being far away enough from a wall, then a region around the point that is far enough from the frontier itself 
		is chosen at the potential target

		Parameters
			The function only needs a map

		Returns
			It returns the start and end points of all the regions that can be chosen as targets
		"""

		maps = maps[0]
		potential_targets = []
		
		clearance = self.clearance / self.resolution
		cut_off = clearance // 2
		frontier_window = self.target_window + self.frontier_clearance + clearance
		
		for i in range(width):
			for j in range(height):
				if maps[i][j] == 0:
					[neighbor_map, x_s, y_s, x_e, y_e] = self.neighbors(frontier_window - 1, i, j, maps, width, height)
					if not any(1 in n for n in neighbor_map):  # Check if there are any obstacles nearby
						if any(-1 in n for n in neighbor_map):  # Check if there are any frontiers nearby
							[target_map, x_start, y_start, x_end, y_end] = self.neighbors(self.target_window + clearance - 1, i, j, maps, width ,height)
							if not any(-1 in n for n in target_map):  # Make sure there are no frontiers in the target
								potential_targets.append([[x_start + cut_off, y_start + cut_off], [x_end - cut_off, y_end - cut_off]])

		return potential_targets

	
	def neighbors(self, radius, row_number, column_number, submaps, width, height):
		"""
		This function finds (radius/2) number of neighbors of given point on all four sides

		Parameters
			radius: how many neighbors do you want to obtain either horizontally or vertically
			row_number: x coordinate of the point
			column_number: y coordinate of the point
			submaps: The part of the map within which we find the neighbors

		Returns
			The function returns the neighbors and the x and y coordinates of the starting and ending element of the neighbors array with respect to the submap coordinate 
		"""

		radius = int(radius // 2)

		# If the selected has neighbors all around it within the map
		if row_number < width - radius and column_number < height - radius and row_number > radius and column_number > radius: 
			return submaps[row_number - radius : row_number + radius, column_number - radius : column_number + radius], row_number - radius, column_number - radius, row_number + radius, column_number + radius
		else:
			return [[]], row_number - radius, column_number - radius, row_number + radius, column_number + radius

	
	def split_targets(self, final_targets, len_of_map, height_of_map):
		"""
		This function splits the target into four based on which quadrant they are located in

		Parameters
			final_targets: The targets that have been found in the entire map
			len_of_map: The length of the map
			height_of_map: The height of the map
		
		Return
			split_quads: The list of points that have been segregated based on the quadrant they are in
		"""

		first_quad_targets = []
		second_quad_targets = []
		third_quad_targets = []
		fourth_quad_targets = []
		split_quads = []
		
		half_len = len_of_map // 2
		half_height = height_of_map // 2
		
		for i in range(len(final_targets)):
			if final_targets[i][0][0] <= half_len and final_targets[i][0][0] >= 0 and final_targets[i][0][1] <= half_height and final_targets[i][0][1] >= 0 and final_targets[i][1][0] <= half_len and final_targets[i][1][0] >= 0 and final_targets[i][1][1] <= half_height and final_targets[i][1][1] >= 0:
			### If the target point is in the first quadrant which is between 0 to halfway in x and 0 to halfway in y
				first_quad_targets.append(final_targets[i])
			if final_targets[i][0][0] <= half_len and final_targets[i][0][0] >= 0 and final_targets[i][0][1] >= half_height and final_targets[i][0][1] <= height_of_map and final_targets[i][1][0] <= half_len and final_targets[i][1][0] >= 0 and final_targets[i][1][1] >= half_height and final_targets[i][1][1] <= height_of_map:
			### If the target point is in the second quadrant which is betwwen 0 to halfway in x and halfway to end in y
				second_quad_targets.append(final_targets[i])
			if final_targets[i][0][0] >= half_len and final_targets[i][0][0] <= len_of_map and final_targets[i][0][1] <= half_height and final_targets[i][0][1] >= 0 and final_targets[i][1][0] >= half_len and final_targets[i][1][0] <= len_of_map and final_targets[i][1][1] <= half_height and final_targets[i][1][1] >= 0:
			### If the target point is in the third quadrant which is betwwen halfway to end of x and 0 to halfway in y
				third_quad_targets.append(final_targets[i])
			if final_targets[i][0][0] >= half_len and final_targets[i][0][0] <= len_of_map and final_targets[i][0][1] >= half_height and final_targets[i][0][1] <= height_of_map and final_targets[i][1][0] >= half_len and final_targets[i][1][0] <= len_of_map and final_targets[i][1][1] >= half_height and final_targets[i][1][1] <= height_of_map:
			### If the target point is in the fourth quadrant which is betwwen halfway to end of x and halfway to end in y
				fourth_quad_targets.append(final_targets[i])
		
		first_quad_targets.append([])
		second_quad_targets.append([])
		third_quad_targets.append([])
		fourth_quad_targets.append([])
		
		split_quads.append(first_quad_targets)
		split_quads.append(second_quad_targets)
		split_quads.append(third_quad_targets)
		split_quads.append(fourth_quad_targets)
		
		return split_quads

	
	def split_map(self, maps, width, height):
		"""
		This function splits the map into four parts

		Parameter
			maps: The map that has to be split

		Returns
			submaps: The four submaps sent back within one array
			sums: The negative of the number of unexplored gridpoints
		"""

		sums = [] # Stores the sums of each submap
		submaps = [] # Stores the submaps
		
		for i in range(2):
			submaps.append(maps[len(maps) - 1][i * width // 2 : (i + 1) * width // 2, 0 : height // 2]) # Dividing the map in half horizontally
			submaps.append(maps[len(maps) - 1][i * width // 2 : (i + 1) * width // 2, height // 2 : height])
	   
		# Stores the sum of all unexplored areas
		for i in range(4):
			t_sum = 0
			for j in range(len(submaps[i])):
				for k in range(len(submaps[i][0])):
					if (submaps[i][j][k] == -1):
						t_sum += 1
			sums.append(-t_sum)
		
		return submaps, sums
	
	
	def rank_targets(self, maps, segregated_target, width, height):
		"""
		This function ranks the targets based on the area of unexplored region. More the unexplored area, higher the rank

		Parameters
			maps: The map within which the unexplored areas are present
			segregated_target: targets that have been segregated based on their quadrant of location

		Return
			rank_index: This is a list of the indices ranked regions in the ranked order
		"""

		rank_index = []
		[submaps, sums] = self.split_map(maps, width, height)
		max_indices = np.argsort(sums)
		
		for i in range(4):
			# If the quadrant that has more unexplored region does not have any potential targets, they are not considered
			if (segregated_target[max_indices[i]] == [[]]):
				pass
			else:
				rank_index.append(max_indices[i])

		return rank_index

	
	def get_best_targets(self, segregated_targets, max_indices):
		"""
		This function returns the middle portion of the frontier in the ranked region. The first element of best_target is best region

		Parameters
			segregated_target: targets that have been segregated based on their quadrant of location
			max_indices: The indices of the maximum unexplored region submap to the minimum unexplored submap

		Returns
			best_targets: Gives the targets at the frontier. First element is the best target
		"""

		best_targets = []

		for i in range(len(max_indices)):
			best_targets.append(segregated_targets[max_indices[i]][len(segregated_targets[max_indices[i]]) // 2])
		
		return best_targets


class scotsActionClient:
	"""docstring for scotsActionClient"""
	def __init__(self):

		self.total_systhessis_time = 0
		self.total_completion_time = 0

		self._ac = actionlib.SimpleActionClient("/scots", autoExplAction)
		self._ac.wait_for_server()

		rospy.loginfo("Action Server is Up, starting to send goals.")

	# Function to send Goals to Action Servers
	def send_goal(self, targets):
		
		# Create Goal message for Simple Action Server
		goal = autoExplGoal(targets=targets)
		
		'''
			* done_cb is set to the function pointer of the function which should be called once 
				the Goal is processed by the Simple Action Server.

			* feedback_cb is set to the function pointer of the function which should be called while
				the goal is being processed by the Simple Action Server.
		''' 
		self._ac.send_goal(goal, done_cb=self.done_callback, feedback_cb=self.feedback_callback)
		
		rospy.loginfo("Goal has been sent.")

	def done_callback(self, status, result):
		self.total_systhessis_time += result.synthesis_time
		self.total_completion_time += result.completion_time
		
		rospy.loginfo("Target id, {}".format(result.target_id))
		rospy.loginfo("Synthesis Time, {}".format(result.synthesis_time))
		rospy.loginfo("Completion Time,  {}".format(result.completion_time))


	def feedback_callback(self, feedback):
		rospy.loginfo("Current Pose: ({}, {}, {})".format(round(feedback.curr_pose.x, 2), round(feedback.curr_pose.y, 2), round(feedback.curr_pose.theta, 2)))

class mapData:
	"""docstring for mapData"""
	def __init__(self):
		self.width = 0
		self.height = 0
		self.resolution = 0
		self.maps = list()

		self.map_topic_name = "/map"
		self.map_sub_handle = rospy.Subscriber(self.map_topic_name, OccupancyGrid, self.mapDataCallback)

		rospy.loginfo("Waiting for data on /map topic..")
		rospy.wait_for_message(self.map_topic_name, OccupancyGrid, timeout=10)

	def mapDataCallback(self, msg):
		self.width = msg.info.width
		self.height = msg.info.height
		self.resolution = msg.info.resolution

		map_image = np.zeros((self.height, self.width, 1), dtype="int8")
		for i in range(0, self.height):
			for j in range(0, self.width):
				if msg.data[i * self.width + j] > 0 :
					map_image[i][j] = 1
				else:
					map_image[i][j] = int(msg.data[i * self.width + j])

		self.maps = list(map_image.T)

	def getMap(self):
		return self.maps

	def getMapDimensions(self):
		return self.width, self.height, self.resolution


if __name__ == '__main__':

	rospy.init_node("target_finder")

	if_send_new_goal = True

	def send_new_goal(req):
		global if_send_new_goal
		rospy.loginfo("Got new goal request.")
		if_send_new_goal = True
		return EmptyResponse()

	new_goal_service_name = "/send_new_goal"
	new_goal_service = rospy.Service(new_goal_service_name, Empty, send_new_goal)

	mapdata = mapData()

	action_client = scotsActionClient()
	target_finder = targetFinder(resolution=0.05, target_window=9, clearance=0.2, frontier_clearance=2, robot_dimensions=[0.2, 0.2])

	rate = rospy.Rate(1)

	try:
		while not rospy.is_shutdown():
			maps = mapdata.getMap()
			width, height, resolution = mapdata.getMapDimensions()

			targets = []

			all_targets = target_finder.get_targets(maps, width, height)
			segregated_targets = target_finder.split_targets(all_targets, width, height)
			max_indices = target_finder.rank_targets(maps, segregated_targets, width, height)
			safe_targets = target_finder.get_best_targets(segregated_targets, max_indices)

			# print(safe_targets)

			if(len(safe_targets) > 0):
				for i in range(len(safe_targets)):
					if not any(safe_targets[i]):
						continue
					tr = Target()
					tr.id = i
					for j in range(len(safe_targets[i])):
						tr.points.append(round(safe_targets[i][j][0] * 0.05, 2))
					for j in range(len(safe_targets[i])):
						tr.points.append(round(safe_targets[i][j][1] * 0.05, 2))
					targets.append(tr)

				if(if_send_new_goal):
					print(targets)
					action_client.send_goal(targets)
					# action_client._ac.wait_for_result()
					if_send_new_goal = False
			else:
				print("No targets..")

			rate.sleep()
	except KeyboardInterrupt:
		pass
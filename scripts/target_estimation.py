#!/usr/bin/env python3
from typing import final
import numpy as np
import rospy
from nav_msgs.msg import OccupancyGrid,MapMetaData
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3 
from std_msgs.msg import Header, ColorRGBA


maps = []
robot_max_dim = 0.2              # Maximum dimension of the robot in meters
robot_min_dim = 0.2              # Minimum dimension of the robot in meters
target_window = 7                # size of the target in meters, note that it is a square and must always be an odd number
grid_size = 0.05                 # Size of each grid in meters
sensor_range = 8                 # Max range of the lidar in meters
clearance = 0.2                  # The inflation of the robot for safety so it is not very close to the frontier or the obstacle
frontier_clearance = 6           # The amount of spacing between robot and frontier, it must always be even


def map_callback(data):
    m = MapMetaData()
    width = data.info.width
    height = data.info.height
    resolution = data.info.resolution
    length = len(data.data)
    
    rospy.loginfo('width: {}'.format(width))
    rospy.loginfo('height: {}'.format(height))
    rospy.loginfo('resolution: {}'.format(resolution))
    rospy.loginfo('length: {}'.format(length))

    map_image = np.zeros((height,width, 1), dtype = "int8")
    for i in range(0,height):
        for j in range(0,width):
            if data.data[(i)*width+j] > 0 :
                map_image[i][j] = 1
            else:
                map_image[i][j] = int(data.data[(i)*width+j])

    # print(list(map_image.T))
    maps = list(map_image.T)

    final_targets = get_targets(maps, width,  height)
    segregated_target = split_targets(final_targets, width, height)
    max_indices = rank_targets(maps, segregated_target, width, height)
    final_target = get_best_targets(segregated_target, max_indices)
    print(final_targets)
    # print(max_indices)
    # print(segregated_target)
    print(final_target)
    pub_marker(final_target)
        


def pub_marker(final_target):
    points = Marker()
    p = Point()

    points.header.frame_id = "origin"
    points.header.stamp = rospy.Time.now()

    # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
    points.type = Marker.POINTS
    points.id = 0

    # Set the scale of the marker
    points.scale.x = 0.05
    points.scale.y = 0.05
    points.scale.z = 0.05

    # Set the color
    points.color.r = 0
    points.color.g = 1.0
    points.color.b = 0.0
    points.color.a = 1.0

    points.points = []
    for i in range(len(final_target)):
        for j in range(len(final_target[0])):
            points.points.append(Point(final_target[i][j][0]*0.05,final_target[i][j][1]*0.05,0))

    marker_pub.publish(points)

def calc_target_window():
    """
    This function checks if the target window given is adequate for safety of the robot and for obtaining sufficient winning domain

    Parameters
        All parameters are set as global variables

    Returns
        The function prints to the console
    """
    if robot_max_dim*robot_min_dim >= (grid_size*target_window)^2:                                             # If the robot is bigger than the target window
        print("The robot is too big. Increase the target window")
    elif target_window<7:                                                                                      # If the target window is smaller than 0.35mX0.35m
        print("The size of the target window is too small. Please increase the target window size")

def get_targets(maps, s, h):
    """
    This function finds out all potential targets in the map
    If a point is near to a frontier while also being far away enough from a wall, then a region around the point that is far enough from the frontier itself 
    is chosen at the potential target

    Parameters
        The function only needs a map

    Returns
        It returns the start and end points of all the regions that can be chosen as targets
    """
    clearance = 0.2
    maps = maps[0]
    if clearance < grid_size:
        print("Clearance is smaller than the grid size, make it bigger and divisible by the grid size")
    elif target_window//2 == 0:
        print("Target window size must be an odd number")
    else:
        clearance = clearance/grid_size
        cut_off = clearance//2
        potential_targets = []
        frontier_window = target_window+frontier_clearance+clearance
        for i in range(s):
            for j in range(h):
                if maps[i][j] == 0:
                    [neighbor_map, x_s, y_s, x_e, y_e] = neighbors(frontier_window-1, i, j, maps, s, h)
                    if not any(1 in n for n in neighbor_map):                                                   # Check if there are any obstacles nearby
                        if any(-1 in n for n in neighbor_map):                                                  # Check if there are any frontiers nearby
                            [target_map, x_start, y_start,x_end, y_end] = neighbors(target_window+clearance-1, i, j, maps,s ,h)
                            if not any(-1 in n for n in target_map):                                            # Make sure there are no frontiers in the target
                                potential_targets.append([[x_start+cut_off, y_start+cut_off],[x_end-cut_off, y_end-cut_off]])
    return potential_targets


# Function to find the nearest radius number of neighbors
def neighbors(radius, row_number, column_number, submaps, s, h):
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
    radius = int(radius//2)
    if row_number<s-radius and column_number<h-radius and row_number>radius and column_number>radius:                        # If the selected has neighbors all around it within the map
        return submaps[row_number-radius:row_number+radius, column_number-radius:column_number+radius], row_number-radius, column_number-radius, row_number+radius, column_number+radius
    else:
        return [[]], row_number-radius, column_number-radius, row_number+radius, column_number+radius

def rank_targets(maps, segregated_target, s, h):
    """
    This function ranks the targets based on the area of unexplored region. More the unexplored area, higher the rank

    Parameters
        maps: The map within which the unexplored areas are present
        segregated_target: targets that have been segregated based on their quadrant of location

    Return
        rank_index: This is a list of the indices ranked regions in the ranked order
    """
    no_of_iter = len(maps[0])//sensor_range
    [submaps, sums] = splitMap(maps, s, h)
    max_indices = np.argsort(sums)
    rank_index = []
    #print(sums)
    # print(segregated_target[3])
    for i in range(4):
        if (segregated_target[max_indices[i]] == [[]]):         # If the quadrant that has more unexplored region does not have any potential targets, they are not considered
            pass
        else:
            rank_index.append(max_indices[i])
    return rank_index

def get_best_targets(segregated_targets, max_indices):
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
        best_targets.append(segregated_targets[max_indices[i]][len(segregated_targets[max_indices[i]])//2])
    return best_targets

def split_targets(final_targets, len_of_map, height_of_map):
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
    half_len = len_of_map//2
    half_height = height_of_map//2
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

def splitMap(maps, s, h):
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
        submaps.append(maps[len(maps)-1][i*s//2:(i+1)*s//2, 0:h//2]) # Dividing the map in half horizontally
        submaps.append(maps[len(maps)-1][i*s//2:(i+1)*s//2, h//2:h])
   
    # Stores the sum of all unexplored areas
    for i in range(4):
        sum = 0
        for j in range(len(submaps[i])):
            for k in range(len(submaps[i][0])):
                if (submaps[i][j][k]==-1):
                    sum += 1
        sums.append(-sum)
    return submaps, sums

if __name__ == '__main__':
    rospy.init_node("target_finder")
    rospy.Subscriber("map",OccupancyGrid,map_callback)
    marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size = 10)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rate.sleep()
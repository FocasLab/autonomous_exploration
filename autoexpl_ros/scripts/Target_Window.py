#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker


import numpy as np
import cv2 as cv
from scipy.ndimage import convolve
import math, cmath

class FrontierExploration():

    def __init__(self):
        # Init Node
        rospy.init_node('Frontier_Exploration', anonymous=False)

        #Subscribed to the Map Topic
        self.map_grid_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        #Frontier Target Marker Publisher
        self.marker_pub=rospy.Publisher('frontier_target_marker', Marker, queue_size=10)
        
        #Wait for 2 seconds every time
        self.rate = rospy.Rate(2)

        # Kernel for smoothning
        kernel_size = 3
        self.kernel = np.ones((kernel_size, kernel_size),np.uint8)/kernel_size**2
        
        #Parameters for Canny Edge Detector
        self.minVal, self.maxVal = 195, 210

        #Frontier Target Window Size
        self.size_frontier_target_window = 11
        self.floor_half_size_frontier_target_window = self.size_frontier_target_window//2

        #Gaussian Kernel used for Theta Function
        self.gaussian_kernel_output=self.gaussian_kernel(size=3, sigma=1)

        # Map resolution
        self.map_resolution=0.05

        # Matrix of window size of elements 207 and 0 only
        self.check_0=np.zeros((self.size_frontier_target_window, self.size_frontier_target_window))
        self.check_207=np.ones((self.size_frontier_target_window, self.size_frontier_target_window))*207

        # Limit on max explore
        self.max_explore_from_point=100

        while not rospy.is_shutdown():
            # Initiated with sleep so it can subscribed to map in the mean time.
            self.rate.sleep()

            self.points_before_shifting=[]
            #Initiated an empty list for shifted frontier target points
            self.shifted_frontier_target_points = []
            
            #Contours will be generated
            self.contour_generator()
            #Ranked Points on the basis of different parameters
            self.PointsSelector()

            # Exploring on ranked points
            self.Explore_From_Ranked_Points()

            # Publishing Marker on Target Point
            if(len(self.shifted_frontier_target_points)>0):
                print("Base Point: ", self.points_before_shifting[0])
                self.FinalFrontierSelector(point=self.shifted_frontier_target_points[0])
            else:
                print("No Point Found")

            print("\n")

    def start_marker(self):
        self.marker=Marker()

        self.marker.header.frame_id = 'map'
        self.marker.header.stamp = rospy.Time.now()

        # Cube Marker
        self.marker.type = 1
        #Marker Id
        self.marker.id = 0
        
        # Size of the Marker
        self.marker.scale.x = self.size_frontier_target_window*self.map_resolution
        self.marker.scale.y = self.size_frontier_target_window*self.map_resolution
        self.marker.scale.z = 1

        # Color
        self.marker.color.r = 1.0
        self.marker.color.g = 0.0
        self.marker.color.b = 0.0
        # Intensity
        self.marker.color.a = 0.95
        
        #Position and Pose
        self.marker.pose.position.x = self.map_x+(self.map_width-self.Final_Frontier_Point[1])*self.map_resolution
        self.marker.pose.position.y = self.map_y+(self.map_height-self.Final_Frontier_Point[0])*self.map_resolution
        self.marker.pose.position.z = self.map_z
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 1.0

        self.marker_pub.publish(self.marker)

        print(self.Window_Features_at_Point(self.Final_Frontier_Point)[1])

    def map_callback(self, data):
        self.map_width = data.info.width
        self.map_height = data.info.height

        self.map_x, self.map_y, self.map_z= data.info.origin.position.x, data.info.origin.position.y, data.info.origin.position.z

        map_image = np.zeros((self.map_height, self.map_width, 1), dtype="uint8")

        for i in range(1, self.map_height):
            for j in range(1, self.map_width):
                if data.data[(i-1)*self.map_width+j] == -1:
                    map_image[self.map_height-i][self.map_width-j] = 207  # Unknown
                if data.data[(i-1)*self.map_width+j] == 0:
                    map_image[self.map_height-i][self.map_width-j] = 255  # Free
                if data.data[(i-1)*self.map_width+j] == 100:
                    map_image[self.map_height-i][self.map_width-j] = 0   # Obstacle

        map_image = np.flipud(map_image)
        self.map_image_gray = cv.rotate(map_image, cv.ROTATE_90_CLOCKWISE)         # Here map is Grey Scale
        self.map_check = self.map_image_gray.copy()
        self.map_image_bgr = cv.cvtColor(self.map_image_gray, cv.COLOR_GRAY2BGR)   # Here map is BGR

    def contour_generator(self):
        map=self.map_image_gray.copy()

        # Erode and Dilate
        # map=cv.morphologyEx(map, cv.MORPH_OPEN, self.kernel, iterations=1)

        self.map_smooth = map.copy()

        # Bring Black between white and grey
        for i in range(map.shape[0]):
            for j in range(map.shape[1]):
                if map[i][j] < 10:
                    map[i][j] = 225

        # Canny Edge Detection
        edge = cv.Canny(map, self.minVal, self.maxVal)
        
        # edge contour has contour containing list of all the points in the contour
        self.edge_contour, hierarchy = cv.findContours(edge, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

    def PointsSelector(self):
        # Firt remove those points in each contour which lies on border
        accepted_points = []
        for contour in self.edge_contour:    
            current_counter = []
            for i in range(len(contour)):
                point=contour[i][0][0], contour[i][0][1]
                if(
                    (self.size_frontier_target_window > point[0] or point[0] > self.map_smooth.shape[1]-self.size_frontier_target_window)
                    or
                    (self.size_frontier_target_window > point[1] or point[1] > self.map_smooth.shape[0]-self.size_frontier_target_window)
                ):
                    continue

                current_counter.append(point)
            accepted_points.append(current_counter)

        self.ranked_points=[]

        #Add points from each contour with best grey count in this list
        #Unordered after this loop according to grey count
        for i in range(len(accepted_points)):
            best_grey_count=0
            for j in range(len(accepted_points[i])):
                map_window=self.Window_Features_at_Point(accepted_points[i][j])[1]
                grey_count=0
                for m in range(self.size_frontier_target_window):
                    for n in range(self.size_frontier_target_window):
                        if(map_window[m][n]==207):
                            grey_count+=1
                if(grey_count>best_grey_count):
                    best_grey_count=grey_count
                    self.ranked_points.append([accepted_points[i][j], grey_count])

        # Sort the list on the basis of grey count
        for i in range(len(self.ranked_points)):
            for j in range(len(self.ranked_points)-i-1):
                if(self.ranked_points[j][1]<self.ranked_points[j+1][1]):
                    temp = self.ranked_points[j]
                    self.ranked_points[j] = self.ranked_points[j+1]
                    self.ranked_points[j+1] = temp
        
        # Removing the grey count part
        for i in range(len(self.ranked_points)):
            self.ranked_points[i]=self.ranked_points[i][0]

    def Explore_From_Ranked_Points(self):
        self.visited_points = []
        for point in self.ranked_points:
            self.point=point
            target_window, map_window, target_theta = self.Window_Features_at_Point(point)

            print("Point: ", point)
            self.visited_points.append(point)

            self.current_explore_count=0
            self.PointShifter(point=point, target_theta=target_theta, map_window=map_window)

    def Window_Features_at_Point(self, point, explore_points=None):
        (x, y) = point

        map_window = np.zeros((self.size_frontier_target_window,self.size_frontier_target_window))
        for county in range(-self.floor_half_size_frontier_target_window, self.floor_half_size_frontier_target_window+1):
            for countx in range(-self.floor_half_size_frontier_target_window, self.floor_half_size_frontier_target_window+1):
                map_window[county+self.floor_half_size_frontier_target_window][countx + self.floor_half_size_frontier_target_window] = self.map_smooth[y+county][x+countx]


        ret, target_window = cv.threshold(map_window, 230, 255, cv.THRESH_BINARY)
        theta, gradient = self.Matrix_Gradient__Theta(target_window)
        target_theta, target_gradient = self.FrontierInterpolation(theta, gradient)

        if(explore_points == True and self.current_explore_count<self.max_explore_from_point and (self.size_frontier_target_window <= point[0] <= self.map_smooth.shape[1]-self.size_frontier_target_window and self.size_frontier_target_window <= point[1] <= self.map_smooth.shape[0]-self.size_frontier_target_window)):
            #Explore on new point by shifting and above we check for boundry condition too
            self.current_explore_count+=1
            print("Explore Point: ", point)
            self.PointShifter(point, target_theta=target_theta, map_window=map_window)

        return target_window, map_window, target_theta

    def gaussian_kernel(self, size, sigma):
        size = int(size)//2
        x, y = np.mgrid[-size:size+1, -size:size+1]
        normal = 1 / (2.0 * np.pi * sigma**2)
        g = np.exp(-((x**2 + y**2) / (2.0*sigma**2))) * normal
        return g

    def sobel_filter(self, smooth_image):
        # Sobel Filter
        sobel_x = np.matrix([[-1, 0, 1], [-2, 0, 2], [-1, 0, 1]])
        sobel_y = np.matrix([[1, 2, 1], [0, 0, 0], [-1, -2, -1]])

        # Convolution
        Ix = convolve(smooth_image, sobel_x)
        Iy = convolve(smooth_image, sobel_y)

        G = np.hypot(Ix, Iy)
        G = G/G.max()*255

        theta = np.ones([self.size_frontier_target_window, self.size_frontier_target_window], dtype=np.float32)

        for i in range(self.size_frontier_target_window):
            for j in range(self.size_frontier_target_window):
                # Black Top White Bottom
                if(Iy[i][j] == 1020 and Ix[i][j] == 0):
                    theta[i][j] = -0.5

                # Black Bottom White Top
                elif(Iy[i][j] == -1020 and Ix[i][j] == 0):
                    theta[i][j] = 0.5

                # Black Left White Right
                elif(Iy[i][j] == 0 and Ix[i][j] == -1020):
                    theta[i][j] = 0

                # White Left Black Right
                elif(Iy[i][j] == 0 and Ix[i][j] == 1020):
                    theta[i][j] = 1

                # Not on edge
                elif(Iy[i][j] == 0 and Ix[i][j] == 0):
                    theta[i][j] = None

                elif(Ix[i][j] > 0 and Iy[i][j] > 0):
                    theta[i][j] = - round(np.arctan(Iy[i][j]/Ix[i][j])/math.pi, 2)-0.5

                elif(Ix[i][j] > 0 and Iy[i][j] < 0):
                    theta[i][j] = round(np.arctan(Iy[i][j]/Ix[i][j])/math.pi, 2)+1
                
                else:
                    theta[i][j] = round(np.arctan(Iy[i][j]/Ix[i][j])/math.pi, 2)
        
        #This provide theta and gradient of each element
        return theta, G 

    def Matrix_Gradient__Theta(self, target_window):
        smooth_image = convolve(target_window, self.gaussian_kernel_output)
        ret, thresh_image = cv.threshold(smooth_image, 150, 255, 0)

        return self.sobel_filter(thresh_image)

    def FrontierInterpolation(self, theta, gradient):
        frontier_direction_vector = complex(0, 0)
        theta=theta*math.pi
        
        #resultant vector for the matrix is created here which provide theta
        for i in range(self.size_frontier_target_window):
            for j in range(self.size_frontier_target_window):
                if(gradient[i][j] == 0):   # if magnitude is 0, then theta would be nan hence we skip it
                    continue

                current_vector = gradient[i][j]*cmath.exp(theta[i][j]*1j)
                frontier_direction_vector += current_vector

        target_theta = cmath.phase(frontier_direction_vector)/math.pi     # theta/pi
        frontier_magnitude = abs(frontier_direction_vector)

        return target_theta, frontier_magnitude

    def PointShifter(self, point, target_theta, map_window):
        x, y = point
        
        # Shifting of the points on the basis of theta 
        if(0 <= target_theta < 0.15):
            point = (x+1, y)
        elif(0.15 <= target_theta < 0.35):
            point = (x+1, y-1)
        elif(0.35 <= target_theta < 0.65):
            point = (x, y-1)
        elif(0.65 <= target_theta < 0.85):
            point = (x-1, y-1)
        elif(0.85 <= target_theta):
            point = (x-1, y)

        elif(-0.15 < target_theta < 0):
            point = (x+1, y)
        elif(-0.35 < target_theta <= -0.15):
            point = (x+1, y+1)
        elif(-0.65 < target_theta <= -0.35):
            point = (x, y+1)
        elif(-0.85 < target_theta <= -0.65):
            point = (x-1, y+1)
        elif(target_theta <= -0.85):
            point = (x-1, y)

        condition_check_0=(self.check_0==map_window).any()
        condition_check_207=(self.check_207==map_window).any()
        condition_check_point_not_visited= point not in self.visited_points
        
        # if we not visited some point and it contains 0 or 207 then append it in list of visited points and explore
        if((condition_check_0 or condition_check_207) and condition_check_point_not_visited):
            self.visited_points.append(point)
            self.Window_Features_at_Point(point=point, explore_points=True)

        #if it doesnt contain any 0 and 207 then it is the frontier point and append it in shifted frontier list
        elif(condition_check_0==False and condition_check_207==False):
            print("FRONTIER Point: ", point)
            self.points_before_shifting.append(self.point)
            self.shifted_frontier_target_points.append(point)

    def FinalFrontierSelector(self, point):
        self.Final_Frontier_Point = point
        print("Final Frontier Point: ", self.Final_Frontier_Point)
        self.start_marker()
        


if __name__ == '__main__':
    try:
        FrontierExploration()
    except rospy.ROSInterruptException:
        pass
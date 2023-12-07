from math import sin, cos, radians, atan2, sqrt
import numpy as np

class GapDetector:
    def __init__(self, angular_resolution = 1):
        self._best_gap = None
        self._gap_goal = []
        self._gaps = []  
        self.raw_lidar_data = []  # Raw lidar data
        self.processed_lidar_data = []  # Processed data from -90 to 90 degrees
        self._good_gaps = []
        self.angular_resolution = angular_resolution
        self.threshold_distance = 0
        self.width_threshold = 1.5 * 0.2

    def preprocess_lidar(self, lidar_data, min_distance = 0.1, max_distance = 5):
        """
        Preprocesses LIDAR data by segmenting and taking mean.
        The resultant data starts from -90 degrees to +90 degrees.
        """
        #lidar_data = np.clip(lidar_data, min_distance, max_distance)
        self.raw_lidar_data = lidar_data

        total_points = len(lidar_data)
        #print("preprocess_lidar")
        #print(total_points)
        points_per_segment = int(self.angular_resolution * total_points / 360)
        
        # Initialize a list to store the mean values of the segments
        segment_means = []

        # Process the front right quarter
        for i in range(3 * total_points // 4, total_points, points_per_segment):
            segment = lidar_data[i:i + points_per_segment]
            segment_mean = sum(segment) / len(segment)
            segment_means.append(segment_mean)
        
        # Process the front left quarter
        for i in range(0, total_points // 4, points_per_segment):
            segment = lidar_data[i:i + points_per_segment]
            segment_mean = sum(segment) / len(segment)  
            segment_means.append(segment_mean)
        #sums = 0
        #for data in segment_means:
        #    sums += data
        #mean = sums / len(segment_means)
        self.processed_lidar_data = segment_means
        self.processed_lidar_data = np.clip(segment_means, min_distance, max_distance)
        self.threshold_distance = min(min(self.processed_lidar_data) + 0.3, max(self.processed_lidar_data))
        #self.threshold_distance = mean

    def update(self, robot_pose, goal):
        self.find_gaps()
        #self.filter_gaps()
        self.find_best_gap_goal(robot_pose, goal)
        #goal_dist = self.distance(robot_pose, goal)
        #if goal_dist < (self.threshold_distance - 0.2) or not self._good_gaps:
        #    self._gap_goal = []

        print(self._gap_goal)


    def find_gaps(self):
        gaps = []
        in_gap = False
        lidar_data = self.processed_lidar_data
        for i, distance in enumerate(lidar_data):
            if distance >= self.threshold_distance and not in_gap:
                # Start of a new gap
                start_index = i
                in_gap = True
            elif (distance < self.threshold_distance or i == len(lidar_data) - 1) and in_gap:
                # End of the current gap
                end_index = i - 1 if distance <= self.threshold_distance else i
                gaps.append((start_index, end_index))
                in_gap = False
        self._gaps = gaps

    def filter_gaps(self):
        """
        """
        gaps = self._gaps
        lidar_data = self.processed_lidar_data
        good_gaps = []

        for gap in gaps:
            # Calculate the weights based on the distance to obstacles within the gap
            start_index, end_index = gap
            start_radius = lidar_data[start_index]
            end_radius = lidar_data[end_index]
            start_pos = self.polar_to_cartesian(start_index, start_radius)
            end_pos = self.polar_to_cartesian(end_index, end_radius)
            gap_width = self.distance(end_pos, start_pos)
            if gap_width >= self.width_threshold:
                good_gaps.append(gap)
        #print(good_gaps)

        self._good_gaps = good_gaps


    
    def find_best_gap_goal(self, robot_pose, goal):
        #if not self._good_gaps:
        #    self._gap_goal = []

        gaps = self._gaps
        lidar_data = self.processed_lidar_data
        min_pos = [0 , 0]
        min_cost = 1000
        max_width = -1
        for gap in gaps:
            start_index, end_index = gap
            width = end_index - start_index
            #gap_data = lidar_data[start_index:end_index - 1]
            center_index = (start_index + end_index)//2
            radius = lidar_data[center_index]
            global_pos =  self.calculate_global_target([center_index, radius], robot_pose)
            #dist = self.distance(global_pos, goal)
            #cost = self.cost(dist, end_index - start_index)
            if width > max_width:
                #cost = min_cost
                min_pos =  global_pos
        self._gap_goal = min_pos

    """ def calculate_weighted_target_point(self):
        largest_gap = self._best_gap #
        lidar_data = self.processed_lidar_data #
        # Calculate the weights based on the distance to obstacles within the gap
        start_index, end_index = largest_gap #
        gap_data = lidar_data[start_index:end_index - 1]
        center_index = (start_index + end_index)//2
        target_distance = lidar_data[center_index]

        
        self._gap_goal =[center_index, target_distance]"""
                        
    def calculate_global_target(self, gap, robot_pose):
        center_index, target_distance = gap
        angular_resolution = self.angular_resolution
        relative_angle = radians((center_index * angular_resolution) - 90)
        global_angle = robot_pose[2] + relative_angle
        global_angle = atan2(sin(global_angle), cos(global_angle))

        target_x_global = robot_pose[0] + (target_distance * cos(global_angle))
        target_y_global = robot_pose[1] + (target_distance * sin(global_angle))

        goal_global_pos = [target_x_global, target_y_global]

        return goal_global_pos
    
    def get_gap_goal(self):
        """
        Returns best gap
        """
        return self._gap_goal

    def polar_to_cartesian(self, index, radius):
        angular_resolution = self.angular_resolution
        angle = radians(index * angular_resolution)
        pos_x = radius * cos(angle)
        pos_y = radius * sin(angle)
        return pos_x, pos_y

    def distance(self, start, end):
        #print(end)
        dist = sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2)
        return dist

    def cost(self, distance, gap_size):
        dist_w = 0.5
        size_w = 3
        return size_w * gap_size + dist_w * distance


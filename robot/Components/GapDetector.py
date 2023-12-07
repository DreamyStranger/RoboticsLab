from math import sin, cos, radians, atan2
import numpy as np

class GapDetector:
    def __init__(self, angular_resolution = .5):
        self._best_gap = None
        self._gap_goal = []
        self._gaps = []  
        self.raw_lidar_data = []  # Raw lidar data
        self.processed_lidar_data = []  # Processed data from -90 to 90 degrees
        self.angular_resolution = angular_resolution
        self.threshold_distance = 0

    def preprocess_lidar(self, lidar_data, min_distance = 0.1, max_distance = 3):
        """
        Preprocesses LIDAR data by segmenting and taking mean.
        The resultant data starts from -90 degrees to +90 degrees.
        """
        self.raw_lidar_data = lidar_data
        total_points = len(lidar_data)
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

        self.processed_lidar_data = segment_means
        self.processed_lidar_data = np.clip(segment_means, min_distance, max_distance)
        self.threshold_distance = min(min(self.processed_lidar_data) + 0.4, max(self.processed_lidar_data))

    def update(self, robot_pose, goal):
        self.find_gaps()
        self.find_largest_gap()
        self.calculate_weighted_target_point()
        self.calculate_global_target(robot_pose)

    def find_gaps(self):
        gaps = []
        in_gap = False
        lidar_data = self.processed_lidar_data
        for i, distance in enumerate(lidar_data):
            if distance > self.threshold_distance and not in_gap:
                # Start of a new gap
                start_index = i
                in_gap = True
            elif (distance <= self.threshold_distance or i == len(lidar_data) - 1) and in_gap:
                # End of the current gap
                end_index = i - 1 if distance <= self.threshold_distance else i
                gaps.append((start_index, end_index))
                in_gap = False
        self._gaps = gaps
    
    def find_largest_gap(self):
        gaps = self._gaps
        max_width = -1
        largest_gap = None
        for start_index, end_index in gaps:
            width = end_index - start_index
            if width > max_width:
                max_width = width
                largest_gap = (start_index, end_index)
        self._best_gap = largest_gap

    def calculate_weighted_target_point(self):
        largest_gap = self._best_gap
        lidar_data = self.processed_lidar_data
        # Calculate the weights based on the distance to obstacles within the gap
        start_index, end_index = largest_gap
        gap_data = lidar_data[start_index:end_index]
        center_index = int((end_index + start_index)//2)
        target_distance = lidar_data[center_index]
        """weights = gap_data - np.min(gap_data) + 1  # Add 1 to avoid division by zero
        weighted_sum = np.sum(np.arange(start_index, end_index + 1) * weights)
        total_weight = np.sum(weights)
        center_index = int(weighted_sum / total_weight)
        target_distance = lidar_data[center_index]"""
        self._gap_goal =[center_index, target_distance]

    def calculate_global_target(self, robot_pose):
        center_index, target_distance = self._gap_goal
        angular_resolution = self.angular_resolution
        relative_angle = radians((center_index * angular_resolution) - 90)
        global_angle = robot_pose[2] + relative_angle
        global_angle = atan2(sin(global_angle), cos(global_angle))

        target_x_global = robot_pose[0] + (target_distance * cos(global_angle))
        target_y_global = robot_pose[1] + (target_distance * sin(global_angle))

        self._gap_goal = [target_x_global, target_y_global]
    
    def get_gap_goal(self):
        """
        Returns best gap
        """
        return self._gap_goal
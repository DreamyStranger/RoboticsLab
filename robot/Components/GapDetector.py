from math import sin, cos, radians, atan2
import numpy as np

class GapDetector:
    """
        A class to detect gaps in the environment using LIDAR data for a robotic navigation system.

        Attributes:
            _best_gap (tuple): The start and end indices of the largest detected gap.
            _gap_goal (list): The global position of the center of the largest gap.
            _gaps (list): List of tuples representing the start and end indices of detected gaps.
            processed_lidar_data (list): The LIDAR data processed to focus on a 180-degree field in front of the robot.
            angular_resolution (float): The angular resolution of LIDAR data processing.
            threshold_distance (float): The minimum radius in which consider intercepting object as an obstacle.
    """
    def __init__(self, angular_resolution = .5):
        """
        Initializes the GapDetector with a specified angular resolution.

        Parameters:
            angular_resolution (float): The angular resolution for processing LIDAR data, defaults to 0.5 degrees.
        """
        self._best_gap = None
        self._gap_goal = []
        self._gaps = []
        self.processed_lidar_data = []
        self.angular_resolution = angular_resolution
        self.threshold_distance = 0

    def preprocess_lidar(self, lidar_data, min_distance = 0.1, max_distance = 3):
        """
        Preprocesses the LIDAR data by segmenting it into angular segments and calculating the mean distance in each segment.
        The processed data provides a simplified representation of the environment, focusing on a 180-degree field in front of the robot.

        Parameters:
            lidar_data (list): The raw LIDAR data as a list of distances.
            min_distance (float): The minimum distance to consider for an object to be an obstacle.
            max_distance (float): The maximum distance to consider for an object to be an obstacle.
        """
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

    def update(self, robot_pose):
        """
        Updates the gap detector by finding gaps, the largest gap, and calculating a target point based on the largest gap.
        This method integrates the current robot pose to determine the global position of the target point.

        Parameters:
            robot_pose (tuple): The current pose of the robot as (x, y, theta).
        """
        self.find_gaps()
        self.find_largest_gap()
        self.calculate_weighted_target_point()
        self.calculate_global_target(robot_pose)

    def find_gaps(self):
        """
        Identifies gaps in the processed LIDAR data. A gap is defined as a contiguous set of points where no obstacle is detected.
        This method updates the '_gaps' attribute with the start and end indices of each detected gap.
        """
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
        """
        Finds the largest gap among the detected gaps. The largest gap is determined based on its width.
        Updates the '_best_gap' attribute with the start and end indices of the largest gap.
        """
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
        """
        Calculates a target point within the largest gap. The target point is determined based on the center of the gap.
        Updates the '_gap_goal' attribute with the position of the target point.
        """
        largest_gap = self._best_gap
        lidar_data = self.processed_lidar_data
        start_index, end_index = largest_gap
        center_index = int((end_index + start_index)//2)
        target_distance = lidar_data[center_index]
        self._gap_goal =[center_index, target_distance]

    def calculate_global_target(self, robot_pose):
        """
        Converts the local target point within the largest gap to global coordinates based on the current robot pose.
        This method is essential for navigation in the global frame of reference.

        Parameters:
            robot_pose (tuple): The current pose of the robot as (x, y, theta).
        """
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
        Retrieves the global position of the target point within the largest gap.

        Returns:
            list: The global coordinates (x, y) of the target point within the largest gap.
        """
        return self._gap_goal
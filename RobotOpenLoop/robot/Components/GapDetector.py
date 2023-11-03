from math import sin, cos, radians
import numpy as np

class GapDetector:
    def __init__(self, angular_resolution = 10):
        """
        Initializes the GapDetector
        """
        self._best_gap = None
        self._best_gap_cost = 1000
        self._gaps = []  
        self.lidar_data = []
        self.angular_resolution = angular_resolution

    def preprocess_lidar(self, lidar_data):
        # Assuming 360-degree coverage and the LIDAR data is ordered clockwise
        # The front half would be the first quarter and the last quarter of the data

        total_points = len(lidar_data)
        points_per_segment = int(self.angular_resolution * total_points / 360)
        
        # Initialize a list to store the mean values of the segments
        segment_means = []
        
        # Process the front left quarter
        for i in range(0, total_points // 4, points_per_segment):
            segment = lidar_data[i:i + points_per_segment]
            segment_mean = sum(segment) / len(segment)
            segment_means.append(segment_mean)

        # Process the front right quarter
        for i in range(3 * total_points // 4, total_points, points_per_segment):
            segment = lidar_data[i:i + points_per_segment]
            segment_mean = sum(segment) / len(segment)
            segment_means.append(segment_mean)

        self.lidar_data = segment_means

    def find_gaps(self, threshold_distance):
        # Assuming preprocessed_data is already focused on the front half
        # Analyze the preprocessed data to identify gaps
        gaps = []
        gap_start = None
        for index, distance in enumerate(self.lidar_data):
            if distance > threshold_distance:
                if gap_start is None:
                    gap_start = index  # Start of a new gap
            else:
                if gap_start is not None:
                    # End of the current gap, add to list
                    gap_end = index
                    gaps.append((gap_start, gap_end))
                    gap_start = None
        # Don't forget to check if the last segment is a gap
        if gap_start is not None:
            gaps.append((gap_start, len(self.data)))

        return gaps




    def find_best_gap(self, robot_pose, goal_position):
        """
        Determines the most suitable gap from the identified gaps based on the robot's 
        current position and the goal's location. Uses it to update self._best_gap and its
        cost if neccessary.

        Parameters:
        - robot_pose (tuple): The current pose of the robot (x, y, theta).
        - goal_position (tuple): The target position (x, y).
        """
        min_cost = 1000
        best_gap = None
        for gap in self._gaps:
            cost = self.cost_function(gap, robot_pose, goal_position)
            if cost < min_cost:
                min_cost = cost
                best_gap = gap
        self.update_best_gap(best_gap, min_cost)

    def cost_function(self, gap_center, robot_pose, goal_position, weight_distance=0.5, weight_alignment=.5):
        """
        Calculates the cost of a gap, considering the distance to the goal and the 
        alignment with the goal direction.

        Parameters:
        - gap_center (tuple): Center of the gap (x, y).
        - robot_pose (tuple): Robot's pose (x, y, theta).
        - goal_position (tuple): Goal's position (x, y).
        - weight_distance (float): Weight factor for distance to the goal.
        - weight_alignment (float): Weight factor for alignment with the goal.

        Returns:
        - float: The calculated cost for the gap.
        """
        gap_x, gap_y = gap_center
        goal_x, goal_y = goal_position
        robot_x, robot_y, robot_theta = robot_pose

        # Calculate the distance from the gap center to the goal
        distance_to_goal = np.hypot(goal_x - gap_x, goal_y - gap_y)

        # Calculate the direction to the gap from the robot
        gap_direction = np.arctan2(gap_y - robot_y, gap_x - robot_x)
        # Normalize to [-pi, pi]
        gap_direction = (gap_direction - robot_theta + np.pi) % (2 * np.pi) - np.pi

        # Calculate the direction to the goal from the robot
        goal_direction = np.arctan2(goal_y - robot_y, goal_x - robot_x)
        # Normalize to [-pi, pi]
        goal_direction = (goal_direction - robot_theta + np.pi) % (2 * np.pi) - np.pi

        # Calculate the difference in direction to the gap and the goal
        direction_difference = abs(goal_direction - gap_direction)

        # The smaller the difference, the better the alignment, hence the subtraction from 1
        alignment_with_goal = 1 - (direction_difference / np.pi)  # Normalized to [0, 1]

        # Compute the weighted cost
        cost = (weight_distance * distance_to_goal) + (weight_alignment * (1 - alignment_with_goal))
        return cost

    def update_best_gap(self, new_gap, new_gap_cost):
        switch = self._best_gap is None or new_gap_cost < self._best_gap_cost * .8

        if 1:
            self._best_gap = new_gap
            self._best_gap_cost = new_gap_cost
    
    def get_best_gap(self):
        """
        Returns best gap
        """
        return self._best_gap


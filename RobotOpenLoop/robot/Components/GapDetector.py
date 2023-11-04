from math import sin, cos, radians
import numpy as np

class GapDetector:
    def __init__(self, angular_resolution=5):
        """
        Initializes the GapDetector
        """
        self._best_gap = None
        self._best_gap_cost = 1000
        self._gaps = []  
        self.raw_lidar_data = []  # Raw lidar data
        self.processed_lidar_data = []  # Processed data from -90 to 90 degrees
        self._suitable_gaps = []
        self.angular_resolution = angular_resolution
        self.threshold_distance = 0

    def preprocess_lidar(self, lidar_data):
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
        self.threshold_distance = min(min(self.processed_lidar_data) + 0.3, max(self.processed_lidar_data))

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
        print("Gaps: ", gaps)
        return gaps
    
    def find_largest_gap(gaps):
        max_width = -1
        largest_gap = None
        for start_index, end_index in gaps:
            width = end_index - start_index
            if width > max_width:
                max_width = width
                largest_gap = (start_index, end_index)
        return largest_gap
    

    def process_gaps(self, robot_diameter):
        """
        Converts gaps' boundaries from polar coordinates to cartesian coordinates
        and finds suitable gaps based on robot diameter.
        """

        suitable_gaps = []
        
        # Retrieve the gaps using the find_gaps function
        gaps = self.find_gaps()
        
        for gap_start, gap_end in gaps:
            # Convert polar to cartesian for the starting boundary of the gap
            start_angle_rad = radians(gap_start * self.angular_resolution - 90)
            start_x = self.threshold_distance * cos(start_angle_rad)
            start_y = self.threshold_distance * sin(start_angle_rad)
            
            # Convert polar to cartesian for the ending boundary of the gap
            end_angle_rad = radians(gap_end * self.angular_resolution - 90)
            end_x = self.threshold_distance * cos(end_angle_rad)
            end_y = self.threshold_distance * sin(end_angle_rad)
            
            # Calculate the distance between the start and end of the gap
            gap_width = ((end_x - start_x)**2 + (end_y - start_y)**2)**0.5
            
            # Check if gap is suitable
            if gap_width >= robot_diameter * 1.5:
                # Calculate the central point of the gap
                center_x = (start_x + end_x) / 2
                center_y = (start_y + end_y) / 2
                
                # Calculate the angle of the gap
                gap_angle = radians((gap_end + gap_start) / 2 * self.angular_resolution - 90)
                
                suitable_gaps.append({
                    'start': (start_x, start_y),
                    'end': (end_x, end_y),
                    'center': (center_x, center_y),
                    'angle': gap_angle,
                    'width': gap_width,
                })
            
            self._suitable_gaps = suitable_gaps

    def determine_best_gap(self, robot_orientation, goal_angle):
        """
        Determines the best gap based on the cost function.
        
        :param goal_angle: The bearing or angle to the goal in radians.
        :param robot_orientation: The current orientation or heading of the robot in radians.
        :return: The best gap.
        """
        
        best_gap = None
        best_gap_cost = 10000 
        
        for gap in self._suitable_gaps:
            gap_cost = self.evaluate_gap_cost(gap,  robot_orientation, goal_angle)
            if gap_cost < best_gap_cost:
                best_gap = gap
                best_gap_cost = gap_cost

        self._best_gap = best_gap
        self._best_gap_cost = best_gap_cost

    def evaluate_gap_cost(self, gap, robot_orientation, goal_angle):
        """
        Evaluates the cost of a gap based on its width, alignment with the goal, and alignment with the robot.
        
        :param gap: The gap to evaluate.
        :param goal_angle: The bearing or angle to the goal in radians.
        :param robot_orientation: The current orientation or heading of the robot in radians.
        :return: The cost of the gap.
        """

        # Weight factors for each criteria
        width_weight = 1.0
        goal_alignment_weight = 1.0
        robot_alignment_weight = 1.0

        # Evaluate how big the gap is - bigger gaps are preferable
        width_cost = 1/gap["width"]  # We use a negative value because wider gaps should have a lower cost (more desirable)

        # Evaluate alignment with the goal - smaller differences in angle are preferable
        goal_alignment_cost = abs(robot_orientation + gap['angle'] - goal_angle)

        # Evaluate alignment with the robot's orientation - smaller differences in angle are preferable
        robot_alignment_cost = abs(gap['angle'])

        # Compute the total cost with weight factors
        total_cost = (width_weight * width_cost + 
                    goal_alignment_weight * goal_alignment_cost + 
                    robot_alignment_weight * robot_alignment_cost)

        return total_cost

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


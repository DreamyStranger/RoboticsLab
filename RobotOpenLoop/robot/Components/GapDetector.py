from math import sin, cos, radians

class GapDetector():
    def __init__(self):
        self.gaps = []
        

    def detect_gaps(self, lidar_data, robot_length, robot_width):
        num_sectors = len(lidar_data)
        degree_per_sector = 360 / num_sectors
        threshold_distance = 2 * robot_length
        self.gaps = []

        i = 0

        while i < num_sectors:
            if lidar_data[i] >= threshold_distance:
                start_sector = i
                while i < num_sectors and lidar_data[i] >= threshold_distance:
                    i += 1
            end_sector = i - 1
            gap_width = (end_sector - start_sector + 1) * degree_per_sector
            if gap_width >= robot_width:
                # Calculate the average angle of the gap
                gap_angle = ((start_sector + end_sector) / 2) * degree_per_sector
                
                # Convert polar coordinates to Cartesian coordinates
                x = lidar_data[i] * cos(radians(gap_angle))
                y = lidar_data[i] * sin(radians(gap_angle))
                self.gaps.append((x, y))
            i += 1
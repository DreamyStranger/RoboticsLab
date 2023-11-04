import numpy as np
### These 2 lines are for linux
#import matplotlib
#matplotlib.use('TkAgg')  # Use the TkAgg backend for interactive plots
###
import matplotlib.pyplot as plt
from robot.robot import Robot
from robot.Lidar import Lidar

# Initial robot position and settings
x_r = 0
y_r = 0
theta = 0
freq = 20

def square_data(x, y, side_length = 4):
    """
    Generate square path data.

    Args:
        x (float): X-coordinate of the square's center.
        y (float): Y-coordinate of the square's center.
        side_length (float, optional): Length of each side of the square. Defaults to 1.

    Returns:
        tuple: Lists of x and y coordinates forming a square path.
    """
    half_side = side_length / 2
    x_out = [x - half_side, x + half_side, x + half_side, x - half_side]
    y_out = [y - half_side, y - half_side, y + half_side, y + half_side]
    return x_out, y_out

def main():
    count = 0
    robot = Robot()
    lidar = Lidar()
    obstacles = robot.environment.obstacles

            
    navigation_type, path_function = "pid", square_data
    
    path_x, path_y = path_function(x_r, y_r)
    robot.navigation.type = navigation_type
    robot.goal_controller.add_goals(path_x, path_y)
    fig, ax = plt.subplots()

    while True:
        linear, angular = robot.odometer.get_velocities()
        pose = robot.odometer.get_pose()
        lidar.update(pose, obstacles)
        robot.gap_detector.preprocess_lidar(lidar.get_data())
        robot.update(0.1)
        #print("Pose: ", robot.odometer.get_pose())
        #print("Linear Velocity: ", linear)
        #print("Angular Velocity: ", angular)
        print("Lidar Data: ", robot.gap_detector.processed_lidar_data)
        robot.draw(ax)

        plt.pause(0.05)  # Pause to update the display

        if count == 20000 or robot.goal_controller.all_goals_reached():
            break
        count += 1

    plt.show()  # Keep the plot window open

if __name__ == "__main__":
    main()

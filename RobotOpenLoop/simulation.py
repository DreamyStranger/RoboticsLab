import numpy as np
### These 2 lines are for linux
#import matplotlib
#matplotlib.use('TkAgg')  # Use the TkAgg backend for interactive plots
###
import matplotlib.pyplot as plt
from robot.robot import Robot
from robot.LidarEmulator import LidarEmulator

# Initial robot position and settings
x_r = 0
y_r = 0
theta = 0
freq = 20

def main():
    count = 0
    robot = Robot()
    lidar = LidarEmulator()
    obstacles = robot.environment.obstacles
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
        #print("Lidar Data: ", robot.gap_detector.processed_lidar_data)
        print("GOAL: ", robot.goal_controller.get_current_goal())
        robot.draw(ax)

        plt.pause(0.05)  # Pause to update the display

        if count == 20000 or robot.goal_controller.all_goals_reached():
            break
        count += 1

    plt.show()  # Keep the plot window open

if __name__ == "__main__":
    main()

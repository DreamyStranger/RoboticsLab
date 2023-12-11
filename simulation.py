import numpy as np
import threading
### These 2 lines are for linux
import matplotlib
matplotlib.use('TkAgg')  # Use the TkAgg backend for interactive plots
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
    robot.goal_controller.add_goal([0, 10])
    lidar = LidarEmulator()
    obstacles = robot.environment.obstacles
    fig, ax = plt.subplots()

    robot.input_system.update('to_goal')

    gesture_thread = threading.Thread(target=robot.gesture_handler.start)
    gesture_thread.start()

    while True:
        if robot.gesture_event.is_set():
            gesture_data = robot.gesture_data
            robot.gesture_event.clear()  # Clear the event after processing

            if gesture_data == "thumbs_up":
                robot.input_system.update("idle")

        pose = robot.odometer.get_pose()
        lidar.update(pose, obstacles)
        robot.gap_detector.preprocess_lidar(lidar.get_data())
        robot.update(0.1)
        #print("Pose: ", robot.odometer.get_pose())
        #print("Linear Velocity: ", linear)
        #print("Angular Velocity: ", angular)
        #print("Lidar Data: ", robot.gap_detector.processed_lidar_data)
        #print("GOAL: ", robot.goal_controller.get_current_goal())
        robot.draw(ax)

        if count == 200000:
            robot.gesture_handler.stop()
            break
    
    gesture_thread.join()
    plt.show()  # Keep the plot window open

if __name__ == "__main__":
    main()

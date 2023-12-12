import numpy as np
import threading
from GestureRecognition import HandGestureRecognition
from queue import Queue	 
### These 2 lines are for linux
import matplotlib
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

def producer(q):
    gesture_recognition = HandGestureRecognition()
    gesture_recognition.start(q)


def consumer(q):
    count = 0
    robot = Robot()
    if robot.state_machine.is_superstate("Simulation"):
        robot.odometer.set_pose([0, -4, 1.17])
    robot.goal_controller.add_goal([0, 10])
    lidar = LidarEmulator()
    obstacles = robot.environment.obstacles
    fig, ax = plt.subplots()

    robot.input_system.update('to_goal')

    while True:
        if not q.empty():
            gesture = q.get()
            if gesture:
                robot.input_system.update(gesture)

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

        if count == 200000 or robot.state_machine.is_state("Stop"):
            robot.gesture_handler.stop()
            break


def main():
    q = Queue()
    t1 = threading.Thread(target = producer, args = (q,))
    t2 = threading.Thread(target = consumer, args = (q,))
    t1.start()
    t2.start()
    t2.join()
    t1.join()
    plt.show()  # Keep the plot window open

if __name__ == "__main__":
    main()

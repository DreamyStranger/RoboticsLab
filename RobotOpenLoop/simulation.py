import numpy as np
### These 2 lines are for linux
#import matplotlib
#matplotlib.use('TkAgg')  # Use the TkAgg backend for interactive plots
###
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from robot.robot import Robot
import time

# Initial robot position and settings
x_r = 0
y_r = 0
theta = 0
freq = 20
robot = None

def circle_data(x, y):
    """
    Generate circular path data.

    Args:
        x (float): X-coordinate of the circle center.
        y (float): Y-coordinate of the circle center.

    Returns:
        tuple: Lists of x and y coordinates forming a circular path.
    """
    r = 1
    theta = np.linspace(0, 2*np.pi, 100)
    x_out = x + r * np.cos(theta)
    y_out = y + r * np.sin(theta)
    return x_out.tolist(), y_out.tolist()

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

def point_data(x, y):
    """
    Generate a point path data.

    Args:
        x (float): X-coordinate of the point.
        y (float): Y-coordinate of the point.

    Returns:
        tuple: Lists of x and y coordinates forming a single point.
    """
    return [x + 1.5], [y + 1.5]

def main():
    count = 0
    robot = Robot()
    
    # Define navigation options with associated navigation types and path functions
    navigation_options = {
        1: ("pid", point_data),
        2: ("pid", square_data),
        3: ("circle", circle_data),
    }

    print("Select a navigation option:")
    print("1. Point navigation")
    print("2. Square navigation")
    print("3. Circular navigation")
    
    while True:
        try:
            navigation_option = int(input("Enter the option number: "))
            if navigation_option in [1, 2, 3]:
                break
            else:
                print("Invalid option. Please enter a valid option number.")
        except ValueError:
            print("Invalid input. Please enter a valid option number.")
            
    navigation_type, path_function = navigation_options[navigation_option]
    
    path_x, path_y = path_function(x_r, y_r)
    robot.navigation.type = navigation_type
    robot.goal_controller.add_goals(path_x, path_y)

    fig, ax = plt.subplots()
    ax.axis([-2, 2, -2, 2])
    
    # Initialize empty lists to store the path and trajectory
    trajectory_x = []
    trajectory_y = []

    while True:
        print("Pose: ", robot.odometer.get_pose())
        linear, angular = robot.odometer.get_velocities()
        print("Linear Velocity: ", linear)
        print("Angular Velocity: ", angular)
        robot.update(0.1)

        ax.clear()  # Clear the previous frame
        ax.axis([-2, 2, -2, 2])  # Set the limits of your workspace
        ax.grid(True)  # Optional: Add grid for better visualization

        # Plot the robot's current position
        robot.plot(ax)
        
        # Append the current position to the trajectory
        pose = robot.odometer.get_pose()
        trajectory_x.append(pose[0])
        trajectory_y.append(pose[1])
        
        # Plot the path
        if navigation_type == "pid":
            ax.plot(path_x, path_y, 'ro', label='Path')
        else:
            ax.plot(path_x, path_y, 'b--', label='Path')
        
        # Plot the robot's trajectory
        ax.plot(trajectory_x, trajectory_y, 'g-', label='Trajectory')
        
        # Add legend
        ax.legend()
        
        plt.pause(0.1)  # Pause to update the display

        if count == 20000 or robot.goal_controller.all_goals_reached():
            break
        count += 1

    plt.show()  # Keep the plot window open

if __name__ == "__main__":
    main()

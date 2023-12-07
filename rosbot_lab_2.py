#!/usr/bin/env python

# Ros
import rospy;
from geometry_msgs.msg import Twist;
from nav_msgs.msg import Odometry;
from sensor_msgs.msg import Imu

# Python
from scipy.spatial.transform import Rotation;
import numpy as np
###
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time

# Ours
from robot.robot import Robot;

x_r = 0;
y_r = 0;
theta = 0;
freq = 20;
robot = None;

# Debug Flags
debug_robot = False;

# State Flags

def odomCallback(data):
	"""
	We are using this for basic troubleshooting
	"""
	global theta, x_r, y_r;
	x = data.pose.pose.orientation.x;
	y = data.pose.pose.orientation.y;
	z = data.pose.pose.orientation.z;
	w = data.pose.pose.orientation.w;
	#below, is w angle?
	rotation = Rotation.from_quat([x,y,z,w]);
	#what is "zyx"
	theta = rotation.as_euler("zyx")[0];
	
	#Difference between orientation and position?
	x_r = data.pose.pose.position.x;
	y_r = data.pose.pose.position.y;
	
	if(debug_robot):
		print("Robot is Running!");
		print("Robot's orientation is ", orientation);
		

def circle_data(x, y):
    """
    Generate circular path data.

    Args:
        x (float): X-coordinate of the circle center.
        y (float): Y-coordinate of the circle center.

    Returns:
        tuple: Lists of x and y coordinates forming a circular path.
    """
    r = .5
    theta = np.linspace(0, 2*np.pi, 100)
    x_out = x + r * np.cos(theta)
    y_out = y + r * np.sin(theta)
    return x_out.tolist(), y_out.tolist()

def square_data(x, y, side_length=1):
    """
    Generate square path data.

    Args:
        x (float): X-coordinate of the square's center.
        y (float): Y-coordinate of the square's center.
        side_length (float, optional): Length of each side of the square. Defaults to 1.

    Returns:
        tuple: Lists of x and y coordinates forming a square path.
    """
    x_out = [x + side_length, x + side_length, x, x]
    y_out = [y, y + side_length, y + side_length, y]
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
    return [x + 1], [y + 1]

	
def main():
	
	# Publishers
    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1);
    
    # Nodes
    rospy.init_node('rosbot_turn', anonymous=True);
    
    # Subscribers
    rospy.Subscriber("/odom", Odometry, odomCallback);


    rate = rospy.Rate(freq);  # Rate with which loop is running
    vel = Twist(); # initialize a geometry_twist message
    count = 0;


    rate.sleep();
    rate.sleep();
    rate.sleep();
    robot = Robot();
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
    robot.navigation.add_goals(path_x, path_y)
        
    while not rospy.is_shutdown():
    	robot.odometer.set_pose([x_r, y_r, theta]);
    	print("Pose: ", robot.odometer.get_pose());
    	robot.update(1.0/freq);
    	linear_velocity, angular_velocity = robot.odometer.get_velocities();
        print("Linear Velocity: ", linear_velocity)
        print("Angular Velocity: ", angular_velocity)
    	vel.linear.x = linear_velocity;
    	vel.angular.z = angular_velocity;
    	vel_pub.publish(vel);

    	if(count == 20000) or robot.navigation.is_goal_reached():
    		vel.linear.x = 0.0;
    		vel.angular.z = 0.0;
    		break;
    	count += 1;
        rate.sleep();
if __name__ == "__main__":
    main()



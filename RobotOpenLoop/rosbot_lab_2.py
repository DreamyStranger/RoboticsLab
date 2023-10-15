#!/usr/bin/env python

# Ros
import rospy;
from geometry_msgs.msg import Twist;
from nav_msgs.msg import Odometry;
from sensor_msgs.msg import Imu

# Python
from scipy.spatial.transform import Rotation;
import numpy as np

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
	r = 0.5;
	theta = np.linspace(0, 2*np.pi, 100);
	x_out = x + r*np.cos(theta);
	y_out = y +  r*np.sin(theta);
	return x_out.tolist(), y_out.tolist();

def square_data(x, y):
	x_c = [0.5, 0.5, 0, 0];
	y_c = [0, 0.5, 0.5, 0];
	x_out = [i + x for i in x_c];
	y_out = [i + y for i in y_c];
	print(x_out, y_out);
	return x_out, y_out;

	
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
    #robot.navigation.add_goal([x_r + .5 , y_r + .5]);
    path_x, path_y = circle_data(x_r, y_r);
    robot.navigation.type = "cirsd";
    robot.navigation.add_goals(path_x, path_y);
        
    while not rospy.is_shutdown():
    	#robot.odometer.set_pose([x_r, y_r, theta]);
    	print("Pose: ", robot.odometer.get_pose());
    	robot.update(1.0/freq);
    	linear_velocity, angular_velocity = robot.odometer.get_velocities();
    	vel.linear.x = linear_velocity;
    	vel.angular.z = angular_velocity;
    	vel_pub.publish(vel);
    	if(count == 20000) or robot.navigation._goal_reached:
    		vel.linear.x = 0.0;
    		vel.angular.z = 0.0;
    		break;
    	count += 1;
        rate.sleep();
if __name__ == "__main__":
    main()



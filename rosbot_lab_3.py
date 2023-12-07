#!/usr/bin/env python

# Ros
import rospy;
from geometry_msgs.msg import Twist;
from geometry_msgs.msg import PoseArray;
from nav_msgs.msg import Odometry;
from sensor_msgs.msg import Imu;
from sensor_msgs.msg import LaserScan;
from tf.transformations import euler_from_quaternion

import math


# Python
#from scipy.spatial.transform import Rotation;


# Ours
from robot.robot import Robot;

x_r = 0;
y_r = 0;
points= []
means = []
theta = 0;
freq = 20;
robot = None;
path_x, path_y = [], []

# Debug Flags
debug_robot = False;

# State Flags

def odomCallback(data):
    """
    We are using this for basic troubleshooting
    """
    global theta, x_r, y_r;
    x = data.pose.pose.orientation.x
    y = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    #below, is w angle?
    #rotation = Rotation.from_quat([x,y,z,w]);
    #what is "zyx"
    #theta = rotation.as_euler("zyx")[0];
    #Difference between orientation and position?
    theta = euler_from_quaternion([x,y,z,w])[2]
    x_r = data.pose.pose.position.x
    y_r = data.pose.pose.position.y
    
    if(debug_robot):
        print("Robot is Running!");
        print("Robot's orientation is ", orientation);
        
"""def gapcb(msg):
        gapList = msg.poses
    global means

    for i in range(0,len(gapList)-1, 2):
        p1 = gapList[i].position
        p2 = gapList[i+1].position
        mean_x = (p1.x + p2.x)/2
        mean_y = (p1.y + p2.y)/2
        mean_point = [mean_x, mean_y]
        means.append(mean_point)"""

        
def lidarCallback(data):
    #print(len(data.ranges))
    global points
    points = data.ranges
    means = []
    for point in points:
        if(math.isinf(point)):
            point = 2
        

def point_data(x, y):
    """
    Generate a point path data.

    Args:
        x (float): X-coordinate of the point.
        y (float): Y-coordinate of the point.

    Returns:
        tuple: Lists of x and y coordinates forming a single point.
    """
    return [x + 10], [y]

def closest_gap(gaps, path_x, path_y):
    
    min_dist = 100
    best_point = []
    for point in gaps:     
        dist = math.sqrt((point[0]-path_x[0])**2+(point[1]-path_y[0])**2)
        if dist < min_dist:
            min_dist = dist
            best_point = point
    return (best_point, min_dist)
    

def main():
    # Publishers
    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1);
    
    # Nodes
    rospy.init_node('rosbot_turn', anonymous=True);
    
    # Subscribers
    rospy.Subscriber("/odom", Odometry, odomCallback);
    rospy.Subscriber("/scan", LaserScan, lidarCallback);
    #rospy.Subscriber('/gaps', PoseArray, gapcb)

    rate = rospy.Rate(freq);  # Rate with which loop is running
    vel = Twist(); # initialize a geometry_twist message
    count = 0;


    rate.sleep();
    rate.sleep();
    rate.sleep();
    rate.sleep();
    rate.sleep();
    robot = Robot();
            
    navigation_type, path_function = "pid", point_data
    
    path_x, path_y = path_function(x_r, y_r)
    robot.navigation.type = navigation_type
    robot.goal_controller.add_goals(path_x, path_y)
        
    while not rospy.is_shutdown():
        robot.gap_detector.preprocess_lidar(points)
        robot.odometer.set_pose([x_r, y_r, theta]);
        #print("Pose: ", robot.odometer.get_pose());
        #print("Best Gap: ", best_gap)
        #print("Min Dist: ", min_dist)
        robot.update(1.0/freq);
        linear_velocity, angular_velocity = robot.odometer.get_velocities();
        #print("Linear Velocity: ", linear_velocity)
        #print("Angular Velocity: ", angular_velocity)
        #print("Gaps: ", robot.gap_detector.get_gap_goal())
        vel.linear.x = linear_velocity;
        vel.angular.z = angular_velocity;
        vel_pub.publish(vel);

        if(count == 20000) or robot.goal_controller.all_goals_reached():
            vel.linear.x = 0.0;
            vel.angular.z = 0.0;
            break;
        count += 1;
        rate.sleep();
if __name__ == "__main__":
    main()



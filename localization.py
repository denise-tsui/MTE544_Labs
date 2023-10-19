import sys

from utilities import Logger, euler_from_quaternion
from rclpy.time import Time
from rclpy.node import Node

from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry as odom

from rclpy import init, spin, executors

# ADDED IMPORTS FROM LAB 1
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
# from nav_msgs.msg import Odometry

rawSensor = 0
class localization(Node):
    
    def __init__(self, localizationType=rawSensor):

        super().__init__("localizer")
        
        # TODO Part 3: Define the QoS profile variable based on whether you are using the simulation (Turtlebot 3 Burger) or the real robot (Turtlebot 4)
        # Remember to define your QoS profile based on the information available in "ros2 topic info /odom --verbose" as explained in Tutorial 3

        # LAB 1 QOS CHECK THIS
        odom_qos = QoSProfile(depth = 10, reliability='RELIABLE', durability='VOLATILE', history='UNKNOWN')
        # odom_qos = QoSProfile(depth = 10, reliability=2, durability=2, history=1)
        
        self.loc_logger=Logger("robot_pose.csv", ["x", "y", "theta", "stamp"])
        self.pose=None
        
        if localizationType == rawSensor:
        # TODO Part 3: subscribe to the position sensor topic (Odometry)
            self.create_subscription(odom, "/odom", self.odom_callback, qos_profile=odom_qos)

        else:
            print("This type doesn't exist", sys.stderr)
    
    
    def odom_callback(self, pose_msg):
        
        # TODO Part 3: Read x,y, theta, and record the stamp
        x = pose_msg.pose.pose.position.x
        y = pose_msg.pose.pose.position.y
        theta = euler_from_quaternion(pose_msg.pose.pose.orientation)
        timestamp = pose_msg.header.stamp
        self.pose=[x, y, theta, timestamp]
        
        # Log the data
        self.loc_logger.log_values([self.pose[0], self.pose[1], self.pose[2], Time.from_msg(self.pose[3]).nanoseconds])
    
    def getPose(self):
        return self.pose

# TODO Part 3
# Here put a guard that makes the node run, ONLY when run as a main thread!
# This is to make sure this node functions right before using it in decision.py
def check_main_thread(node):
    # Get the executor associated with the node
    execute = node.get_node_names_and_namespaces()
    return execute == executors.get_rclpy_executor().get_thread_name()

while not check_main_thread(Node):
    print("Node is not running in main thread.")

print("Node is now running in main thread.")Node
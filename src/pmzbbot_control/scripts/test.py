#!/usr/bin/python3

# Import necessary ROS 2 and other Python libraries
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, Point
import tf_transformations
import math
from geometry_msgs.msg import Pose, TransformStamped
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

class pmzbbotTestNode(Node):
    # Constructor of the class
    def __init__(self):
        # Initialize the ROS 2 node
        super().__init__(node_name='pmzbbotTestNode')

        # Create a publisher to publish the velocity commands
        self.cmd_pub = self.create_publisher(Twist, '/pmzb_cmd_vel', 10)
        self.cmd_timer = self.create_timer(0.1, self.cmd_timer_callback)
        self.cmd_vel = Twist()

        self.wheel_vel_sub = self.create_subscription(Twist, '/pmzb_wheel_vel', self.wheel_vel_callback, 10)
        self.wheel_vel_buffer = Twist()
        self.left_wheel_vel = 0.0
        self.right_wheel_vel = 0.0

        self.imu_sub = self.create_subscription(Imu, '/pmzb_imu', self.imu_callback, 10)
        self.imu_buffer = Imu()

    def wheel_vel_callback(self, msg: Twist):
        self.wheel_vel_buffer = msg
        self.left_wheel_vel = msg.angular.x
        self.right_wheel_vel = msg.angular.y
        self.get_logger().info(f'Received wheel velocity data: {self.left_wheel_vel}, {self.right_wheel_vel}')

    def imu_callback(self, msg):
        self.imu_buffer = msg
        # self.get_logger().info(f'Received IMU data: ax={self.imu_buffer.linear_acceleration.x}, ay={self.imu_buffer.linear_acceleration.y}, az={self.imu_buffer.linear_acceleration.z}, gx={self.imu_buffer.angular_velocity.x}, gy={self.imu_buffer.angular_velocity.y}, gz={self.imu_buffer.angular_velocity.z}')
        # self.get_logger().info(f'Received IMU data: {self.imu_buffer}')

    def cmd_timer_callback(self):
        self.cmd_vel.linear.x = 0.25
        self.cmd_vel.angular.y = 0.8
        self.cmd_pub.publish(self.cmd_vel)
        


# Main function to initialize and run the ROS 2 node
def main(args=None):
    rclpy.init(args=args)
    node = pmzbbotTestNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

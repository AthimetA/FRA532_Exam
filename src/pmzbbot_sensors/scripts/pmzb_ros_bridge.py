#!/usr/bin/python3

import os
from signal import signal
import rclpy
from rclpy.node import Node
from ament_index_python import get_package_share_directory
import sys, yaml
import numpy as np

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

from pmzbbot_interfaces.srv import PmzbbotBeginCalibration

class PMZBRosBridge(Node):
    def __init__(self):
        super().__init__('PMZBRosBridge')

        # Read config file
        self.project_name = 'pmzbbot_sensors'
        self.project_path = get_package_share_directory(self.project_name)

        with open(os.path.join(self.project_path, 'config', 'pmzb_ros_bridge_config.yaml'), 'r') as file:
            config = yaml.safe_load(file)
            self.imu_sub_topic = config['IMU_SUB_TOPIC_NAME']
            self.imu_pub_topic = config['IMU_PUB_TOPIC_NAME']
            self.wheel_vel_sub_topic = config['WHEEL_VELOCITY_SUB_TOPIC_NAME']
            self.wheel_odom_pub_topic = config['WHEEL_ODOMETRY_PUB_TOPIC_NAME']
            self.timer_period = 1.0 / config['NODE_TIMER_HZ']

            self._WHEEL_RADIUS = config['WHEEL_RADIUS']
            self._BASE_WIDTH = config['BASE_WIDTH']

            self.odom_frame_id = config['ODOM_FRAME_ID']
            self.base_frame_id = config['BASE_FRAME_ID']
            self.imu_frame_id = config['IMU_FRAME_ID']

            # Calibration
            self.imu_calibration_file_name = config['IMU_CALIBRATION_FILE_NAME']

        # IMU subscriber
        self.imu_sub = self.create_subscription(Imu, self.imu_sub_topic, self.imu_callback, 10)
        # Load the calibration parameters
        imu_config_path = os.path.join(self.project_path, 'config', self.imu_calibration_file_name)
        if os.path.exists(imu_config_path):
            with open(imu_config_path, 'r') as file:
                imu_calibration = yaml.safe_load(file)
                # Load the calibration parameters
                # Offset values
                self.gx_offset = imu_calibration["imu_angular_velocity_mean"][0]
                self.gy_offset = imu_calibration["imu_angular_velocity_mean"][1]
                self.gz_offset = imu_calibration["imu_angular_velocity_mean"][2]
                self.linear_acceleration_x_offset = imu_calibration["imu_linear_acceleration_mean"][0]
                self.linear_acceleration_y_offset = imu_calibration["imu_linear_acceleration_mean"][1]
                self.linear_acceleration_z_offset = imu_calibration["imu_linear_acceleration_mean"][2]
                # Covariance matrices
                self.linear_acceleration_cov = np.array(imu_calibration["imu_linear_acceleration_cov"]).reshape(9)
                self.angular_velocity_cov = np.array(imu_calibration["imu_angular_velocity_cov"]).reshape(9)
        else:
            self.get_logger().info('No calibration data found. Running in normal mode.')
            self.gx_offset = 0.0
            self.gy_offset = 0.0
            self.gz_offset = 0.0
            self.linear_acceleration_x_offset = 0.0
            self.linear_acceleration_y_offset = 0.0
            self.linear_acceleration_z_offset = 0.0
            self.linear_acceleration_cov = np.zeros(9)
            self.angular_velocity_cov = np.zeros(9)

        # IMU publisher
        self.imu_pub = self.create_publisher(Imu, self.imu_pub_topic, 10)

        # Wheel velocity subscriber
        self.wheel_vel_sub = self.create_subscription(Twist, self.wheel_vel_sub_topic, self.wheel_vel_callback, 10)
        self.x = 0.0
        self.y = 0.0
        self.wz = 0.0
        self.pose = np.matrix([0.0, 0.0, 0.0]).T # x, y, theta
        self.pose_cov = np.diag([0.1, 0.1, 0.1])

        # Wheel odometry publisher
        self.wheel_odom_pub = self.create_publisher(Odometry, self.wheel_odom_pub_topic, 10)

        # Timer to publish IMU data and wheel odometry
        self.create_timer(self.timer_period, self.timer_callback)

    def imu_callback(self, msg: Imu):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = self.imu_frame_id
        # Gyroscope data in rad/s
        imu_msg.angular_velocity.x = msg.angular_velocity.x - self.gx_offset
        imu_msg.angular_velocity.y = msg.angular_velocity.y - self.gy_offset
        imu_msg.angular_velocity.z = msg.angular_velocity.z - self.gz_offset
        imu_msg.angular_velocity_covariance = self.angular_velocity_cov
        # # Accelerometer data in m/s^2
        imu_msg.linear_acceleration.x = msg.linear_acceleration.x - self.linear_acceleration_x_offset
        imu_msg.linear_acceleration.y = msg.linear_acceleration.y - self.linear_acceleration_y_offset
        imu_msg.linear_acceleration.z = msg.linear_acceleration.z - self.linear_acceleration_z_offset
        imu_msg.linear_acceleration_covariance = self.linear_acceleration_cov
        # # Publish the IMU data
        # self.get_logger().info(f'rpy: {self.rpy_cal(imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z, imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z)}')
        self.imu_pub.publish(imu_msg)

    def rpy_cal(self, ax, ay, az, gx, gy, gz):
        roll = np.arctan2(ay, az)
        pitch = np.arctan2(-ax, np.sqrt(ay**2 + az**2))
        yaw = np.arctan2(gz, np.sqrt(gx**2 + gy**2))
        return roll, pitch, yaw


    def wheel_vel_callback(self, msg: Twist):
        now = self.get_clock().now()

        # Get the wheel velocities from the joint_states
        wl = msg.angular.x # left wheel velocity
        wr = msg.angular.z # right wheel velocity
        dt = 1/10
        # self.get_logger().info(f'wl: {wl}, wr: {wr}')

        ds = self._WHEEL_RADIUS * (wl + wr) / 2.0 * dt
        dtheta = self._WHEEL_RADIUS * (wr - wl) / self._BASE_WIDTH * dt

        # Update the pose
        self.pose[2] += dtheta
        self.pose[0] += ds * np.cos(self.pose[2])
        self.pose[1] += ds * np.sin(self.pose[2])
        
        pos0 = self.pose[0].item()
        pos1 = self.pose[1].item()
        pos2 = self.pose[2].item()

        # # calculate the linear and angular velocity of the robot
        # dt = 1/10
        # vx = ((self._WHEEL_RADIUS/2) * (wl + wr) ) * dt
        # wz = ((self._WHEEL_RADIUS/self._BASE_WIDTH) * (wr - wl)) * dt

        # if vx != 0:
        #     # calculate distance traveled in x and y
        #     x = np.cos(wz) * vx
        #     y = -np.sin(wz) * vx
        #     # calculate the final position of the robot
        #     self.x = self.x + (np.cos(self.wz) * x - np.sin(self.wz) * y)
        #     self.y = self.y + (np.sin(self.wz) * x + np.cos(self.wz) * y)
        # if wz != 0:
        #     self.wz = self.wz + wz

        # self.pose[0] = self.x
        # self.pose[1] = self.y
        # self.pose[2] = self.wz

        # pos0 = self.x
        # pos1 = self.y
        # pos2 = self.wz
        # ds = vx
        # dtheta = wz

        self.get_logger().info(f'x: {pos0}, y: {pos1}, theta: {pos2}')
        # Covariance matrix
        Fp = np.matrix([[1, 0, -ds*np.sin(self.pose[2])], [0, 1, ds*np.cos(self.pose[2])], [0, 0, 1]])
        Fdrl = np.matrix([[0.5*np.cos(self.pose[2]), 0.5*np.cos(self.pose[2])], [0.5*np.sin(self.pose[2]), 0.5*np.sin(self.pose[2])], [1/self._BASE_WIDTH, -1/self._BASE_WIDTH]])
        self.pose_cov = Fp * self.pose_cov * Fp.T + Fdrl * np.diag([self.linear_acceleration_cov[0], self.linear_acceleration_cov[4]]) * Fdrl.T

        # self.get_logger().info(f'pose_cov: {self.pose_cov}')

        # Publish the wheel odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = self.odom_frame_id
        odom_msg.child_frame_id = self.base_frame_id
        # Position
        odom_msg.pose.pose.position.x = self.pose[0].item()
        odom_msg.pose.pose.position.y = self.pose[1].item()
        odom_msg.pose.pose.position.z = 0.0
        # Orientation
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = np.sin(self.pose[2].item() / 2)
        q.w = np.cos(self.pose[2].item() / 2)
        odom_msg.pose.pose.orientation = q
        # Covariance
        odom_msg.pose.covariance[0] = self.pose_cov[0,0]
        odom_msg.pose.covariance[7] = self.pose_cov[1,1]
        odom_msg.pose.covariance[35] = self.pose_cov[2,2]
        # Velocity
        odom_msg.twist.twist.linear.x = ds
        odom_msg.twist.twist.angular.z = dtheta
        # Publish the odometry
        self.wheel_odom_pub.publish(odom_msg)

    def timer_callback(self):
        pass

    
def main(args=None):
    rclpy.init(args=args)
    node = PMZBRosBridge()
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()

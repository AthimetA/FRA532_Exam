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

from pmzbbot_interfaces.srv import PmzbbotBeginCalibration

class PMZBCalibrationSensor(Node):
    def __init__(self):
        super().__init__('PMZBCalibrationSensor')


        # Read config file
        self.project_name = 'pmzbbot_sensors'
        self.project_path = get_package_share_directory(self.project_name)
        self.config_yaml = os.path.join(self.project_path, 'config', 'pmzb_calibration_sensor_config.yaml')
        self.save_path = "/home/duplicix/FRA532_Exam/src/pmzbbot_sensors/config/"

        with open(self.config_yaml, 'r') as file:
            self.config = yaml.safe_load(file)
            self.imu_topic = self.config['IMU_TOPIC_NAME']
            self.whel_vel_topic = self.config['WHEEL_VELOCITY_TOPIC_NAME']
            self.max_n = self.config['MAX_CALIBRATION_ITERATIONS']
            self.node_calibration_srv_name = self.config['NODE_CALIBRATION_SERVICE_NAME']
        
        # IMU subscriber
        self.sensor_subscriber = self.create_subscription(Imu, self.imu_topic, self.imu_callback, 10)
        self.imu_linear_acceleration_list = np.array([])
        self.imu_angular_velocity_list = np.array([])
        self.imu_n = 0
        self.imu_calibration = False
        self.imu_path = os.path.join(self.save_path, 'imu_calibration.yaml')

        # Wheel velocity subscriber
        self.wheel_velocity_subscriber = self.create_subscription(Twist, self.whel_vel_topic, self.wheel_velocity_callback, 10)
        self.left_wheel_velocity_list = np.array([])
        self.right_wheel_velocity_list = np.array([])
        self.wheel_n = 0
        self.wheel_calibration = False
        self.wheel_path = os.path.join(self.save_path, 'wheel_calibration.yaml')
        
        # Service
        self.srv = self.create_service(PmzbbotBeginCalibration, self.node_calibration_srv_name , self.begin_calibration_callback)
        
    def imu_callback(self,msg: Imu):
        angular_velocity = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        linear_acceleration = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])

        if self.imu_calibration == True and self.imu_n < self.max_n:
            self.imu_angular_velocity_list = np.append(self.imu_angular_velocity_list, angular_velocity)
            self.imu_linear_acceleration_list = np.append(self.imu_linear_acceleration_list, linear_acceleration)
            self.imu_n += 1
            self.get_logger().info(f'IMU calibration: {self.imu_n}/{self.max_n}')
        elif self.imu_calibration == True and self.imu_n == self.max_n:
            self.imu_angular_velocity_list = self.imu_angular_velocity_list.reshape(self.max_n,3)
            self.imu_linear_acceleration_list = self.imu_linear_acceleration_list.reshape(self.max_n,3)
            self.imu_linear_acceleration_mean = np.mean(self.imu_linear_acceleration_list, axis=0)
            self.imu_angular_velocity_mean = np.mean(self.imu_angular_velocity_list, axis=0)
            self.imu_linear_acceleration_cov = np.cov(self.imu_linear_acceleration_list.T)
            self.imu_angular_velocity_cov = np.cov(self.imu_angular_velocity_list.T)
            self.imu_calibration = False
            self.get_logger().info('IMU calibration finished')
            self.get_logger().info(f'IMU linear acceleration mean: {self.imu_linear_acceleration_mean}')
            self.get_logger().info(f'IMU angular velocity mean: {self.imu_angular_velocity_mean}')
            self.get_logger().info(f'IMU linear acceleration covariance: {self.imu_linear_acceleration_cov}')
            self.get_logger().info(f'IMU angular velocity covariance: {self.imu_angular_velocity_cov}')
            # Write to yaml
            self.get_logger().info(f'Writing to {self.imu_path}')
            with open(self.imu_path, 'w') as f:
                data = {'imu_linear_acceleration_mean': self.imu_linear_acceleration_mean.tolist(), 'imu_angular_velocity_mean': self.imu_angular_velocity_mean.tolist(), 'imu_linear_acceleration_cov': self.imu_linear_acceleration_cov.tolist(), 'imu_angular_velocity_cov': self.imu_angular_velocity_cov.tolist()}
                yaml.dump(data, f)
        else:
            pass
        

    def wheel_velocity_callback(self, msg: Twist):
        left_wheel_velocity = msg.angular.x
        right_wheel_velocity = msg.angular.z
        if self.wheel_calibration == True and self.wheel_n < self.max_n:
            self.left_wheel_velocity_list = np.append(self.left_wheel_velocity_list, left_wheel_velocity)
            self.right_wheel_velocity_list = np.append(self.right_wheel_velocity_list, right_wheel_velocity)
            self.wheel_n += 1
            self.get_logger().info(f'Wheel calibration: {self.wheel_n}/{self.max_n}')
        elif self.wheel_calibration == True and self.wheel_n == self.max_n:
            self.left_wheel_velocity_list = self.left_wheel_velocity_list.reshape(self.max_n,1)
            self.right_wheel_velocity_list = self.right_wheel_velocity_list.reshape(self.max_n,1)
            self.left_wheel_velocity_mean = np.mean(self.left_wheel_velocity_list, axis=0)
            self.right_wheel_velocity_mean = np.mean(self.right_wheel_velocity_list, axis=0)
            self.left_wheel_velocity_cov = np.cov(self.left_wheel_velocity_list.T)
            self.right_wheel_velocity_cov = np.cov(self.right_wheel_velocity_list.T)
            self.wheel_calibration = False
            # Write to yaml
            self.get_logger().info('Wheel calibration finished')
            self.get_logger().info(f'Left wheel velocity mean: {self.left_wheel_velocity_mean}')
            self.get_logger().info(f'Right wheel velocity mean: {self.right_wheel_velocity_mean}')
            self.get_logger().info(f'Left wheel velocity covariance: {self.left_wheel_velocity_cov}')
            self.get_logger().info(f'Right wheel velocity covariance: {self.right_wheel_velocity_cov}')
            with open(self.wheel_path, 'w') as f:
                data = {'left_wheel_velocity_mean': self.left_wheel_velocity_mean.tolist(), 'right_wheel_velocity_mean': self.right_wheel_velocity_mean.tolist(), 'left_wheel_velocity_cov': self.left_wheel_velocity_cov.tolist(), 'right_wheel_velocity_cov': self.right_wheel_velocity_cov.tolist()}
                yaml.dump(data, f)
        else:
            pass

    def begin_calibration_callback(self, request: PmzbbotBeginCalibration.Request, response: PmzbbotBeginCalibration.Response):

        self.get_logger().info('Calibration service called')
        self.imu_calibration = request.imu_calibration
        self.wheel_calibration = request.wheel_calibration
        call_back_text = "Sensor calibration: "

        if self.imu_calibration == True:
            self.imu_angular_velocity_list = np.array([])
            self.imu_linear_acceleration_list = np.array([])
            self.imu_n = 0
            call_back_text += "IMU calibration"
        if self.wheel_calibration == True:
            self.left_wheel_velocity_list = np.array([])
            self.right_wheel_velocity_list = np.array([])
            self.wheel_n = 0
            call_back_text += "Wheel calibration"

        response.node_name = self.get_name()
        response.call_back_status = call_back_text
        self.get_logger().info(call_back_text)
        return response

    
def main(args=None):
    rclpy.init(args=args)
    node = PMZBCalibrationSensor()
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()

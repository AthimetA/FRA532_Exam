#ifndef GENERAL_PARAM_H
#define GENERAL_PARAM_H


// General
#define DEG2RAD PI/180
#define RAD2DEG 180/PI
#define RPM2RAD 0.10471975511965977
#define RAD2RPM 9.549296585513721
#define MOTOR_LEFT 0
#define MOTOR_RIGHT 1

// WiFi

// #define WIFI_MODE_ROUTER
#define WIFI_MODE_HOTSPOT

#ifdef WIFI_MODE_HOTSPOT
    #define WIFI_SSID "X2"
    #define WIFI_PASSWORD "qwertyuiop"
    #define AGENT_IP_0 192
    #define AGENT_IP_1 168
    #define AGENT_IP_2 213
    #define AGENT_IP_3 254
#endif

// #ifdef WIFI_MODE_HOTSPOT
//     #define WIFI_SSID "Athimet.A"
//     #define WIFI_PASSWORD "Apthpa4062"
//     #define AGENT_IP_0 172
//     #define AGENT_IP_1 20
//     #define AGENT_IP_2 10
//     #define AGENT_IP_3 2
// #endif

#ifdef WIFI_MODE_ROUTER
    #define WIFI_SSID "AAPA-2.4G"
    #define WIFI_PASSWORD "Athpun4062"
    #define AGENT_IP_0 192
    #define AGENT_IP_1 168
    #define AGENT_IP_2 1
    #define AGENT_IP_3 8
#endif

#define AGENT_PORT 8888

// Serial
#define SERIAL_BAUDRATE 115200
#define GENERAL_DELAY_MS 100

// Motor
#define DYNAMIXEL_SERIAL_BAUDRATE 115200
#define DYNAMIXEL_DIRECTION_PIN 4
#define DYNAMIXEL_BIT_2_RPM 0.916
#define DYNAMIXEL_MOTOR_LEFT_ID 1
#define DYNAMIXEL_MOTOR_RIGHT_ID 2
#define WHEEL_VELOCITY_TOPIC_NAME "/pmzb_uros/wheel_vel"
#define WHEEL_VELOCITY_MSG_TYPE std_msgs__msg__Float64
#define DYNAMIXEL_CMD_VEL_TOPIC_NAME "/pmzb_cmd_vel"
#define DYNAMIXEL_CMD_VEL_MSG_TYPE geometry_msgs__msg__Twist
#define DYNAMIXEL_CMD_VEL_WHEEL_CMD_CAP 80 // RPM

// IMU
#define IMU_ADDRESS 0x68
#define GAVITY 9.80665
#define IMU_ACC_X_DIRECTIONAL_CORRECTION -1
#define IMU_ACC_Y_DIRECTIONAL_CORRECTION -1
#define IMU_ACC_Z_DIRECTIONAL_CORRECTION -1
#define IMU_TOPIC_NAME "/pmzb_uros/imu"
#define IMU_MSG_TYPE sensor_msgs__msg__Imu

// ROS
#define SENSOR_NODE_NAME "PMZB_sensor"
#define TIMER_PERIOD_MS 50
#define ROBOT_BASE_WIDTH 0.161 // m
#define ROBOT_WHEEL_RADIUS 0.034 // m

// RPM Linear Regression
#define RPM_LEFT_SLOPE 1.023715
#define RPM_LEFT_INTERCEPT -2.195521
#define RPM_RIGHT_SLOPE 1.025501
#define RPM_RIGHT_INTERCEPT -2.285550


#endif

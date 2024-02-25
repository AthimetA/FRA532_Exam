#include <Arduino.h>

// ROS2
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// General parameters
#include <General_params.h>

// Dynamixel motor
#include <Motor.h>
#include <geometry_msgs/msg/twist.h>

// MPU9250 sensor 
#include <MPU9250.h>
#include <sensor_msgs/msg/imu.h>

// ----------------- Global variables -----------------
// Ros2
rclc_executor_t executor_pub; // Executor for publisher
rclc_executor_t executor_sub; // Executor for subscriber
rclc_support_t support; // Support for micro-ROS
rcl_allocator_t allocator; // Allocator for micro-ROS
rcl_node_t node; // Node for micro-ROS
rcl_timer_t timer; // Timer for control loop

// CMD_VEL Subscriber
rcl_subscription_t cmd_vel_sub;
geometry_msgs__msg__Twist cmd_vel_msg;

// Motor Publisher
rcl_publisher_t wheel_velocity_publisher; // Publisher for wheel velocity
geometry_msgs__msg__Twist wheel_velocity_msg; // Message for wheel velocity

// IMU Publisher
rcl_publisher_t imu_publisher; // Publisher for IMU
sensor_msgs__msg__Imu imu_msg; // Message for IMU

// MPU9250 sensor
MPU9250 mpu; // MPU9250 sensor

// Motor --> Already defined in Motor.h
// Motor motor; // Motor 

// Declare global variables for motor velocity, encoder counts, etc.
// Motor variables
int dynamixel_wheel_left_raw = 0;
int dynamixel_wheel_right_raw = 0;
float wheel_left_data = 0;
float wheel_right_data = 0;

// IMU variables
float ax, ay, az, gx, gy, gz; // Variables for roll, pitch, yaw, acceleration and angular velocity

// Time variables
long preMilliseconds = 0;
float dt = 0;

long main_loop_counter_ms = 0;

// ----------------- Function declarations -----------------
void timer_callback(rcl_timer_t * timer, int64_t last_call_time);
void readDynamixelWheel(float &wheel_left_data, float &wheel_right_data);
void readIMU(float &ax, float &ay, float &az, float &gx, float &gy, float &gz);
void cmd_vel_callback(const void * msgin);


#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

// ----------------- Main code -----------------

void setup() {

    // Serial initialization
    Serial.begin(SERIAL_BAUDRATE);

    // Motor initialization
    Motor.begin(DYNAMIXEL_SERIAL_BAUDRATE, DYNAMIXEL_DIRECTION_PIN, &Serial2);

    // MPU9250 sensor initialization
    Wire.begin();
    MPU9250Setting mpu_setting;
    mpu_setting.accel_fs_sel = ACCEL_FS_SEL::A16G;
    mpu_setting.gyro_fs_sel = GYRO_FS_SEL::G2000DPS;
    mpu_setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
    mpu_setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_200HZ;
    mpu_setting.gyro_fchoice = 0x03;
    mpu_setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;
    mpu_setting.accel_fchoice = 0x01;
    mpu_setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;

    if (!mpu.setup(IMU_ADDRESS,mpu_setting)) { 
        while (1) {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(1000);
        }
    }

    // WiFi initialization
    IPAddress agent_ip(AGENT_IP_0,AGENT_IP_1,AGENT_IP_2,AGENT_IP_3);
    size_t agent_port = AGENT_PORT;

    char ssid[] = WIFI_SSID;
    char password[] = WIFI_PASSWORD;
    set_microros_wifi_transports(ssid,password,agent_ip,agent_port);

    delay(GENERAL_DELAY_MS);

    // Ros2 initialization
    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "pmzb_node", "", &support));

    // Publisher initialization
    RCCHECK(rclc_publisher_init_default(
      &wheel_velocity_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      WHEEL_VELOCITY_TOPIC_NAME));

    RCCHECK(rclc_publisher_init_default(
        &imu_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        IMU_TOPIC_NAME));

    // Timer initialization
    const unsigned int timer_timeout = TIMER_PERIOD_MS;
    RCCHECK(rclc_timer_init_default(
      &timer,
      &support,
      RCL_MS_TO_NS(timer_timeout),
      timer_callback));

    // Subscriber initialization
    RCCHECK(rclc_subscription_init_default(
      &cmd_vel_sub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      DYNAMIXEL_CMD_VEL_TOPIC_NAME));

    // Executor initialization
    RCCHECK(rclc_executor_init(&executor_pub, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor_pub, &timer));
    RCCHECK(rclc_executor_init(&executor_sub, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor_sub,&cmd_vel_sub,&cmd_vel_msg,&cmd_vel_callback, ON_NEW_DATA));

    Serial.println("Micro-ROS PMZB node has been initialized");

}

void loop() {
    // spin up the executor

    if (millis() - main_loop_counter_ms >= 50) {
        // Serial.println("Main loop");
        RCSOFTCHECK(rclc_executor_spin_some(&executor_pub, RCL_MS_TO_NS(50)));
        RCSOFTCHECK(rclc_executor_spin_some(&executor_sub, RCL_MS_TO_NS(50)));
        main_loop_counter_ms = millis();
    }
    
}

// ----------------- End of main code -----------------

// ----------------- Function definitions -----------------

// Timer callback
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    // Get time
    long currentMilliseconds = millis();
    dt = (currentMilliseconds - preMilliseconds) / 1000.0;
    preMilliseconds = currentMilliseconds;

    // IMU sensor
    readIMU(ax, ay, az, gx, gy, gz);
    imu_msg.linear_acceleration.x = ax;
    imu_msg.linear_acceleration.y = ay;
    imu_msg.linear_acceleration.z = az;
    imu_msg.angular_velocity.x = gx;
    imu_msg.angular_velocity.y = gy;
    imu_msg.angular_velocity.z = gz;
    // Publish IMU
    RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));

    // Motor Read
    readDynamixelWheel(wheel_left_data, wheel_right_data);
    // Publish wheel velocity
    wheel_velocity_msg.angular.x = wheel_left_data;
    wheel_velocity_msg.angular.z = wheel_right_data;
    RCSOFTCHECK(rcl_publish(&wheel_velocity_publisher, &wheel_velocity_msg, NULL));

}

// Read motor data
void readDynamixelWheel(float &wheel_left_data, float &wheel_right_data) {
    dynamixel_wheel_left_raw = Motor.readSpeed(DYNAMIXEL_MOTOR_LEFT_ID);
    dynamixel_wheel_right_raw = Motor.readSpeed(DYNAMIXEL_MOTOR_RIGHT_ID);

    // Serial.println("Left: " + String(dynamixel_wheel_left_raw) + " Right: " + String(dynamixel_wheel_right_raw));

    if (dynamixel_wheel_left_raw > 1023) {
        wheel_left_data = (dynamixel_wheel_left_raw - 1024) * -1;
    } else {
        wheel_left_data = dynamixel_wheel_left_raw;
    }

    if (dynamixel_wheel_right_raw < 1024) {
        wheel_right_data = dynamixel_wheel_right_raw * -1;
    } else {
        wheel_right_data = dynamixel_wheel_right_raw - 1024;
    }

    // Convert raw data to RPM
    wheel_left_data = wheel_left_data * DYNAMIXEL_BIT_2_RPM;
    wheel_right_data = wheel_right_data * DYNAMIXEL_BIT_2_RPM;

    // Convert RPM to rad/s
    wheel_left_data = wheel_left_data * RPM2RAD;
    wheel_right_data = wheel_right_data * RPM2RAD;
}

// Read IMU data
void readIMU(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
    if (mpu.update()) {
        // Linear acceleration
        ax = mpu.getAccX() * IMU_ACC_X_DIRECTIONAL_CORRECTION;
        ay = mpu.getAccY() * IMU_ACC_Y_DIRECTIONAL_CORRECTION;
        az = mpu.getAccZ() * IMU_ACC_Z_DIRECTIONAL_CORRECTION;

        // Convert g to m/s^2
        ax = ax * GAVITY;
        ay = ay * GAVITY;
        az = az * GAVITY;

        // Angular velocity
        gx = mpu.getGyroX();
        gy = mpu.getGyroY();
        gz = mpu.getGyroZ();

        // Convert deg/s to rad/s
        gx = gx * DEG2RAD;
        gy = gy * DEG2RAD;
        gz = gz * DEG2RAD;

    }
}

// CMD_VEL callback
void cmd_vel_callback(const void * msgin) {
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
    float robot_linear_velocity = msg->linear.x;
    float robot_angular_velocity = msg->angular.z;

    Serial.println("Robot linear velocity: " + String(robot_linear_velocity) + " m/s"+ " Robot angular velocity: " + String(robot_angular_velocity) + " rad/s");

    // Convert robot velocity to wheel velocity
    float left_wheel_velocity = (robot_linear_velocity/ROBOT_WHEEL_RADIUS) - (robot_angular_velocity * ROBOT_BASE_WIDTH / (2*ROBOT_WHEEL_RADIUS));
    float right_wheel_velocity = (robot_linear_velocity/ROBOT_WHEEL_RADIUS) + (robot_angular_velocity * ROBOT_BASE_WIDTH / (2*ROBOT_WHEEL_RADIUS));
    
    // Convert wheel velocity to RPM
    float left_wheel_rpm = left_wheel_velocity * RAD2RPM;
    float right_wheel_rpm = right_wheel_velocity * RAD2RPM;

    // float left_wheel_rpm = msg->linear.x;
    // float right_wheel_rpm = msg->angular.z;

    Serial.println("Left wheel RPM: " + String(left_wheel_rpm) + " Right wheel RPM: " + String(right_wheel_rpm));

    // Cap the wheel velocity
    if (left_wheel_rpm > DYNAMIXEL_CMD_VEL_WHEEL_CMD_CAP) {
        left_wheel_rpm = DYNAMIXEL_CMD_VEL_WHEEL_CMD_CAP;
    } else if (left_wheel_rpm < -DYNAMIXEL_CMD_VEL_WHEEL_CMD_CAP) {
        left_wheel_rpm = -DYNAMIXEL_CMD_VEL_WHEEL_CMD_CAP;
    }

    if (left_wheel_rpm < 0) {
        Motor.turnWheel(DYNAMIXEL_MOTOR_LEFT_ID, RIGHT, abs(left_wheel_rpm));
    } else {
        Motor.turnWheel(DYNAMIXEL_MOTOR_LEFT_ID, LEFT, left_wheel_rpm);
    }

    if (right_wheel_rpm < 0) {
        Motor.turnWheel(DYNAMIXEL_MOTOR_RIGHT_ID, LEFT, abs(right_wheel_rpm));
    } else {
        Motor.turnWheel(DYNAMIXEL_MOTOR_RIGHT_ID, RIGHT, right_wheel_rpm);
    }
}

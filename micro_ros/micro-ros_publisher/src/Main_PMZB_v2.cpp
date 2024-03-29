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
rclc_executor_t executor_pub_imu; // Executor for IMU publisher
rclc_executor_t executor_pub_wheel_velocity; // Executor for wheel velocity publisher
rclc_executor_t executor_sub; // Executor for subscriber
rclc_support_t support; // Support for micro-ROS
rcl_allocator_t allocator; // Allocator for micro-ROS
rcl_node_t node; // Node for micro-ROS

// CMD_VEL Subscriber
rcl_subscription_t cmd_vel_sub;
geometry_msgs__msg__Twist cmd_vel_msg;
float robot_linear_velocity_buffer = 0.0;
float robot_angular_velocity_buffer = 0.0;
float left_wheel_rpm = 0;
float right_wheel_rpm = 0;

// Motor Publisher
rcl_publisher_t wheel_velocity_publisher; // Publisher for wheel velocity
geometry_msgs__msg__Twist wheel_velocity_msg; // Message for wheel velocity
rcl_timer_t timer_wheel_velocity; // Timer for wheel velocity

// IMU Publisher
rcl_publisher_t imu_publisher; // Publisher for IMU
geometry_msgs__msg__Twist imu_msg; // Message for IMU
rcl_timer_t timer_imu; // Timer for IMU

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
void timer_imu_callback(rcl_timer_t * timer, int64_t last_call_time);
void timer_wheel_velocity_callback(rcl_timer_t * timer, int64_t last_call_time);
void readDynamixelWheel(float &wheel_left_data, float &wheel_right_data);
void readIMU(float &ax, float &ay, float &az, float &gx, float &gy, float &gz);
void cmd_vel_callback(const void * msgin);
void DynamixelCommandVelocity(float &left_wheel_rpm, float &right_wheel_rpm, float &left_wheel_data, float &right_wheel_data);
#ifdef LINEAR_REGRESSION_FLAG
void LinearRegression(float &left_wheel_rpm, float &right_wheel_rpm);
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

// ----------------- Main code -----------------

void setup() {

    // Serial initialization
    Serial.begin(SERIAL_BAUDRATE);

    // WiFi initialization
    IPAddress agent_ip(AGENT_IP_0,AGENT_IP_1,AGENT_IP_2,AGENT_IP_3);
    size_t agent_port = AGENT_PORT;

    char ssid[] = WIFI_SSID;
    char password[] = WIFI_PASSWORD;
    set_microros_wifi_transports(ssid,password,agent_ip,agent_port);

    Serial.println("WiFi has been initialized");
    delay(GENERAL_DELAY_MS);

    // Motor initialization
    Motor.begin(DYNAMIXEL_SERIAL_BAUDRATE, DYNAMIXEL_DIRECTION_PIN, &Serial2);

    Serial.println("Dynamixel motor has been initialized");

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
            delay(GENERAL_DELAY_MS);
        }
    }

    Serial.println("MPU has been initialized");

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
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        IMU_TOPIC_NAME));

    // Timer initialization for wheel velocity
    const unsigned int timer_timeout_wheel_velocity = TIMER_PERIOD_MS;
    RCCHECK(rclc_timer_init_default(
      &timer_wheel_velocity,
      &support,
      RCL_MS_TO_NS(timer_timeout_wheel_velocity),
      timer_wheel_velocity_callback));

    // Timer initialization for IMU
    const unsigned int timer_timeout_imu = TIMER_PERIOD_MS;
    RCCHECK(rclc_timer_init_default(
      &timer_imu,
      &support,
      RCL_MS_TO_NS(timer_timeout_imu),
      timer_imu_callback));

    // Subscriber initialization
    RCCHECK(rclc_subscription_init_default(
      &cmd_vel_sub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      DYNAMIXEL_CMD_VEL_TOPIC_NAME));

    // Executor initialization
    RCCHECK(rclc_executor_init(&executor_pub_imu, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor_pub_imu, &timer_imu));
    RCCHECK(rclc_executor_init(&executor_pub_wheel_velocity, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor_pub_wheel_velocity, &timer_wheel_velocity));
    RCCHECK(rclc_executor_init(&executor_sub, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor_sub,&cmd_vel_sub,&cmd_vel_msg,&cmd_vel_callback, ON_NEW_DATA));

    Serial.println("Micro-ROS PMZB node has been initialized");

}

void loop() {
    // spin up the executor
    RCSOFTCHECK(rclc_executor_spin_some(&executor_pub_imu, RCL_MS_TO_NS(10)));
    RCSOFTCHECK(rclc_executor_spin_some(&executor_pub_wheel_velocity, RCL_MS_TO_NS(10)));
    RCSOFTCHECK(rclc_executor_spin_some(&executor_sub, RCL_MS_TO_NS(10)));

    if (millis() - main_loop_counter_ms >= 50) {

        DynamixelCommandVelocity(left_wheel_rpm, right_wheel_rpm, wheel_left_data, wheel_right_data);
        

        // Cap the wheel velocity
        if (left_wheel_rpm > DYNAMIXEL_CMD_VEL_WHEEL_CMD_CAP) {
            left_wheel_rpm = DYNAMIXEL_CMD_VEL_WHEEL_CMD_CAP;
        } else if (left_wheel_rpm < -DYNAMIXEL_CMD_VEL_WHEEL_CMD_CAP) {
            left_wheel_rpm = -DYNAMIXEL_CMD_VEL_WHEEL_CMD_CAP;
        }

        if (right_wheel_rpm > DYNAMIXEL_CMD_VEL_WHEEL_CMD_CAP) {
            right_wheel_rpm = DYNAMIXEL_CMD_VEL_WHEEL_CMD_CAP;
        } else if (right_wheel_rpm < -DYNAMIXEL_CMD_VEL_WHEEL_CMD_CAP) {
            right_wheel_rpm = -DYNAMIXEL_CMD_VEL_WHEEL_CMD_CAP;
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

        main_loop_counter_ms = millis();
    }
    
}


// ----------------- End of main code -----------------

// ----------------- Function definitions -----------------

// Timer callback for wheel velocity
void timer_wheel_velocity_callback(rcl_timer_t * timer, int64_t last_call_time) {
    // Motor Read
    readDynamixelWheel(wheel_left_data, wheel_right_data);
    // Publish wheel velocity
    wheel_velocity_msg.angular.x = wheel_left_data;
    wheel_velocity_msg.angular.z = wheel_right_data;
    RCSOFTCHECK(rcl_publish(&wheel_velocity_publisher, &wheel_velocity_msg, NULL));

}

// Timer callback for IMU
void timer_imu_callback(rcl_timer_t * timer, int64_t last_call_time) {

    // IMU sensor
    readIMU(ax, ay, az, gx, gy, gz);
    imu_msg.linear.x = ax;
    imu_msg.linear.y = ay;
    imu_msg.linear.z = az;
    imu_msg.angular.x = gx;
    imu_msg.angular.y = gy;
    imu_msg.angular.z = gz;
    // Publish IMU
    RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));

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
    
    // Serial.println("Read left wheel RPM: " + String(wheel_left_data) + " Read right wheel RPM: " + String(wheel_right_data));

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
    else {
        Serial.println("Failed to read IMU data");
    }
}

// CMD_VEL callback
void cmd_vel_callback(const void * msgin) {
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
    robot_linear_velocity_buffer = msg->linear.x;
    robot_angular_velocity_buffer = msg->angular.z;
}

// Dynamixel command velocity
void DynamixelCommandVelocity(float &left_wheel_rpm, float &right_wheel_rpm, float &left_wheel_data, float &right_wheel_data) {
    // Get time
    // long currentMilliseconds = millis();
    // dt = (currentMilliseconds - preMilliseconds) / 1000.0;
    // preMilliseconds = currentMilliseconds;
    // Serial.println("dt: " + String(dt) + " s");

    // Convert robot velocity to wheel velocity
    float left_wheel_velocity = (robot_linear_velocity_buffer/ROBOT_WHEEL_RADIUS) - (robot_angular_velocity_buffer * ROBOT_BASE_WIDTH / (2*ROBOT_WHEEL_RADIUS));
    float right_wheel_velocity = (robot_linear_velocity_buffer/ROBOT_WHEEL_RADIUS) + (robot_angular_velocity_buffer * ROBOT_BASE_WIDTH / (2*ROBOT_WHEEL_RADIUS));
    
    // Convert wheel velocity to RPM
    left_wheel_rpm = left_wheel_velocity * RAD2RPM;
    right_wheel_rpm = right_wheel_velocity * RAD2RPM;

    // Serial.println("Left wheel RPM: " + String(left_wheel_rpm) + " Right wheel RPM: " + String(right_wheel_rpm));

    // Serial.println("Before linear regression: Left wheel RPM: " + String(left_wheel_rpm) + " Right wheel RPM: " + String(right_wheel_rpm));

    #ifdef LINEAR_REGRESSION_FLAG

    // Linear regression
    LinearRegression(left_wheel_rpm, right_wheel_rpm);

    // Serial.println("After linear regression: Left wheel RPM: " + String(left_wheel_rpm) + " Right wheel RPM: " + String(right_wheel_rpm));

    #endif
}

#ifdef LINEAR_REGRESSION_FLAG

void LinearRegression(float &left_wheel_rpm, float &right_wheel_rpm) {
    // Linear regression
    // y = mx + c

    // Left wheel
    if (left_wheel_rpm < LR_ZONE_1_START) { // Out of range
        left_wheel_rpm = left_wheel_rpm;
    } else if (left_wheel_rpm >= LR_ZONE_1_START && left_wheel_rpm <= LR_ZONE_1_END) {
        left_wheel_rpm = (left_wheel_rpm - RPM_LEFT_ZONE_1_INTERCEPT) / RPM_LEFT_ZONE_1_SLOPE;
    } else if (left_wheel_rpm > LR_ZONE_1_END && left_wheel_rpm <= LR_ZONE_2_START) {
        left_wheel_rpm = (left_wheel_rpm - RPM_LEFT_ZONE_2_INTERCEPT) / RPM_LEFT_ZONE_2_SLOPE;
    } else { // Out of range
        left_wheel_rpm = left_wheel_rpm;
    }

    // Right wheel
    if (right_wheel_rpm < LR_ZONE_1_START) { // Out of range
        right_wheel_rpm = right_wheel_rpm;
    } else if (right_wheel_rpm >= LR_ZONE_1_START && right_wheel_rpm <= LR_ZONE_1_END) {
        right_wheel_rpm = (right_wheel_rpm - RPM_RIGHT_ZONE_1_INTERCEPT) / RPM_RIGHT_ZONE_1_SLOPE;
    } else if (right_wheel_rpm > LR_ZONE_1_END && right_wheel_rpm <= LR_ZONE_2_START) {
        right_wheel_rpm = (right_wheel_rpm - RPM_RIGHT_ZONE_2_INTERCEPT) / RPM_RIGHT_ZONE_2_SLOPE;
    } else { // Out of range
        right_wheel_rpm = right_wheel_rpm;
    }
}

#endif

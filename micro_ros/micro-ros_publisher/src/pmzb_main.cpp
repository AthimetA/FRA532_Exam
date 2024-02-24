// #include <Arduino.h>
// #include <micro_ros_platformio.h>

// #include <rcl/rcl.h>
// #include <rclc/rclc.h>
// #include <rclc/executor.h>

// #include <std_msgs/msg/int32.h>
// #include <geometry_msgs/msg/twist.h>

// #include <General_params.h>
// #include <Motor.h>
// #include "MPU9250.h"

// MPU9250 mpu;

// // Declare global variables for motor velocity, encoder counts, etc.
// int velocity = 20;
// int dynamixel_wheel_left_raw = 0;
// int dynamixel_wheel_right_raw = 0;
// long preMilliseconds = 0;
// float dt = 0;

// // Variables for phase, offset and position calculations
// float lastPhase_left = 0;
// int offset_left = 0;

// float lastPhase_right = 0;
// int offset_right = 0;

// // Publisher
// rcl_publisher_t wheel_velocity_publisher;
// geometry_msgs__msg__Twist wheel_velocity_msg;

// rclc_executor_t executor;
// rclc_support_t support;
// rcl_allocator_t allocator;
// rcl_node_t node;
// rcl_timer_t timer;

// #define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
// #define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// // Error handle loop
// void error_loop() {
//   while(1) {
//     delay(100);
//   }
// }
// void readDynamixelWheel(float &wheel_left_data, float &wheel_right_data) {
//   dynamixel_wheel_left_raw = Motor.readSpeed(1);
//   dynamixel_wheel_right_raw = Motor.readSpeed(2);


//   if (dynamixel_wheel_left_raw > 1023) {
    
//     wheel_left_data = (dynamixel_wheel_left_raw - 1024) * -1;
//   } else {
//     wheel_left_data = dynamixel_wheel_left_raw;
//   }

//   if (dynamixel_wheel_right_raw < 1024) {
//     wheel_right_data = dynamixel_wheel_right_raw * -1;
//   } else {
//     wheel_right_data = dynamixel_wheel_right_raw - 1024;
//   }

//   // Convert raw data to RPM
//   wheel_left_data = wheel_left_data * DYNAMIXEL_BIT_2_RPM;
//   wheel_right_data = wheel_right_data * DYNAMIXEL_BIT_2_RPM;

//   // Convert RPM to rad/s
//   // wheel_left_data = wheel_left_data * RPM2RAD;
//   // wheel_right_data = wheel_right_data * RPM2RAD;
// }


// void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
//   RCLC_UNUSED(last_call_time);
//   if (timer != NULL) {

//     // Read current motor positions
//     float wheel_left_data = 0;
//     float wheel_right_data = 0;

//     readDynamixelWheel(wheel_left_data, wheel_right_data);

//     wheel_velocity_msg.linear.x = wheel_left_data;
//     wheel_velocity_msg.linear.y = wheel_right_data;

//     RCSOFTCHECK(rcl_publish(&wheel_velocity_publisher, &wheel_velocity_msg, NULL));
//   }
// }

// void setup() {
//   // Configure serial transport
//   Serial.begin(115200);
//   Serial.println("ROS Sensor node started");

//   // Adding Wifi
//   IPAddress agent_ip(AGENT_IP_0,AGENT_IP_1,AGENT_IP_2,AGENT_IP_3);
//   size_t agent_port = AGENT_PORT;

//   char ssid[] = WIFI_SSID;
//   char password[] = WIFI_PASSWORD;
//   Serial.println("Connecting to WiFi and Initialize micro-ROS...");
//   set_microros_wifi_transports(ssid,password,agent_ip,agent_port);
//   delay(2000);

//   // // IMU
//   // Serial.println("Initialize IMU...");
//   // Wire.begin(); // I2C Master
//   // MPU9250Setting setting;

//   // setting.accel_fs_sel = ACCEL_FS_SEL::A16G;
//   // setting.gyro_fs_sel = GYRO_FS_SEL::G2000DPS;
//   // setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
//   // setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_200HZ;
//   // setting.gyro_fchoice = 0x03;
//   // setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;
//   // setting.accel_fchoice = 0x01;
//   // setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;

//   // if (!mpu.setup(0x68, setting)) {  // change to your own address
//   //     while (1) {
//   //         Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
//   //         delay(2000);
//   //         Serial.println("MPU connection retrying...");
//   //     }
//   // }
//   // Serial.println("MPU connection successful!");

//   // Serial.print("Yaw, Pitch, Roll: ");
//   // Serial.print(mpu.getYaw(), 2);
//   // Serial.print(", ");
//   // Serial.print(mpu.getPitch(), 2);
//   // Serial.print(", ");
//   // Serial.println(mpu.getRoll(), 2);

//   allocator = rcl_get_default_allocator();

//   //create init_options
//   RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

//   // create node
//   RCCHECK(rclc_node_init_default(&node, NODE_NAME, "", &support));

//   // create publisher
//   RCCHECK(rclc_publisher_init_default(
//     &wheel_velocity_publisher,
//     &node,
//     ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
//     WHEEL_VELOCITY_TOPIC_NAME));

//   // create timer,
//   const unsigned int timer_timeout = 1000;
//   RCCHECK(rclc_timer_init_default(
//     &timer,
//     &support,
//     RCL_MS_TO_NS(timer_timeout),
//     timer_callback));

//   // create executor
//   RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
//   RCCHECK(rclc_executor_add_timer(&executor, &timer));

//   Motor.begin(MOTOR_BAUDRATE, MOTOR_DIRECTION_PIN, &Serial2);

//   // Read initial motor positions to set offsets
//   offset_left = Motor.readPosition(1);
//   offset_right = -Motor.readPosition(2);
//   lastPhase_left = offset_left;
//   lastPhase_right = offset_right;

//   RCSOFTCHECK(rclc_executor_spin(&executor));

//   Motor.turnWheel(1, LEFT, 0);
//   // Motor.turnWheel(2, RIGHT, 10);

// }


// void loop() {

// }
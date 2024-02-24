// // #include <Arduino.h>
// // #include <micro_ros_platformio.h>

// // #include <rcl/rcl.h>
// // #include <rclc/rclc.h>
// // #include <rclc/executor.h>

// // #include <std_msgs/msg/int32.h>


// // rcl_publisher_t publisher;
// // std_msgs__msg__Int32 msg;

// // rclc_executor_t executor;
// // rclc_support_t support;
// // rcl_allocator_t allocator;
// // rcl_node_t node;
// // rcl_timer_t timer;

// // #define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
// // #define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// // // Error handle loop
// // void error_loop() {
// //   while(1) {
// //     delay(100);
// //   }
// // }

// // void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
// //   RCLC_UNUSED(last_call_time);
// //   if (timer != NULL) {
// //     RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
// //     msg.data++;
// //   }
// // }

// // void setup() {
// //   // Configure serial transport
// //   Serial.begin(115200);
// //   IPAddress agent_ip(192,168,213,254); // Mobile Hotspot
// //   // IPAddress agent_ip(192,168,1,8); // Home Wifi
// //   size_t agent_port = 8888;

// //   char ssid[] = "X2";
// //   char password[] = "qwertyuiop";
// //   // char ssid[] = "AAPA-2.4G";
// //   // char password[] = "Athpun4062";

// //   Serial.println("Connecting to WiFi and Initialize micro-ROS...");
// //   set_microros_wifi_transports(ssid,password,agent_ip,agent_port);
// //   delay(2000);

// //   allocator = rcl_get_default_allocator();

// //   //create init_options
// //   RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

// //   // create node
// //   RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

// //   // create publisher
// //   RCCHECK(rclc_publisher_init_default(
// //     &publisher,
// //     &node,
// //     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
// //     "micro_ros_platformio_node_publisher"));

// //   // create timer,
// //   const unsigned int timer_timeout = 1000;
// //   RCCHECK(rclc_timer_init_default(
// //     &timer,
// //     &support,
// //     RCL_MS_TO_NS(timer_timeout),
// //     timer_callback));

// //   // create executor
// //   RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
// //   RCCHECK(rclc_executor_add_timer(&executor, &timer));

// //   msg.data = 0;
// // }

// // void loop() {
// //   delay(100);
// //   RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
// // }

// #include <Arduino.h>
// #include <micro_ros_platformio.h>
// #include "MPU9250.h"
// #include <General_params.h>

// MPU9250 mpu;

// void setup() {
//     Serial.begin(115200);
//     Wire.begin();

//     // WiFi initialization
//     IPAddress agent_ip(AGENT_IP_0,AGENT_IP_1,AGENT_IP_2,AGENT_IP_3);
//     size_t agent_port = AGENT_PORT;

//     char ssid[] = WIFI_SSID;
//     char password[] = WIFI_PASSWORD;
//     set_microros_wifi_transports(ssid,password,agent_ip,agent_port);

//     delay(2000);

//     MPU9250Setting setting;
//     setting.accel_fs_sel = ACCEL_FS_SEL::A16G;
//     setting.gyro_fs_sel = GYRO_FS_SEL::G2000DPS;
//     setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
//     setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_200HZ;
//     setting.gyro_fchoice = 0x03;
//     setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;
//     setting.accel_fchoice = 0x01;
//     setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;

//     if (!mpu.setup(0x68, setting)) {  // change to your own address
//         while (1) {
//             Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
//             delay(5000);
//         }
//     }
// }

// void print_accxyz() {
//     Serial.print("AccX, AccY, AccZ: ");
//     Serial.print(mpu.getAccX() *-1, 2);
//     Serial.print(", ");
//     Serial.print(mpu.getAccY()*-1, 2);
//     Serial.print(", ");
//     Serial.println(mpu.getAccZ()*-1, 2);
// }

// void print_roll_pitch_yaw() {
//     Serial.print("Yaw, Pitch, Roll: ");
//     Serial.print(mpu.getYaw() * -1, 2);
//     Serial.print(", ");
//     Serial.print(mpu.getPitch() * -1, 2);
//     Serial.print(", ");
//     Serial.println(mpu.getRoll(), 2);
// }

// void print_gyroxzyz() {
//     Serial.print("GyroX, GyroY, GyroZ: ");
//     Serial.print(mpu.getGyroX(), 2);
//     Serial.print(", ");
//     Serial.print(mpu.getGyroY(), 2);
//     Serial.print(", ");
//     Serial.println(mpu.getGyroZ(), 2);

// }

// void loop() {
//     if (mpu.update()) {
//         print_roll_pitch_yaw();
//         print_accxyz();
//         print_gyroxzyz();
//     }
// }


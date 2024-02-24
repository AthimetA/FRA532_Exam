// #include <Arduino.h>
// #include "Motor.h" // Include Arduino and Motor library headers
// #include <micro_ros_platformio.h>
// #include <General_params.h>

// #define DirectionPin 4 // Define pin for motor direction
// #define BaudRate 115200 // Define baud rate for serial communication

// // Declare global variables for motor velocity, encoder counts, etc.
// int velocity = 20;
// int count_left = 0;
// int count_right = 0;
// long preMilliseconds = 0;
// float dt = 0;

// // Variables for phase, offset and position calculations
// float lastPhase_left = 0;
// int offset_left = 0;

// float lastPhase_right = 0;
// int offset_right = 0;



// void setup() 
// {
//   // Initialize motor control and serial communication
//   Motor.begin(BaudRate, DirectionPin, &Serial2);
//   Serial.begin(SERIAL_BAUDRATE);
//   // Adding Wifi
//   IPAddress agent_ip(AGENT_IP_0,AGENT_IP_1,AGENT_IP_2,AGENT_IP_3);
//   size_t agent_port = AGENT_PORT;

//   char ssid[] = WIFI_SSID;
//   char password[] = WIFI_PASSWORD;
//   Serial.println("Connecting to WiFi and Initialize micro-ROS...");
//   set_microros_wifi_transports(ssid,password,agent_ip,agent_port);

//   delay(2000);
//   // Read initial motor positions to set offsets
//   offset_left = Motor.readPosition(1);
//   offset_right = -Motor.readPosition(2);
//   lastPhase_left = offset_left;
//   lastPhase_right = offset_right;

//   Motor.turnWheel(1, LEFT, 0);
//   Motor.turnWheel(2, LEFT, 0);
// }


// void loop() 
// {
//   // Execute code block every 30 milliseconds
//   if (millis() - preMilliseconds >= 30)
//   {
//     // Calculate time elapsed since last loop iteration
//     dt = (millis() - preMilliseconds) / 1000.0;
//     preMilliseconds = millis();

//     // Read current motor positions
//     count_left = Motor.readSpeed(1);
//     count_right = -Motor.readSpeed(2);

//     // Send command to Dynamixel
//     // Motor.turnWheel(1, LEFT, 10);
//     // Motor.turnWheel(2, RIGHT, 10);
//     Serial.println(count_left);
//     // Serial.println(count_right);

//   }
// }


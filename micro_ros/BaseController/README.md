# Robotics Workshop: Navigating a Two-Wheeled Robot

Welcome to our Robotics Workshop! In this series, we'll explore various fundamental concepts in robotics using a two-wheeled robot platform. Each workshop focuses on a specific aspect of robotics, from motor control and sensor data processing to navigation algorithms.

## Prerequisites

- Basic understanding of C++ and Arduino programming.
- Arduino IDE installed.
- A two-wheeled robot with an Arduino-compatible controller and motor encoders.

## Setup

1. Clone this repository to your local machine.
2. Open the provided Arduino sketch in the Arduino IDE.
3. Connect your Arduino-based robot to your computer.
4. Upload the sketch to the robot's microcontroller.

## Workshop Overview

### Workshop 1: Unwrapping Function

#### Objective
Learn to handle continuous rotation encoder data by implementing an unwrapping function to avoid sudden jumps in the data due to wrap-around.

#### Tasks
1. Study the `unwrapPhase` function in the code.
2. Understand how the function uses the `lastPhase` and `cumulativeOffset` variables to adjust the phase angle.

### Workshop 2: Finding Velocity

#### Objective
Calculate the angular velocities of the robot's wheels from encoder data.

#### Tasks
1. Examine the code section where `angularSpeedLeft` and `angularSpeedRight` are computed.
2. Understand how the change in encoder count is translated into angular velocities.

### Workshop 3: Forward Kinematics

#### Objective
Understand how to derive the robot's linear and angular velocity from wheel velocities.

#### Tasks
1. Analyze how `linearVelocityRobot` and `angularVelocityRobot` are calculated.
2. Discuss the relationship between wheel speeds and overall robot motion.

### Workshop 4: Odometry

#### Objective
Implement odometry to estimate the robot's position and orientation over time.

#### Tasks
1. Explore the section of the code that updates `positionX`, `positionY`, and `theta`.
2. Discuss how odometry is used for tracking the movement of a robot.

### Workshop 5: Inverse Kinematics

#### Objective
Learn how to calculate individual wheel speeds required to achieve a desired linear and angular velocity of the robot.

#### Tasks
1. Look at how `commandSpeedLeft` and `commandSpeedRight` are derived.
2. Understand the inverse kinematics equations used for a differential drive robot.

### Workshop 6: Go to Goal

#### Objective
Implement a simple control strategy to navigate the robot to a specified goal position.

#### Tasks
1. Analyze the code responsible for computing `commandLinearVelocity` and `commandAngularVelocity` towards the goal.
2. Test and observe the robot as it navigates to the specified `goalX`, `goalY`, and `goalTh`.

## Contribution

Feel free to contribute to this project by suggesting improvements or by extending the workshop with additional exercises.

## License

[Specify the license under which this workshop is released, if applicable]

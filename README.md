# 2-DOF Robotic Arm for Pick and Drop Operations

## Introduction
This project focuses on developing a 2-DOF (Degrees of Freedom) robotic arm capable of performing pick and drop operations. It is controlled via an Arduino and uses servo motors for precise movements.

## Materials
- Arduino Uno
- 2 x Servo Motors (for arm movement)
- 2 x Servo Motors (for gripper control)
- Power Supply for Servos
- Jumper Wires
- Mechanical parts for the arm and gripper

## Code Overview
The provided Arduino sketch (`sketch_dec16e.ino`) manages the robotic arm's movements. Key features include:

- **Servo Motor Control**: Utilizing the `Servo.h` library for servo operations.
- **Inverse Kinematics**: Implementing calculations for the angles (`theta1` and `theta2`) for the arm joints.
- **Workspace Validation**: Ensuring the target point is within the robot's reachable area.
- **Gripper Control**: Managing the opening and closing of the gripper.

## How the Code Works
1. **Initialization**: Sets up servo objects and defines their pins.
2. **Inverse Kinematics Calculation**: The `calculate_theta` function computes joint angles based on target (x, y) coordinates.
3. **Movement Execution**: Positions the servos using calculated angles to move the arm.
4. **Gripper Operation**: Controls the gripper for picking and dropping objects.

## Getting Started
1. Assemble the robotic arm using the listed materials.
2. Connect the servo motors to the Arduino as per the wiring diagram (not included).
3. Upload `sketch_dec16e.ino` to the Arduino.
4. Power the Arduino and servo motors.
5. Test the arm using the Arduino IDE's Serial Monitor.

## Contribution
Contributions are welcome! Feel free to fork this project and propose enhancements or additional features.


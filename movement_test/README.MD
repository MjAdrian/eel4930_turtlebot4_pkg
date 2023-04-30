# Movement Test Package
A package to test various movement methods such as constant linear, angular speed, and joystick control.

## Package Contents
1. Constant Movement: 
   - Moves the TurtleBot at a constant linear and angular speed. Primarily used to test if the TurtleBot is receiving commands from the computer.
2. Joy Movement:
   - Receives input from a joystick to control the TurtleBot.
   - Requires running ros2 run joy joy-node to receive controller input.
  
## Dependancies:
- ROS 2 Galactic
- Joy Node: `sudo apt install ros-galactic-joy` 

- Optional: For keyboard control, install Teleop Twist Keyboard package: `sudo apt install ros-galactic-teleop-twist-keyboard`
   - To run: `ros2 run teleop-twist-keyboard teleop-twist-keyboard`

## Package Usage 
- Package Name: `movement_test`
  - i.e. `ros2 (run | launch) movement_test (node/launch file)`

## Node Names
1. Constant movement: `constant_move` 
2. Joy movement: `joy_movement` 

## Launch Files
1. `joy_movemen.launch`
   - Launches all the nodes necessary to use a joystick controller.
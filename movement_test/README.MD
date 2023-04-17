# Movement Test pkg

Package to test movement methods, such as constant linear and angular speed, joy, camera perception, etc.

## Nodes in Package
1. Constant Movement: 
   - moves the turtlebot at a cosntant linear and angular speed. Just meant to test if the turtlebot is receiving from the computer
2. Joy Movement:
   - recieves from joy to move the turtlebot 
   - need to run `ros2 run joy joy-node` to recieved controller input
   - TODO: make a launch file to make this easier to run
3. Camera Distance: 
   - IN PROGRESS
   - hopes to move turtlebot based on recieve values,
   - but stops after being a certain distance from an object: IE it will stop 0.5 meters away from a wall
   - might change later 

## dependancies:
OpenCV: `sudo apt install python3-opencv ros-galactic-cv-bridge` 
Joy Node: `sudo apt install ros-galactic-joy` 

If you want to run the bot using your keyboard: `sudo apt install ros-galactic-teleop-twist-keyboard`
- then run `ros2 run teleop-twist-keyboard teleop-twist-keyboard`

## Package name 
movement_test
- i.e. `ros2 [run/launch] movement_test {node/launch file}`

## Node Names
Constant movement: `constant_move` 
Joy movement: `joy_movement` 
Camera Distance: TBA

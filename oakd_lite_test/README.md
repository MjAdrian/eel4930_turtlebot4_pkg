# Camera_Test Package
A package for testing the TurtleBot4 camera.

## Package Contents
1. `oakd_pub_test`
   - Runs a publisher that connects to the camera and publishes the captured images.
2. `oakd_sub_test`
   - Receives the published images and displays them in a window.

## Launch Files
1. `pc_cam_test.launch`
   - Run this launch file on the PC side to receive and display the images.
2. `turtlebot_cam_test.launch`
   - Run this launch file on the TurtleBot4 to enable the camera and publish images.


## Depedencies
- DepthAI: https://docs.luxonis.com/projects/api/en/latest/install/#supported-platforms
- DepthAI SDK: https://docs.luxonis.com/projects/sdk/en/latest/oak-camera/#examples
- OpenCV: `sudo apt install python3-opencv ros-galactic-cv-bridge` 

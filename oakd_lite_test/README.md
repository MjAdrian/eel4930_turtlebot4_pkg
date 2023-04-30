# Camera_Test pkg
Package to do turtlebot4 camera test

## Nodes in Package
1. `oakd_pub_test`
   - Runs Publisher that connects to camera and publishes the images
2. `oakd_sub_test`
   - Recieves images and displayes them in a window

## Launch Files
1. `pc_cam_test.launch`
   - As the name implies, run this launch file on PC side and it will recieve the images
2. `turtlebot_cam_test.launch`
   - Run this on the TB4 and it will turn on camera and publish images


## Depedencies
`depthai`: https://docs.luxonis.com/projects/api/en/latest/install/#supported-platforms
`depthai_sdk`: https://docs.luxonis.com/projects/sdk/en/latest/oak-camera/#examples

OpenCV: `sudo apt install python3-opencv ros-galactic-cv-bridge` 

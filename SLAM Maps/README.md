# SLAM Maps
A directory for storing maps created using TurtleBot4's built-in SLAM package.

## Procedure
To perform this task, you will need 4 terminals. It is recommended to use a terminal multiplexer like tmux or byobu.

### Terminal 1:

1. SSH into the TurtleBot4.
2. Run the following command: `ros2 launch turtlebot4_navigation slam_sync.launch.py`

------------------------------------------------------------------------

### Terminal 2: 
1.  On your PC (not SSH), run: ros2 launch turtlebot4_viz view_robot.launch.py

------------------------------------------------------------------------

### Terminal 3: 
1. Use your preferred controller, e.g., teleop_twist_keyboard, joystick, or a custom one.
  
------------------------------------------------------------------------

### Terminal 4:
Once you have finished mapping:
1. SSH into the TurtleBot4.
2. Run the following command:
   
```
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "name:
    data: 'YOUR_FILE_NAME'"
```

Note: There must be a new line, `\`, between `name:` and `data:` or it won't work.
 
Once saved, the map files will be saved in the current directory with the following names:	
   1. YOUR_FILE_NAME.pgm
   2. YOUR_FILE_NAME.yaml

This will save your file on the Raspberry Pi 4, to move it to your computer, do the following:
    - `scp username@remote_host:file/path/file.type local_directory_path`
    - If you have the Turtlebot saved as a hostname in `~/.ssh/config`
      - `scp myhostname:remote_file_path local_file_path`
        - e.g., `scp turltebot4:~/map.pgm .` will take the file `map.pgm` from the Turtlebot's home directory and move it to the current directory that your terminal is in
# eel4930_turtlebot4_pkg
Repo for EEL4930 Autonomous Robotics Ros 2 Galactic Packages

## Useful Turtlebot 4 Docs:
1. https://turtlebot.github.io/turtlebot4-user-manual/
	- https://github.com/turtlebot/turtlebot4_desktop
2. https://github.com/turtlebot/turtlebot4_robot/blob/galactic/turtlebot4_bringup/launch/oakd.launch.py

### Useful Tooling
#### RQT
```
rqt_graph
ros2 run rqt_console rqt_console
```

#### Topics
```
ros2 topic
    list
        -t
    echo <topic_name>
    info <topic_name>
    pub <topic_name> <msg_type> '<args>'
        --once
        --rate <rateHz>
    hz <topic_name>
ros2 interface show <type_name>
```

#### Services
```
ros2 service
    list
        -t
    type <service_name>
    find <service_type>
    call <service_name> <service_type> <arguments>
ros2 interface show <type_name>
```

#### Parameters
```
ros2 param
    list
    get <node_name> <parameter_name>
    set <node_name> <parameter_name> <value>
    dump <node_name>
    load <node_name> <parameter_file>
ros2 run <package_name> <executable_name> --ros-args --params-file <file_name>
```

#### Actions
```
ros2 action
    list
        -t
    info <action_name>
    send_goal <action_name> <action_type> <values>
        --feedback
ros2 interface show <action_name>
```

#### Bag
```
ros2 bag
    record
        <topic_name>
        -o <bag_file_name> <topic_name> <topic_name2>
    info <bag_file_name>
    play <bag_file_name>
```

#### Colcon
```
colcon
    build
        --symlink-install
        --packages-up-to <package_name>
        --packages-select <package_name>
        --event-handlers console_direct+
    test
colcon_cd <package_name>
```

#### Packages
```
# in <workspace_folder>/src/ run
ros2 pkg create --build-type ament_python <package_name>
    --node-name <node_name> <package_name>
# Put packages in src/<package_name>/<package_name>
```

#### Workspace Structure
```
workspace_folder/
    src/
        package_1/
            CMakeLists.txt
            package.xml
        package_2/
            setup.py
            package.xml
            resource/package_2
```

#### Common Packages
```
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py                        # Turtlebot Gazebo
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=true   # SLAM
ros2 run turtlebot3_teleop teleop_keyboard                                      # Teleop keyboard
ros2 run nav2_map_server map_saver_cli -f maps/world_map                        # Save map
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=maps/world_map.yaml
```

#### Custom Packages
```
source install/setup.bash
ros2 run autoro_navigation test
ros2 run autoro_navigation get_costmap_test
ros2 run autoro_navigation get_map_test
```

#### Turtlebot3-specific Code
```
export TURTLEBOT3_MODEL=burger
sudo apt install ros-galactic-rmw-cyclonedds-cpp
In .bashrc: export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
In /opt/ros/galactic/share/turtlebot3_navigation2/param/burger.yaml:
    Change robot_model_type: "nav2_amcl::DifferentialMotionModel"
```

#### Compatibility with ROS1
* Make sure to export ROS2 vars as described above.
* Update PYTHONPATH to point to ROS2 directories
* Make sure that ROS1 has not been sourced before building/installing ROS2 packages

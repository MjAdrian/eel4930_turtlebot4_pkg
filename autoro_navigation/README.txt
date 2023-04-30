To run:

-Add to setup.bash
    export TURTLEBOT3_MODEL=waffle
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

Map of desired world must be located in /maps directory

-Run
    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
    ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=maps/world_map.yaml
    ros2 run autoro_navigation test
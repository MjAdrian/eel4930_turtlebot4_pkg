cmake_minimum_required(VERSION 3.5)
project(autoro_navigation)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(autoro_interfaces REQUIRED)

ament_python_install_package(${PROJECT_NAME})

# Put launch
install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME})

# Put maps into share
install(DIRECTORY maps
    DESTINATION share/${PROJECT_NAME})
install(DIRECTORY models
    DESTINATION share/${PROJECT_NAME})
install(DIRECTORY worlds
    DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz
    DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME})

# Python scripts
install(PROGRAMS
    scripts/test_node.py
    scripts/get_map_test.py
    scripts/occupancy_grid_test.py
    nodes/marker_server.py
    nodes/navigation.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()

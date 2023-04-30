cmake_minimum_required(VERSION 3.5)
project(autoro_navigation)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

ament_python_install_package(${PROJECT_NAME})

# Put launch
install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME})

# Python scripts
install(PROGRAMS
    scripts/test_node.py
    scripts/get_map_test.py
    scripts/occupancy_grid_test.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()

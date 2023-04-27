cmake_minimum_required(VERSION 3.5)
project(autoro_navigation)

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME})

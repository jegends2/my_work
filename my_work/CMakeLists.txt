cmake_minimum_required(VERSION 3.5)
project(my_work)

find_package(Fields2Cover REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

target_link_libraries(<<<my_work>>> Fields2Cover)


install(
    DIRECTORY my_work/
    DESTINATION lib/python${PYTHON_VERSION}/site-packages/my_work
)

ament_package()

cmake_minimum_required(VERSION 3.5)
project(ars_40x)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ars_40x_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(hb_msgs REQUIRED)

include_directories(include)

add_executable(radar_node src/radar_node.cpp src/controller.cpp src/iolayer.cpp)
ament_target_dependencies(radar_node ars_40x_msgs rclcpp std_msgs hb_msgs)

add_executable(rviz_node src/rviz_node.cpp)
ament_target_dependencies(rviz_node ars_40x_msgs rclcpp visualization_msgs)

install(
	TARGETS radar_node rviz_node
	DESTINATION lib/${PROJECT_NAME}
)

install(
	DIRECTORY launch
	DESTINATION share/${PROJECT_NAME}
)


ament_package()

cmake_minimum_required(VERSION 3.8)
project(radar_annotation)

set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_STANDARD 17)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

# file(GLOB PCL_LIBRARIES /usr/lib/libpcl_*)
# set(PCL_INCLUDE_DIRS /usr/include/pcl-1.12)

find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})
link_directories(${Pangolin_LIBRARY_DIRS})

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosbag2 REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosbag2_cpp REQUIRED)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ars_40x_msgs REQUIRED)
# find_package(box_msgs REQUIRED)
# find_package(hb_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rviz_common REQUIRED)
find_package(pluginlib REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)

find_package(Qt5Widgets REQUIRED)
find_package(PCL 1.12 REQUIRED)
find_package(pcl_conversions REQUIRED)
# find_package(pcl_ros REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(QT_LIBRARIES Qt5::Widgets)


include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_data_preprocess src/data_preprocess.cpp src/SensorDataWriter.cpp)
ament_target_dependencies(${PROJECT_NAME}_data_preprocess sensor_msgs ars_40x_msgs rosbag2_cpp rclcpp rosbag2 rosbag2_storage pcl_conversions  cv_bridge yaml-cpp)
# ament_target_dependencies(${PROJECT_NAME}_data_preprocess sensor_msgs ars_40x_msgs rosbag2_cpp rclcpp rosbag2 rosbag2_storage pcl_conversions pcl_ros cv_bridge yaml-cpp)

# add_executable(${PROJECT_NAME}_annotation src/main.cpp)
# ament_target_dependencies(${PROJECT_NAME}_annotation rclcpp rclpy std_msgs tf2_sensor_msgs sensor_msgs PCL pcl_conversions visualization_msgs ars_40x_msgs pcl_msgs
#         cv_bridge OpenCV)
# target_link_libraries(${PROJECT_NAME}_annotation ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(${PROJECT_NAME}_annotation src/main.cpp)
ament_target_dependencies(${PROJECT_NAME}_annotation PCL pcl_conversions ars_40x_msgs pcl_msgs
        cv_bridge OpenCV)
target_link_libraries(${PROJECT_NAME}_annotation ${PCL_LIBRARIES} ${OpenCV_LIBS})



install(TARGETS ${PROJECT_NAME}_data_preprocess DESTINATION lib/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}_annotation DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)


install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)


install(
  DIRECTORY "include/"
  DESTINATION include
)

install(FILES
        plugin_description.xml
        DESTINATION share/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch
)

ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()


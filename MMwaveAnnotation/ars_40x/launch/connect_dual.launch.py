from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
# launch two radar nodes with different namespaces and parameter files

def generate_launch_description():
    params_front = os.path.join(
        get_package_share_directory("ars_40x"), "launch", "radar_front.yaml"
    )
    radar_front = Node(
        package="ars_40x", executable="radar_node", namespace="front", parameters=[params_front]
    )
    params_back = os.path.join(
        get_package_share_directory("ars_40x"), "launch", "radar_back.yaml"
    )
    radar_back = Node(
        package="ars_40x", executable="radar_node", namespace="back", parameters=[params_back]
	)
    rviz_back = Node(package="ars_40x", executable="rviz_node", namespace="back")
    rviz_front = Node(package="ars_40x", executable="rviz_node", namespace="front")
    launch_description = LaunchDescription([radar_front, radar_back,rviz_front,rviz_back])
    return launch_description

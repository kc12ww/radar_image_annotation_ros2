import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():
    ld = LaunchDescription()
    share_dir = get_package_share_directory('radar_annotation')
    params_file = os.path.join(share_dir, "config", "params.yaml")

    annotation_node = Node(
        package = 'radar_annotation',
        executable = 'radar_annotation_annotation',
        parameters = [params_file],
        output = 'screen'
    )
 
    ld.add_action(annotation_node)
    return ld



import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import yaml


def generate_launch_description():
    parameters_file_path = Path(get_package_share_directory('peak_cam'), 'params', 'settings', 'front.yaml')
    camera_info_path = Path(get_package_share_directory('peak_cam'), 'params', 'intrinsics', 'default_camera_info.yaml')

    with open(parameters_file_path, 'r') as f:
        params = yaml.safe_load(f)['/**']['ros__parameters']
    print(params)

    container = ComposableNodeContainer(
        name='peak_cam_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='peak_cam',
                plugin='peak_cam::PeakCamNode',
                name='peak_cam',
                parameters=[
                    params,
                    {'camera_info_url': 'file://' + str(camera_info_path)}])
        ],
        output='screen'
    )

    return launch.LaunchDescription([container])
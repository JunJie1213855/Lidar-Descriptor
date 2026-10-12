from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(get_package_share_directory('btc_desc'), 'config', 'config_outdoor.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('input_topic', default_value='/points_raw'),
        DeclareLaunchArgument('output_dir', default_value='/tmp/btc_descriptors'),
        DeclareLaunchArgument('config_file', default_value=config),
        DeclareLaunchArgument('max_visualized_descriptors', default_value='100'),
        Node(package='btc_desc', executable='btc_extract_node',
             name='btc_extract_node', output='screen',
             parameters=[{'input_topic': LaunchConfiguration('input_topic'),
                          'output_dir': LaunchConfiguration('output_dir'),
                          'config_file': LaunchConfiguration('config_file'),
                          'max_visualized_descriptors': ParameterValue(
                              LaunchConfiguration('max_visualized_descriptors'), value_type=int)}]),
    ])

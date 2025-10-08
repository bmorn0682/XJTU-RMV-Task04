import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    exposure_time_arg = DeclareLaunchArgument(
        'exposure_time', default_value='5000.0', description='setting a exposure_time_arg'
    )

    exposure_time = LaunchConfiguration('exposure_time')

    frame_rate_arg = DeclareLaunchArgument(
        'frame_rate', default_value='40.0', description='frame_rate'
    )

    frame_rate = LaunchConfiguration('frame_rate')

    gain_arg = DeclareLaunchArgument(
        'gain', default_value='1.0', description='gain'
    )

    gain = LaunchConfiguration('gain')

    pixel_format_arg = DeclareLaunchArgument(
        'pixel_format', default_value='PixelType_Gvsp_Mono8', description='pixel_format'
    )

    pixel_format = LaunchConfiguration('pixel_format')

    PubNode = Node(
        package='sdk_ros2',
        executable='sdk_ros2_node',
        output='screen',
        parameters=[
            {'exposure_time':exposure_time},
            {'gain':gain},
            {'frame_rate':frame_rate},
            {'pixel_format': pixel_format}
        ]
    )

    pkg_share = get_package_share_directory('sdk_ros2')
    rviz2_config_file = os.path.join(
        pkg_share, 'rviz', 'rviz2_config.rviz'
    )

    rviz2_node = Node(
            package='rviz2',       # RViz 所在包
            executable='rviz2',    # 可执行文件名
            output='screen',       # 输出到屏幕
            arguments=['--display-config', rviz2_config_file]  # 可选，加载配置文件
        )
    
    return LaunchDescription([
        exposure_time_arg,
        gain_arg,
        frame_rate_arg,
        pixel_format_arg,
        PubNode,
        rviz2_node
    ])
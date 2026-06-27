import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('gpd_ros'),
        'config',
        'ros_eigen_params.cfg'
    )

    return LaunchDescription([
        Node(
            package='gpd_ros',
            executable='detect_grasps',
            name='detect_grasps',
            output='screen',
            parameters=[{
                
                'cloud_topic': '/camera/depth/color/points',

                'config_file' : config_file,

                # RViz topic to publish grasps
                'rviz_topic': 'plot_grasps',

                'use_sim_time': False
            }]
        )
    ])

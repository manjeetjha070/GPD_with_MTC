from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gpd_ros',
            executable='detect_grasps',
            name='detect_grasps',
            output='screen',
            parameters=[{
                
                'cloud_topic': '/camera/depth/color/points',

                # Filepath to the configuration file for GPD
                'config_file': '/home/pg/rrs_ss25/manjeet/ros/src/GPD_with_MTC/gpd_ros/config/ros_eigen_params.cfg',

                # RViz topic to publish grasps
                'rviz_topic': 'plot_grasps',

                'use_sim_time': False
            }]
        )
    ])

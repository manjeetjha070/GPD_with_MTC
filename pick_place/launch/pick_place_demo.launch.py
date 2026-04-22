from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("kinova_gen3_6dof_robotiq_2f_85").to_dict()
    #moveit_config = MoveItConfigsBuilder("moveit_resources_panda").to_dict()

    # MTC Demo node
    pick_place_demo = Node(
        package="pick_place",
        executable="mtc_node",
        output="screen",
        parameters=[
            moveit_config,
            {'use_sim_time': False}
        ],
    )

    return LaunchDescription([pick_place_demo])
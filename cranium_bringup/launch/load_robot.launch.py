import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from rclpy.node import Node
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from typing import Dict, Optional, Union
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.event_handlers import OnProcessStart

from lbr_description import LBRDescriptionMixin
from lbr_ros2_control import LBRROS2ControlMixin

class LBRSystemInterface(LBRDescriptionMixin, LBRROS2ControlMixin):
    pass

def generate_launch_description():
    ld = LaunchDescription()

    # Get the package share directory
    lbr_ros2_control_directory = get_package_share_directory('lbr_ros2_control')
    cranium_bringup_directory = get_package_share_directory('cranium_bringup')



    # Define the path to your launch file
    lbr_launch_file = os.path.join(lbr_ros2_control_directory, 'launch', 'system_interface.launch.py')

    # Create an IncludeLaunchDescription action to include your launch file
    included_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lbr_launch_file),
        launch_arguments={
            'ctrl_cfg_pkg': 'cranium_bringup',
            'ctrl_cfg': 'config/lbr_controllers.yaml'}.items(),
    )

    cartesian_force_controller = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "cartesian_force_controller",
            "--controller-manager",
            "controller_manager",
        ],
        namespace="lbr"
    )

    robot_description = LBRSystemInterface.param_robot_description(sim=False)

    ros2_control_node = LBRSystemInterface.node_ros2_control(
        robot_description=robot_description
    )

    controller_event_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=ros2_control_node,
            on_start=[
                cartesian_force_controller
            ],
        )
    )
    ld.add_action(cartesian_force_controller)

    # Create the LaunchDescription and add the included launch file

    ld.add_action(included_launch_file)

    return ld
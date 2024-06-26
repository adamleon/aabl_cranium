from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnIncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from lbr_description import LBRDescriptionMixin, RVizMixin


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    ld.add_action(LBRDescriptionMixin.arg_sim())

    ld.add_action(
        DeclareLaunchArgument(
            name="model",
            default_value="med14",
            description="Which robot to use.",
        )
    )
    
    controller_config = PathJoinSubstitution(
                    [
                        FindPackageShare("cranium_bringup"),
                        "config",
                        "lbr_controllers.yaml",
                    ]
                )
    
    ld.add_action(
        DeclareLaunchArgument(
            name="ctrl",
            default_value="joint_trajectory_controller",
            description="Which controller to load",
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("lbr_bringup"),
                        "launch",
                        "sim.launch.py",
                    ]
                )
            ),
            condition=IfCondition(LaunchConfiguration("sim")),
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("lbr_bringup"),
                        "launch",
                        "real.launch.py",
                    ]
                )
            ),
            condition=UnlessCondition(LaunchConfiguration("sim")),
        )
    )

    return ld

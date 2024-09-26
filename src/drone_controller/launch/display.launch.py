import launch
from launch.substitutions import Command, LaunchConfiguration 
import launch_ros
import os 

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package="drone_controller").find("drone_controller")
    default_model_path = os.path.join(pkg_share,"urdf/drone_robot.urdf.xacro")
    default_rviz_config_path = os.path.join(pkg_share,"rviz/config.rviz")

    robot_description = Command(["xacro ", default_model_path])
    
    rviz_node = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", default_rviz_config_path],
        parameters=[{"robot_description" : robot_description}]
    )

    return launch.LaunchDescription([
       rviz_node
    ])

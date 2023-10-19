from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():

    md_robot_node = Node(
        package='motor_test',
        executable='md_robot_node',
        parameters=[
            ("use_MDUI", "0"),                  # 0: not use MDUI, 1: use MDUI
            ("wheel_radius", "0.0935"),         # unit: meter
            ("wheel_length", "0.454"),          # unit: meter
            ("reduction", "30"),                # Gear ratio
            ("reverse_direction", "0"),         # 0: forward, 1: reverse
            ("maxrpm", "1000"),                 # unit: RPM
            ("position_type", "1"),             # 0: hallsensor, 1: encoder
            ("encoder_PPR", "16384"),           # if use encoder position, encoder PPR(Pulse Per Rotation)
            ("position_proportion_gain", "20"), # reference PID 203(PID_GAIN)
            ("speed_proportion_gain", "50"),    # reference PID 203(PID_GAIN)
            ("integral_gain", "1800"),          # reference PID 203(PID_GAIN)
            ("slow_start", "300"),              # unit: RPM
            ("slow_down", "300")                # unit: RPM
        ],
        arguments=[],
        output="screen"
    )

    return LaunchDescription(
        [
            md_robot_node,
        ]
    )
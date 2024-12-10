from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='examen_final_robotica_2024',
            executable='principal',
            name='principal',
            output='screen'
        ),
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam_node',
        )
    ])

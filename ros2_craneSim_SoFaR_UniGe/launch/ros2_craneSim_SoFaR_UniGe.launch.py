import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_craneSim_SoFaR_UniGe',
            executable='crane_sim_node',
            name='crane_sim_node',
            output='screen'
        ),
        Node(
            package='ros2_craneSim_SoFaR_UniGe',
            executable='motor_x_controller',
            name='motor_x_controller',
            output='screen',
            prefix='xterm -e',
            emulate_tty=True,
        ),
        Node(
            package='ros2_craneSim_SoFaR_UniGe',
            executable='motor_y_controller',
            name='motor_y_controller',
            output='screen',
            prefix='xterm -e',
            emulate_tty=True,
        ),
        Node(
            package='ros2_craneSim_SoFaR_UniGe',
            executable='robot_logic',
            name='robot_logic',
            output='screen',
            prefix='xterm -e',
            emulate_tty=True,
        ),
    ])
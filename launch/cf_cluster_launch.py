from launch import LaunchDescription
from launch_ros.actions import Node

#this launch file runs all cluster control files
def generate_launch_description():
    return LaunchDescription([
        Node( 
            package='crazyfly_core',
            executable='clusterControllerClass',
            name='cluster_controller_node',
            output='screen'
        ),
        Node(
            package='crazyfly_core',
            executable='pid_controller_cf1',
            name='pid_controller_cf1_node',
            output='screen'
        ),
        Node(
            package='crazyfly_core',
            executable='pid_controller_cf2',
            name='pid_controller_cf2_node',
            output='screen'
        ),
        Node(
            package='crazyfly_core',
            executable='cf_command_controller2',
            name='cf_command_controller2_node',
            output='screen'
        )
        
    ])
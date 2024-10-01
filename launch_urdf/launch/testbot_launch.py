import launch
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os 

def generate_launch_description():
    # Define launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    urdf = os.path.join(get_package_share_directory('launch_urdf'), 'urdf', 'testbot.urdf')

    with open(urdf, 'r') as infp:
        robot_desc = infp.read()


    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
           'use_sim_time',
            default_value='false',),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_desc,
                'source_list': ['joint_1', 'joint_2', 'joint_3'],
                'zeros': {
                    'joint_1': 1.5708,
                    'joint_2': 1.5708,
                    'joint_3': 1.5708
                }
            }]
        ),
         Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen',
            arguments=['-d', os.path.join(get_package_share_directory('launch_urdf'),'rviz','urdf_rviz.rviz')] # Replace with your RViz config file path
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
				'robot_description': robot_desc,
				'source_list': ['joint1', 'joint2', 'joint3'],
				'zeros': {
					'joint1': 1.5708,
					'joint2': 1.5708,
					'joint3': 1.5708
				}
			}]
        )
    ])


if __name__ == '__main__':
    generate_launch_description()
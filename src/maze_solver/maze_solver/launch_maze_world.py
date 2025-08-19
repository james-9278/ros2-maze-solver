import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchService
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Path to your custom maze world file
    pkg_maze_solver = get_package_share_directory('maze_solver')
    world_file = os.path.join(pkg_maze_solver, 'worlds', 'maze_world.world')

    # Path to the launch file for spawning the robot
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    spawn_robot_launch_file = os.path.join(pkg_turtlebot3_gazebo, 'launch', 'spawn_turtlebot3.launch.py')

    return LaunchDescription([
        # 1. Launch Gazebo with your custom world
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # 2. Include the official TurtleBot3 spawner launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(spawn_robot_launch_file),
            launch_arguments={'x_pose': '0.0', 'y_pose': '0.0'}.items(),
        ),
    ])

# Main function to allow running with ros2 run
def main(args=None):
    ld = generate_launch_description()
    ls = LaunchService()
    ls.include_launch_description(ld)
    return ls.run()

if __name__ == '__main__':
    main()

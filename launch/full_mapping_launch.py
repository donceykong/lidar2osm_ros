import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue

# IS this the same as FindPackageShare?
# from ament_index_python.packages import get_package_prefix 

def add_robot(env, robot_num, root_data_dir):
    robot_name = f"robot{robot_num}"

    # Play bag for robot
    start_offset_sec = 200 #900 #1030  # Try 900 s
    robot_bag_path = os.path.join(root_data_dir, f"{env}_{robot_name}", f"{env}_{robot_name}_0.db3")
    robot_bag_play = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play', robot_bag_path,
            '--clock',  # publishes /clock for sim time
            '--start-offset', str(start_offset_sec)
        ],
        output = 'screen'
    )

    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (bag) time if true'
    )

    # Publish URDF for robot
    xacro_file = PathJoinSubstitution([FindPackageShare('lidar2osm_ros'), 'urdf', 'multi_robot.urdf.xacro'])
    xacro_command = Command(['xacro ', xacro_file, ' ', 'name:=', f"{robot_name}"])
    robot_urdf = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name=f'{robot_name}_robot_state_publisher',
        parameters=[{'robot_description': xacro_command}],
        remappings=[('/robot_description', f'/{robot_name}/robot_description')]
    )

    robot_map_accumulator_node = Node(
        package='lidar2osm_ros',
        executable='robot_map_accumulator',
        # name=f'{robot_name}_robot_map_accumulator',
        arguments=[robot_name],  # Pass the robot name as an argument
    )

    return use_sim_time, robot_bag_play, robot_urdf, robot_map_accumulator_node

def add_bag_recording(output_dir, topics_to_record=None):
    record_cmd = ['ros2', 'bag', 'record']
    if topics_to_record:
        record_cmd.extend(topics_to_record)  # Add specific topics
    else:
        record_cmd.append('-a')  # Record all topics

    return ExecuteProcess(
        cmd=record_cmd + ['-o', output_dir],  # Add output directory
        output='screen'
    )

def generate_launch_description():
    environment = "kittredge_loop"
    number_of_robots = 4
    root_data_dir = "/media/donceykong/doncey_ssd_02/datasets/CU_MULTI/ros2_bags/with_gt"
    # root_data_dir = "/home/donceykong/Desktop/ARPG/projects/spring2025/lidar2osm_full/cu-multi-dataset/data/ros2_bags"
    node_list = []

    robot_nums_to_use = [1, 2, 3, 4]
    for robot_num in robot_nums_to_use: #range(1, number_of_robots+1):
        per_robot_node_list = add_robot(environment, robot_num, root_data_dir)
        node_list.extend(per_robot_node_list)

    robot_distance_checker_node = Node(
        package='lidar2osm_ros',
        executable='robot_distance_checker',
        name='robot_distance_checker'
    )

    # RViz node
    rviz_config_path = PathJoinSubstitution([FindPackageShare('lidar2osm_ros'), 'rviz', 'full_mapping.rviz'])
    rviz_node = Node(
        package='rviz2', 
        executable='rviz2', 
        name='rviz2', 
        output='screen', 
        arguments=['-d', rviz_config_path]
    )
    rviz_timed_node = TimerAction(
        period= 5.0,  # start after delay to ensure topics are available
        actions=[rviz_node]
    )

    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (bag) time if true'
    )

    # Choose non-robot nodes here
    non_robot_nodes = [
        # use_sim_time,
        # robot_distance_checker_node,
        rviz_timed_node,
    ]

    # Extend node list with non-robot nodes
    node_list.extend(non_robot_nodes)

    return LaunchDescription(node_list)

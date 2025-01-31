import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue

# IS this the same as FindPackageShare?
# from ament_index_python.packages import get_package_prefix 

def add_robot(env, robot_num, root_data_dir):
    robot_name = f"robot{robot_num}"

    # Play bag for robot
    robot_bag_path = os.path.join(root_data_dir, f"{env}_{robot_name}", f"{env}_{robot_name}_0.db3")
    robot_bag_play = ExecuteProcess(cmd = ['ros2', 'bag', 'play', robot_bag_path], output = 'screen')

    # Publish URDF for robot
    xacro_file = PathJoinSubstitution([FindPackageShare('lidar2osm_ros'), 'urdf', 'multi_robot.urdf.xacro'])
    xacro_command = Command(['xacro ', xacro_file, ' ', 'name:=', f"{robot_name}"])
    robot_urdf = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': xacro_command}],
        remappings=[('/robot_description', f'/{robot_name}/robot_description')]
    )

    robot_map_accumulator_node = Node(
        package='lidar2osm_ros',
        executable='robot_map_accumulator',
        name='robot_map_accumulator',
        arguments=[robot_name],  # Pass the robot name as an argument
    )
    # # Octomap for specific robot
    # octomap_server_node = Node(
    #     package='octomap_server',
    #     executable='octomap_server_node',
    #     name='octomap_server',
    #     parameters=[{
    #         'resolution': 0.5,
    #         'frame_id': 'world',
    #         'base_frame_id': f'{robot_name}_base_link',
    #         'filter_speckles': True
    #         # 'colored_map': True
    #         # 'sensor_model.max_range': 200.0,
    #         # 'incremental_2D_projection': False,
    #         # 'occupancy_min_z': 0.1,
    #         # 'occupancy_max_z': 20.0,
    #         # 'filter_ground_plane': False,
    #         # 'ground_filter.distance': 0.04,
    #         # 'ground_filter.angle': 0.15,
    #         # 'ground_filter.plane_distance': 1.0,
    #         # 'pointcloud_min_z': -3.0,
    #         # 'pointcloud_max_z': 1.5
    #     }],
    #     remappings=[
    #         ('/cloud_in', f'/{robot_name}/ouster/points'),
    #         ('/occupied_cells_vis_array', f'/{robot_name}/occupied_cells_vis_array')
    #     ]
    # )

    return robot_bag_play, robot_urdf, robot_map_accumulator_node

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
    # root_data_dir = "/media/donceykong/doncey_ssd_02/lidar2osm_bags"
    root_data_dir = "/home/donceykong/Desktop/ARPG/projects/spring2025/lidar2osm_full/cu-multi-dataset/data/ros2_bags"
    node_list = []

    for robot_num in range(1, number_of_robots+1):
        per_robot_node_list = add_robot(environment, robot_num, root_data_dir)
        node_list.extend(per_robot_node_list)

    robot_distance_checker_node = Node(
        package='lidar2osm_ros',
        executable='robot_distance_checker',
        name='robot_distance_checker'
    )

    tsdf_server_node = Node(
        package='voxblox_ros',
        executable='tsdf_server',
        name='voxblox_node',
        output='screen',
        arguments=['-alsologtostderr'],
        # arguments=['-alsologtostderr', '--ros-args', '--log-level', 'debug'],
        remappings=[('pointcloud', '/robot1/ouster/points')],
        parameters=[
            {'min_time_between_msgs_sec': 0.0},
            {'tsdf_voxel_size': 0.5},       # Was 0.2
            {'truncation_distance': 0.5},   # Was 0.5
            {'color_mode': 'normals'},
            {'enable_icp': False},
            {'icp_refine_roll_pitch': False},
            {'update_mesh_every_n_sec': 1.0},
            {'mesh_min_weight': 0.1},       # Was 2.0
            {'method': 'fast'},
            {'max_ray_length_m': 50.0},
            {'use_const_weight': True},
            {'world_frame': 'world'},
            {'verbose': False},
            # {'mesh_filename': LaunchConfiguration('bag_file')}
        ],
    )

    # RViz node
    rviz_config_path = PathJoinSubstitution([FindPackageShare('lidar2osm_ros'), 'rviz', 'rviz2_layout.rviz'])
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

    bag_rec = add_bag_recording("/media/donceykong/doncey_ssd_02/lidar2osm_bags/full_bag")

    # Choose non-robot nodes here
    non_robot_nodes = [
        # tsdf_server_node,
        # robot_distance_checker_node,
        rviz_timed_node,
        # bag_rec,
    ]

    # Extend node list with non-robot nodes
    node_list.extend(non_robot_nodes)

    return LaunchDescription(node_list)

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

def add_liosam_nodes(robot_name):
    share_dir = get_package_share_directory('lidar2osm_ros')
    parameter_file = LaunchConfiguration('params_file')
    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(share_dir, 'config', 'liosam_params.yaml'),
        description='FPath to the ROS2 parameters file to use.'
    )

    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=f'0.0 0.0 0.0 0.0 0.0 0.0 map odom'.split(' '),
        parameters=[parameter_file],
        output='screen'
    )

    # static_tf_node_2 = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments=f'0.0 0.0 0.0 0.0 0.0 0.0 map base_link'.split(' '),
    #     parameters=[parameter_file],
    #     output='screen'
    # )

    preintegration_node = Node(
        package='lio_sam',
        executable='lio_sam_imuPreintegration',
        name=f'lio_sam_imuPreintegration',
        parameters=[parameter_file],
        output='screen'
    )

    image_proj_node = Node(
        package='lio_sam',
        executable='lio_sam_imageProjection',
        name=f'lio_sam_imageProjection',
        parameters=[parameter_file],
        output='screen'
    )

    feature_extraction_node = Node(
        package='lio_sam',
        executable='lio_sam_featureExtraction',
        name=f'lio_sam_featureExtraction',
        parameters=[parameter_file],
        output='screen'
    )

    map_optimization_node = Node(
        package='lio_sam',
        executable='lio_sam_mapOptimization',
        name=f'lio_sam_mapOptimization',
        parameters=[parameter_file],
        output='screen'
    )
    
    lio_sam_timed_nodes = []
    lio_sam_nodes = [params_declare, static_tf_node, preintegration_node, image_proj_node, feature_extraction_node, map_optimization_node]
    for node in lio_sam_nodes:
        lio_sam_timed_nodes.append(
            TimerAction(
                period= 10.0,  # start after delay to ensure topics are available
                actions=[node]
            )
        )

    return lio_sam_timed_nodes

    
def add_robot(env, robot_num, root_data_dir):
    robot_name = f"robot{robot_num}"

    # Play bag for robot
    robot_bag_path = os.path.join(root_data_dir, f"{env}_{robot_name}", f"{env}_{robot_name}_0.db3")
    robot_bag_play = ExecuteProcess(
        cmd = ['ros2', 'bag', 'play', robot_bag_path],
        output = 'screen'
    )

    # Publish URDF for robot
    xacro_file = PathJoinSubstitution([FindPackageShare('lidar2osm_ros'), 'urdf', 'single_robot.urdf.xacro'])
    # xacro_command = Command(['xacro ', xacro_file])
    xacro_command = Command(['xacro ', xacro_file, ' ', 'name:=', f""])
    robot_urdf = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name=f'robot_state_publisher',
        parameters=[{'robot_description': xacro_command}],
        # remappings=[('/robot_description', f'/{robot_name}/robot_description')]
    )

    # robot_map_accumulator_node = Node(
    #     package='lidar2osm_ros',
    #     executable='robot_map_accumulator',
    #     # name=f'{robot_name}_robot_map_accumulator',
    #     arguments=[robot_name],  # Pass the robot name as an argument
    # )

    # robot_map_reciever_node = Node(
    #     package='lidar2osm_ros',
    #     executable='robot_map_reciever',
    #     # name=f'{robot_name}_robot_map_accumulator',
    #     arguments=[robot_name],  # Pass the robot name as an argument
    # )

    node_list = [robot_bag_play, robot_urdf] #, robot_map_accumulator_node, robot_map_reciever_node]

    liosam_nodes = add_liosam_nodes(robot_name)
    node_list.extend(liosam_nodes)

    return node_list

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
    number_of_robots = 1
    root_data_dir = "/media/donceykong/doncey_ssd_02/datasets/CU_MULTI/ros2_raw_bags"
    # root_data_dir = "/media/donceykong/doncey_ssd_03/ros2bags"
    # root_data_dir = "/home/donceykong/Desktop/ARPG/projects/spring2025/lidar2osm_full/cu-multi-dataset/data/ros2_bags"
    node_list = []

    for robot_num in range(1, number_of_robots+1):
        node_list.extend(add_robot(environment, robot_num, root_data_dir))

    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (bag) time if true'
    )

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
    rviz_config_path = PathJoinSubstitution([FindPackageShare('lidar2osm_ros'), 'rviz', 'rviz2_layout_02.rviz'])
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
        # use_sim_time
    ]

    # Extend node list with non-robot nodes
    node_list.extend(non_robot_nodes)

    return LaunchDescription(node_list)

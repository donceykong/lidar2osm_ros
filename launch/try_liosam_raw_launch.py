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
        default_value=os.path.join(share_dir, 'config', f'liosam_params.yaml'),
        description='FPath to the ROS2 parameters file to use.'
    )

    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=f'0.0 0.0 0.0 0.0 0.0 0.0 map odom'.split(' '),
        parameters=[parameter_file],
        output='screen'
    )

    static_tf_node2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=f'0.0 0.0 0.0 0.0 0.0 0.0 map base_link'.split(' '),
        parameters=[parameter_file],
        output='screen'
    )

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
    lio_sam_nodes = [params_declare, static_tf_node, static_tf_node2, preintegration_node, image_proj_node, feature_extraction_node, map_optimization_node]
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
    robot_bag_path = os.path.join(root_data_dir, f"{env}_{robot_name}", f"{env}_{robot_name}.db3")
    robot_bag_play = ExecuteProcess(
        cmd = ['ros2', 'bag', 'play', robot_bag_path],
        output = 'screen'
    )

    # Publish URDF for robot
    xacro_file = PathJoinSubstitution([FindPackageShare('lidar2osm_ros'), 'urdf', 'single_robot.urdf.xacro'])
    xacro_command = Command(['xacro ', xacro_file, ' ', 'name:=', f""])
    robot_urdf = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name=f'robot_state_publisher',
        parameters=[{'robot_description': xacro_command}],
    )

    node_list = [robot_bag_play, robot_urdf]

    liosam_nodes = add_liosam_nodes(robot_name)
    node_list.extend(liosam_nodes)

    return node_list

def generate_launch_description():
    environment = "kittredge_loop"
    robot_list = [1]
    root_data_dir = f"/media/donceykong/doncey_ssd_011/ros2bags/{environment}"
    root_data_dir = f"/media/donceykong/doncey_ssd_02/ON_SSD/ros2_raw_bags/{environment}"
    node_list = []

    for robot_num in robot_list:
        node_list.extend(add_robot(environment, robot_num, root_data_dir))

    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (bag) time if true'
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

    # Choose non-robot nodes here
    non_robot_nodes = [
        rviz_timed_node,
        # use_sim_time
    ]

    # Extend node list with non-robot nodes
    node_list.extend(non_robot_nodes)

    return LaunchDescription(node_list)

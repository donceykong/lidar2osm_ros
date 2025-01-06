import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rviz_config_path = "/home/donceykong/Desktop/ARPG/projects/fall_2024/Lidar2OSM_FULL/lidar2osm_testing_ws/src/lidar2osm_ros/rviz/rviz2_layout.rviz"
    bag1_path = "/media/donceykong/doncey_ssd_02/orig_kittredge_loop_robot1/kittredge_loop_robot1_0.db3"
    bag2_path = "/media/donceykong/doncey_ssd_02/orig_kittredge_loop_robot2/kittredge_loop_robot2_0.db3"

    # rviz_config_path = "/home/donceykong/Desktop/ros2_ws/src/lidar2osm_ros/rviz/rviz2_layout.rviz"
    # bag1_path = "/home/donceykong/Desktop/lidar2osm_data/orig_kittredge_loop_robot1/kittredge_loop_robot1_0.db3"
    # bag2_path = "/home/donceykong/Desktop/lidar2osm_data/orig_kittredge_loop_robot2/kittredge_loop_robot2_0.db3"

    # Paths
    # bag_file_path = LaunchConfiguration(bag_path)
    
    # rviz_config_path = LaunchConfiguration(
    #     os.path.join(get_package_share_directory('lidar2osm_ros'), 'rviz', rviz_config_path)
    # )

    robot_distance_checker_node = Node(
        package='lidar2osm_ros',
        executable='robot_distance_checker',
        name='robot_distance_checker'
    )
    
    octomap_server_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        parameters=[{
            'resolution': 0.5,
            'frame_id': 'robot1_map',
            'base_frame_id': 'robot1_base_link',
            'filter_speckles': True
            # 'colored_map': True
            # 'sensor_model.max_range': 200.0,
            # 'incremental_2D_projection': False,
            # 'occupancy_min_z': 0.1,
            # 'occupancy_max_z': 20.0,
            # 'filter_ground_plane': False,
            # 'ground_filter.distance': 0.04,
            # 'ground_filter.angle': 0.15,
            # 'ground_filter.plane_distance': 1.0,
            # 'pointcloud_min_z': -3.0,
            # 'pointcloud_max_z': 1.5
        }],
        remappings=[
            ('/cloud_in', '/robot1/ouster/semantic_points')
        ]
    )
    
    # ROS 2 bag play node
    play_bag1 = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag1_path],
        output='screen'
    )
    play_bag2 = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag2_path],
        output='screen'
    )

    # RViz node
    rviz_node = TimerAction(
        period=5.0,  # Start RViz after a delay to ensure topics are available
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config_path]
            )
        ]
    )

    return LaunchDescription([
        # octomap_server_node, 
        robot_distance_checker_node,
        play_bag1,
        play_bag2,
        rviz_node
    ])

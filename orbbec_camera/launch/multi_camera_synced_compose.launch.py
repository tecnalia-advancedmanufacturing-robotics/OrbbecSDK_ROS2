import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import LoadComposableNodes


def generate_launch_description():
    # Include launch files
    package_dir = get_package_share_directory('orbbec_camera')
    launch_file_dir = os.path.join(package_dir, 'launch')
    config_file_dir = os.path.join(package_dir, 'config')
    config_file_path = os.path.join(config_file_dir, 'camera_params.yaml')

    # Create a shared container to hold composable nodes
    # for speed ups through intra process communication.
    shared_container_name = "shared_camera_container"
    shared_container = Node(
        name=shared_container_name,
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen')

    front_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'gemini_330_series.launch.py')
        ),
        launch_arguments={
            'camera_name': 'front_camera',
            'usb_port': '2-1',
            'device_num': '4',
            'sync_mode': 'software_triggering',
            'config_file_path': config_file_path,
            'attach_to_shared_component_container': 'True',
            'component_container_name': shared_container_name
        }.items()
    )

    left_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'gemini_330_series.launch.py')
        ),
        launch_arguments={
            'camera_name': 'left_camera',
            'usb_port': '2-2',
            'device_num': '4',
            'sync_mode': 'hardware_triggering',
            'config_file_path': config_file_path,
            'attach_to_shared_component_container': 'True',
            'component_container_name': shared_container_name
        }.items()
    )
    rear_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'gemini_330_series.launch.py')
        ),
        launch_arguments={
            'camera_name': 'rear_camera',
            'usb_port': '2-3',
            'device_num': '4',
            'sync_mode': 'hardware_triggering',
            'config_file_path': config_file_path,
            'attach_to_shared_component_container': 'True',
            'component_container_name': shared_container_name
        }.items()
    )
    right_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'gemini_330_series.launch.py')
        ),
        launch_arguments={
            'camera_name': 'right_camera',
            'usb_port': '2-5',
            'device_num': '4',
            'sync_mode': 'hardware_triggering',
            'config_file_path': config_file_path,
            'attach_to_shared_component_container': 'True',
            'component_container_name': shared_container_name
        }.items()
    )

    camera_topics_config = os.path.join(get_package_share_directory("orbbec_camera"), "config", "camera_topics.yaml")
    print("Camera topics config path: ", camera_topics_config)  # 打印路径以调试
    print(os.environ.get('LD_LIBRARY_PATH'))

    topic_statistics = ComposableNode(
        package='orbbec_camera',
        plugin='orbbec_camera::TopicStatistics',
        name='topic_statistics_compose_node',
        parameters=[camera_topics_config],
    )
    load_topic_statistics = LoadComposableNodes(
        target_container=shared_container_name,
        composable_node_descriptions=[topic_statistics]
    )

    # If you need more cameras, just add more launch_include here, and change the usb_port and device_num

    delay_action = TimerAction(
        period=3.0,  # 延迟 3 秒
        actions=[front_camera]
        )

    # Launch description
    ld = LaunchDescription([
        shared_container,
        rear_camera,
        left_camera,
        right_camera,
        # front_camera, # The primary camera should be launched at last
        delay_action,
        load_topic_statistics,
    ])

    return ld
import os
import launch
import launch_ros.actions
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return launch.LaunchDescription([
        # Right Camera Node
        launch_ros.actions.Node(
            package='opencv_cam', executable='opencv_cam_main',
            namespace='/right',
            parameters=[{'index': 2, 'fps': 10, 'width': 320, 'height': 240, 'camera_info_path': 'camera-info-right.ini'}],
            remappings=[('/right/image_raw', '/right/image_raw')]
        ),

        # Left Camera Node
        launch_ros.actions.Node(
            package='opencv_cam', executable='opencv_cam_main',
            namespace='/left',
            parameters=[{'index': 4, 'fps': 10, 'width': 320, 'height': 240, 'camera_info_path': 'camera-info-left.ini', 'camera_frame_id': 'test_frame_id'}],
            remappings=[('/left/image_raw', '/left/image_raw'), ('/left/camera_info', '/left/camera_info')]
        ),

        # Camera Info Republisher
        launch_ros.actions.Node(
            package='my_vision_package', executable='camera_info_republisher'
        ),

        # Image Processing Pipeline
        ComposableNodeContainer(
            name='image_proc_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='image_proc',
                    plugin='image_proc::RectifyNode',
                    name='left_rectify_node',
                    remappings=[
                        ('image', '/left/image_raw'),
                        ('image_rect', '/left/image_rect_color'),
                        ('camera_info', '/left/camera_info')
                    ],
                    parameters=[{
                        'camera_info_url': 'file:///ros2_ws/left.yaml',
                        'width': 320,
                        'height': 240,
                        'fps': 10
                    }]
                ),
                ComposableNode(
                    package='image_proc',
                    plugin='image_proc::RectifyNode',
                    name='right_rectify_node',
                    remappings=[
                        ('image', '/right/image_raw'),
                        ('camera_info', '/right/camera_info')
                    ],
                    parameters=[{
                        'camera_info_url': 'file:///ros2_ws/right.yaml',
                        'width': 320,
                        'height': 240,
                        'fps': 10
                    }]
                ),
                ComposableNode(
                    package='stereo_image_proc',
                    plugin='stereo_image_proc::PointCloudNode',
                    name='point_cloud_node',
                    remappings=[('left/image_rect_color', 'left/image_rect')],
                    parameters=[{
                        'approximate_sync': True,
                        'avoid_point_cloud_padding': True,
                        'width': 320,
                        'height': 240,
                        'fps': 10
                    }]
                ),
                ComposableNode(
                    package='stereo_image_proc',
                    plugin='stereo_image_proc::DisparityNode',
                    name='disparity_right_node',
                    namespace='/right/stereo',
                    remappings=[
                        ('left/image_rect', '/right/stereo/image_rect'),
                        ('right/image_rect', '/left/stereo/image_rect'),
                        ('left/camera_info', '/right/stereo/camera_info'),
                        ('right/camera_info', '/left/stereo/camera_info'),
                        ('disparity', '/disparity_right'),
                    ],
                ),

                # Rectify for left stereo (used by right disparity)
                ComposableNode(
                    package='image_proc',
                    plugin='image_proc::RectifyNode',
                    name='left_stereo_rectify_node',
                    namespace='/left/stereo',
                    remappings=[
                        ('image', '/left/image_raw'),
                        ('camera_info', '/left/camera_info')
                    ]
                ),

                # Rectify for right stereo (used by right disparity)
                ComposableNode(
                    package='image_proc',
                    plugin='image_proc::RectifyNode',
                    name='right_stereo_rectify_node',
                    namespace='/right/stereo',
                    remappings=[
                        ('image', '/right/image_raw_sync'),
                        ('camera_info', '/right/stereo/camera_info_sync')
                    ]
                ),

                # Reversed Disparity Node for WLS
                ComposableNode(
                    package='stereo_image_proc',
                    plugin='stereo_image_proc::DisparityNode',
                    name='disparity_right_node_simple',
                    remappings=[
                        ('left/image_rect', '/right/image_rect'),
                        ('right/image_rect', '/left/image_rect'),
                        ('left/camera_info', '/right/camera_info'),
                        ('right/camera_info', '/left/camera_info'),
                        ('disparity', '/disparity_right'),
                    ],
                ),


            ],
            output='screen'
        ),

        # Disparity Processing
        launch_ros.actions.Node(
            package='stereo_image_proc', executable='disparity_node',
            parameters=['disparity-params.yaml']
        ),

        # Stereo Image Processing Launch File
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(FindPackageShare("stereo_image_proc").find("stereo_image_proc"), "launch", "stereo_image_proc.launch.py")
            ),
            launch_arguments={'namespace': 'stereo', 'debug': 'true'}.items()
        ),

        # Disparity Viewer
        launch_ros.actions.Node(
            package='image_view', executable='disparity_view',
            remappings=[('image', '/disparity')]
        ),

        # Time Synchronizer
        launch_ros.actions.Node(
            package='my_vision_package', executable='time_synchronizer'
        ),
        
        #launch_ros.actions.Node(
        #    package='my_vision_package',
        #    executable='stereo_cluster_node',
        #    output='screen'
        #),
        
        # Static Transform Publisher
        launch_ros.actions.Node(
            package='tf2_ros', executable='static_transform_publisher',
            arguments=['0', '0', '4', '0', '1.5708', '1.5708', 'test_frame_id', 'test_child_frame_id']
        )
    ])

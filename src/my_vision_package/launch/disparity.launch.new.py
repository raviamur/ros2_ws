import os
import launch
import launch_ros.actions
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    container = launch_ros.actions.ComposableNodeContainer(
        name='stereo_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[

            # Rectify for left
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='left_rectify_node',
                namespace='/left',
                remappings=[
                    ('image', '/left/image_raw'),
                    ('camera_info', '/left/camera_info')
                ]
            ),

            # Rectify for right
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='right_rectify_node',
                namespace='/right',
                remappings=[
                    ('image', '/right/image_raw'),
                    ('camera_info', '/right/camera_info')
                ]
            ),

            # Stereo processing
            ComposableNode(
                package='stereo_image_proc',
                plugin='stereo_image_proc::DisparityNode',
                name='disparity_node',
                remappings=[
                    ('left/image_rect', '/left/image_rect'),
                    ('right/image_rect', '/right/image_rect'),
                    ('left/camera_info', '/left/camera_info'),
                    ('right/camera_info', '/right/camera_info'),
                    ('disparity', '/disparity'),
                ]
            ),

            # Rectify for right stereo (used by right disparity)
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='right_stereo_rectify_node',
                namespace='/right/stereo',
                remappings=[
                    ('image', '/right/image_raw'),
                    ('camera_info', '/right/camera_info')
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
        output='screen',
    )

    zone_wls_node = launch_ros.actions.Node(
        package='my_vision_package',
        executable='zone_based_wls_filter_node',
        name='zone_based_wls_filter_node',
        output='screen',
    )

    cluster_node = launch_ros.actions.Node(
        package='my_vision_package',
        executable='stereo_cluster_node_wls',
        name='stereo_cluster_node_wls',
        output='screen',
    )

    return launch.LaunchDescription([
        container,
        zone_wls_node,
        cluster_node,
    ])

import os
import launch
import launch_ros.actions
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

def generate_launch_description():
  
    return launch.LaunchDescription([
        # Right Camera Node
        launch_ros.actions.Node(
            package='opencv_cam', executable='opencv_cam_main',
            namespace='/right',
            parameters=[{'index': 2, 'fps': 10, 'width': 320, 'height': 240, 'camera_info_path': 'camera-info-right.ini'}],
            remappings=[('/right/image_raw', '/right/image_raw'),
            ]
        ),

        # Left Camera Node
        launch_ros.actions.Node(
            package='opencv_cam', executable='opencv_cam_main',
            namespace='/left',
            parameters=[{'index': 4, 'fps': 10, 'width': 320, 'height': 240, 'camera_info_path': 'camera-info-left.ini', 'camera_frame_id': 'test_frame_id'}],
            remappings=[('/left/image_raw', '/left/image_raw'), 
            ('/left/camera_info', '/left/camera_info'),
            ('/left/camera_info', '/camera_info')], 
            
        ),

        launch_ros.actions.Node(
            package='my_vision_package', executable='camera_info_republisher',
        ),

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
            ('camera_info', '/left/camera_info'),
            ],
            parameters=[{
            'qos_overrides./left/image_rect_color.reliability': 'best_effort',
            'qos_overrides./left/image_rect_color.history': 'keep_last',
            'qos_overrides./left/image_rect_color.depth': 1, 
            'camera_info_url': 'file:///ros2_ws/left.yaml',
            'width': 320,
            'height': 240,
            'fps': 10
            }],
            ),

            ComposableNode(
            package='image_proc',
            plugin='image_proc::RectifyNode',
            name='right_rectify_node',
            parameters=[{
            'camera_info_url': 'file:///ros2_ws/right.yaml',
            'width': 320,
            'height': 240,
            'fps': 10
            }],
            remappings=[
            ('image', '/right/image_raw'),
            ('camera_info', '/right/camera_info'),
                        #('image_rect', '/right/image_rect')
                    ],
                ),

              
                ComposableNode(
                    package='stereo_image_proc',
                    plugin='stereo_image_proc::PointCloudNode',
                    name='point_cloud_node',
                    parameters=[{
                        'approximate_sync': True,
                        'avoid_point_cloud_padding': True,
                        'width': 320,
                        'height': 240,
                        'fps': 10
                    }],
                    remappings=[
                        ('left/image_rect_color', 'left/image_rect')
                    ]
                ),
            ],
            output='screen'
        ),       

        # Stereo Image Processing
        launch_ros.actions.Node(
            package='stereo_image_proc', executable='disparity_node',
            parameters=['disparity-params.yaml']
        ),
                
       
        #Stereo Image Proc Launch File
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
            # No parameters or remappings required for this node
        )
    ])

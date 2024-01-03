import launch_ros

import launch


def generate_launch_description():
    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="kinect2_ros2",
                executable="kinect2_ros2_node",
                name="kinect2_ros2",
                namespace="kinect2"
            ),
            launch_ros.actions.Node(
                package="image_tools",
                executable="showimage",
                name="rgb_showimage",
                parameters=[{"window_name": "RGB"}],
                remappings=[("image", "kinect2/image_raw")],
            ),
            launch_ros.actions.Node(
                package="image_tools",
                executable="showimage",
                name="depth_showimage",
                parameters=[{"window_name": "Depth"}],
                remappings=[("image", "kinect2/depth/image_raw")],
            ),
        ]
    )

import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    publish_orientation = True

    tilt_detection_node =  Node(
        package='tilt_detection',
        executable='tilt_detection_node',
        parameters = [
            {"publish_orientation" : publish_orientation}
        ]
    )
    ld.add_action(tilt_detection_node)

    if publish_orientation :
        tf_broadcaster_node =  Node(
            package='orientation_viz',
            executable='tf_broadcaster'
        )
        ld.add_action(tf_broadcaster_node)

        rviz2 =  Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d'+str(os.path.realpath('/imu-orientation-estimator/src/orientation_viz/config/config.rviz'))]
        )
        ld.add_action(rviz2)


    return ld
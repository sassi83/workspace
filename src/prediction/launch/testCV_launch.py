import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # load parameters fom config/params.yaml
    config = os.path.join(
        get_package_share_directory('prediction'),
        'config',
        'params.yaml'
    )


    ld = LaunchDescription([
        Node(
            package='prediction',
            name='prediction_cv',
            executable='prediction_cv',
            parameters=[config],
            output='screen',
        ),
        Node(
            package='prediction',
            name='publisher_dummy_data',
            executable='publisher_dummy_data',
            parameters=[config],
            output='screen',
        ),
        Node(
            package='prediction',
            name='subsciber_predicted_list',
            executable='subsciber_predicted_list',
            output='screen',
        )
    ])
    return ld
import os 
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    config = os.path.join(

        get_package_share_directory('challenge_1'),
        'config',
        'params.yaml'
    )

    path_node = Node(
        package = 'challenge_1',
        executable = 'controller_car',
        output = 'screen',
        parameters = [config]
    )


    return LaunchDescription([path_node])


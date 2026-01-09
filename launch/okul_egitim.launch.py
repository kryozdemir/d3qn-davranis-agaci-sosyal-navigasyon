from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    paket = get_package_share_directory('d3qn_sosyal_navigasyon')

    parametreler = LaunchConfiguration('parametreler')
    senaryo = LaunchConfiguration('senaryo')
    bt_aktif = LaunchConfiguration('bt_aktif')

    gazebo_world = LaunchConfiguration('world')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': gazebo_world}.items()
    )

    node = Node(
        package='d3qn_sosyal_navigasyon',
        executable='ana_node',
        output='screen',
        parameters=[{
            'parametreler': parametreler,
            'senaryo': senaryo,
            'bt_aktif': bt_aktif,
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument('parametreler', default_value=os.path.join(paket, 'config', 'd3qn_parametreler.yaml')),
        DeclareLaunchArgument('senaryo', default_value=os.path.join(paket, 'config', 'senaryo_ayarlari', 'okul.yaml')),
        DeclareLaunchArgument('bt_aktif', default_value='true'),
        DeclareLaunchArgument('world', default_value=os.path.join(paket, 'worlds', 'okul.world')),
        gazebo,
        node
    ])

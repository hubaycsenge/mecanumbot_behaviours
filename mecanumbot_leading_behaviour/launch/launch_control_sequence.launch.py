import os 

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    rviz_config_dir = os.path.join( 
         get_package_share_directory('mecanumbot_description'),
        'rviz',
        'model.rviz')
    rviz = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen')
    map_name = 'ethodept_old'
    mecanumbot_description_pkg_share = get_package_share_directory('mecanumbot_description')
    param_file = os.path.join(mecanumbot_description_pkg_share, 'param', 'mecanumbot_custom_nav2.yaml')
    map_file = os.path.join(mecanumbot_description_pkg_share,'maps',map_name, f"{map_name}.yaml")

    # Declare a namespace argument
    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='mecanumbot',
        description='Namespace for the robot'
    )
    namespace = LaunchConfiguration('namespace')

    leading_bt = Node(
        package='mecanumbot_leading_behaviour',
        executable='control_leading_bt_node',
        name='ctrl_leading_bt',
        output='screen',
        namespace=namespace
    )

    nav_launch = PushRosNamespace(namespace)(
    IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'), 
                'launch', 'bringup_launch.py'
            )
        ),
        launch_arguments={
            "map": map_file,
            "params_file": param_file,
            "use_sim_time": "false",
        }.items()
    )
)

    
    return LaunchDescription([
        leading_bt,
        rviz,
        nav_launch
    ])

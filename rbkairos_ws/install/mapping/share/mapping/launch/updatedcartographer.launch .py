from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    # Package Directories
    pkg_name = 'mapping'
    pkg_dir = get_package_share_directory(pkg_name)
    xacro_pkg_name = 'robotnik_description'
    cartographer_pkg_dir = get_package_share_directory('cartographer_ros')

    # Process XACRO file from another package
    xacro_file_path = os.path.join(
        get_package_share_directory(xacro_pkg_name),
        'robots/rbkairos',
        'rbkairos_plus.urdf.xacro'
    )
    # Convert XACRO to URDF string
    urdf_content = xacro.process_file(xacro_file_path).toxml()

    # Launch Arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    configuration_directory = DeclareLaunchArgument(
        'configuration_directory',
        default_value=PathJoinSubstitution([pkg_dir, 'config']),
        description='Full path to folder containing Cartographer config files'
    )
    configuration_basename = DeclareLaunchArgument(
        'configuration_basename',
        default_value='rbkairos.lua',
        description='Name of the Cartographer configuration file'
    )

    # Get the share directory of the package containing the URDF
    urdf_pkg_name = 'robotnik_description'
    urdf_pkg_dir = get_package_share_directory(urdf_pkg_name)

    # URDF file path in the external package
    urdf_file = DeclareLaunchArgument(
        'urdf_file',
        default_value=PathJoinSubstitution([
            urdf_pkg_dir,
            'urdf',
            'rbkairos.urdf'
        ]),
        description='Path to URDF file in another package'
    )

    # Nodes
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': urdf_content},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {
                'configuration_directory': LaunchConfiguration('configuration_directory'),  # Pass directory as parameter
                'configuration_basename': LaunchConfiguration('configuration_basename')    # Pass basename as parameter
            }
        ],
        remappings=[
            ('scan', 'front_scan'),
            ('scan_1', 'rear_scan')
        ],
        output='screen'
    )

    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        parameters=[
            {'resolution': 0.05},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen'
    )

    # RViz2 Node with Cartographer's default configuration
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([
            cartographer_pkg_dir,
            'configuration_files',
            'demo_2d.rviz'
        ])],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        use_sim_time,
        configuration_directory,
        configuration_basename,
        urdf_file,
        robot_state_publisher,
        cartographer_node,
        occupancy_grid_node,
        rviz2_node
    ])


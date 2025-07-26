from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os
import xacro  # Import XACRO processor

def generate_launch_description():
    # Package Directories
    pkg_name = 'mapping'  # Replace with your package name
    pkg_dir = get_package_share_directory(pkg_name)
    xacro_pkg_name = 'robotnik_description'  # Package containing the .xacro file
    cartographer_pkg_dir = get_package_share_directory('cartographer_ros')  # For RViz config

    # Process XACRO file from another package
    xacro_file_path = os.path.join(
        get_package_share_directory(xacro_pkg_name),
        'robots/rbkairos',  # Subdirectory (e.g., urdf/, description/)
        'rbkairos.urdf.xacro'  # Replace with your XACRO filename
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
        default_value='/home/hima/bachelors/rbkairos_ws/src/mapping/config',  # Replace with the actual absolute path
        #default_value=PathJoinSubstitution([pkg_dir, 'config']),
        description='Full path to folder containing Cartographer config files'
    )
    configuration_basename = DeclareLaunchArgument(
        'configuration_basename',
        default_value='rbkairos.lua',  # Replace with your config filename
        description='Name of the Cartographer configuration file'
    )
    
    # Get the share directory of the package containing the URDF
    urdf_pkg_name = 'robotnik_description'  # Replace with the actual package name
    urdf_pkg_dir = get_package_share_directory(urdf_pkg_name)
    
    # URDF file path in the external package
    urdf_file = DeclareLaunchArgument(
        'urdf_file',
        default_value=PathJoinSubstitution([
            urdf_pkg_dir,  # Path to "another_pkg"
            'urdf',        # Subdirectory (e.g., urdf/, robots/)
            'rbkairos.urdf' # Actual URDF filename
        ]),
        description='Path to URDF file in another package'
    )


    # urdf_file = DeclareLaunchArgument(
    #     'urdf_file',
    #     default_value=PathJoinSubstitution([pkg_dir, 'urdf', 'my_robot.urdf']),
    #     description='Path to URDF file'
    # )

    # Nodes
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': urdf_content},  # Use processed URDF
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    # robot_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     parameters=[
    #         {'robot_description': LaunchConfiguration('urdf_file')},
    #         {'use_sim_time': LaunchConfiguration('use_sim_time')}
    #     ]
    # )

    # cartographer_node = Node(
    #     package='cartographer_ros',
    #     executable='cartographer_node',
    #     parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    #     arguments=[
    #         '-configuration_directory', LaunchConfiguration('configuration_directory'),
    #         '-configuration_basename', LaunchConfiguration('configuration_basename')
    #     ],
    #     remappings=[
    #         ('scan', 'front_scan'),  # First laser topic
    #         ('scan_1', 'rear_scan')  # Second laser topic
    #     ],
    #     output='screen'
    # )
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=[
            '-configuration_directory', '/home/hima/bachelors/rbkairos_ws/src/mapping/config', #add config folder path
            '-configuration_basename', 'rbkairos.lua'
       # '   -configuration_basename', 'rbkairos.lua'
        ],
        remappings=[
            ('scan', '/robot/front_laser/scan'),  # First laser topic
            ('scan_1', '/robot/rear_laser/scan'),  # Second laser topic
            ('odom', '/robot/robotnik_base_controller/odom') #odometry topic
        ],
        output='screen'
    )   

    # cartographer_node = Node(
    #     package='cartographer_ros',
    #     executable='cartographer_node',
    #     parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    #     arguments=[
    #         '-configuration_directory', PathJoinSubstitution([pkg_name, 'config']),
    #         '-configuration_basename', 'rbkairos.lua'  # Your Cartographer config
    #     ],
    #     output='screen'
    # )

    # cartographer_node = Node(
    #     package='cartographer_ros',
    #     executable='cartographer_node',
    #     parameters=[
    #         {'use_sim_time': LaunchConfiguration('use_sim_time')}
    #     ],
    #     arguments=[
    #         '-configuration_directory', LaunchConfiguration('configuration_directory'),
    #         '-configuration_basename', LaunchConfiguration('configuration_basename')
    #     ],
    #     output='screen'
    # )

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
            get_package_share_directory('cartographer_ros'),
            'configuration_files',
            'cartographer.rviz'  # Use the default config
            #'demo_2d.rviz'  # Pre-configured RViz file for Cartographer
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

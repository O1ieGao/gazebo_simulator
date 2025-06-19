import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 获取包的路径
    robot_pkg_dir = get_package_share_directory('robot')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    
    # 文件路径
    sdf_path = os.path.join(robot_pkg_dir, 'urdf', 'pioneer2dx.sdf')
    urdf_path = os.path.join(robot_pkg_dir, 'urdf', 'pioneer2dx.urdf')
    
    # 启动参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # 读取URDF文件
    with open(urdf_path, 'r') as infp:
        robot_description = infp.read()
    
    # 启动Gazebo服务器
    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'verbose': 'true',
            'pause': 'false'
        }.items()
    )
    
    # 启动Gazebo客户端
    start_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gzclient.launch.py')
        )
    )
    
    # 生成机器人模型
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-file', sdf_path,
            '-entity', 'pioneer2dx_lidar',
            '-x', '0',
            '-y', '0',
            '-z', '0.2'
        ],
        output='screen'
    )
    
    # Robot State Publisher - 发布TF变换
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # 启动RViz2
    rviz_config_file = os.path.join(robot_pkg_dir, 'config', 'pioneer_lidar.rviz')
    start_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        start_gazebo_server,
        start_gazebo_client,
        robot_state_publisher,  # 添加这个来发布TF
        spawn_entity,
        start_rviz
    ]) 
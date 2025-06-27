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
    urdf_path = os.path.join(robot_pkg_dir, 'urdf', 'pioneer2dx_dual.urdf')
    
    world_path = os.path.join(robot_pkg_dir, 'worlds', 'random_objects.world')
    
    # 启动参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # 读取URDF文件
    try:
        with open(urdf_path, 'r') as infp:
            robot_description = infp.read()
    except:
        robot_description = ""
        print(f"警告：无法读取URDF文件 {urdf_path}")
    
    # 启动Gazebo服务器
    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world': world_path,
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
            '-entity', 'pioneer2dx_dual_lidar',
            '-x', '0',
            '-y', '0',
            '-z', '0.2'
        ],
        output='screen'
    )
    
    # Robot State Publisher - 发布TF变换（重要！）
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': use_sim_time}
        ]
    ) if robot_description else None
    
    # 静态TF发布器作为备用方案
    static_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_link_tf',
        arguments=['0.2', '0', '0.09', '0', '0', '0', 'chassis', 'laser_link'],
        output='screen'
    )
    
    # 电量发布节点 - 使用自定义yhs_can_msgs/BatteryInfo消息
    battery_publisher = Node(
        package='robot',
        executable='battery_publisher',
        name='battery_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    task_simplex_2d_server = Node(
        package='robot',
        executable='task_simplex_2d_server',
        name='task_simplex_2d_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # RViz配置文件路径
    rviz_config_file = os.path.join(robot_pkg_dir, 'config', 'pioneer_camera_2d_3d.rviz')
    
    # 启动RViz2
    start_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # 构建launch描述
    launch_nodes = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        start_gazebo_server,
        start_gazebo_client,
        static_tf_publisher,  # 添加静态TF发布器
        spawn_entity,
        battery_publisher,    # 添加电量发布节点
        task_simplex_2d_server,
        start_rviz
    ]
    
    # 如果有URDF文件，添加robot_state_publisher
    if robot_state_publisher:
        launch_nodes.insert(-2, robot_state_publisher)
    
    return LaunchDescription(launch_nodes) 
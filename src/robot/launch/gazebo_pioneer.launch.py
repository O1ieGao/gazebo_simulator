import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    '''
    load pioneer2dx model in empty world
    '''
    # 获取包的路径
    robot_pkg_dir = get_package_share_directory('robot')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    
    # SDF模型文件路径
    sdf_path = os.path.join(robot_pkg_dir, 'urdf', 'pioneer2dx.sdf')
    
    # 世界文件路径
    world_path = os.path.join(os.path.dirname(robot_pkg_dir), '..', '..', 'worlds', 'empty.world')
    
    # 启动Gazebo服务器和客户端
    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            # 'world': world_path,
            'verbose': 'true'
        }.items()
    )
    
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
            '-entity', 'pioneer2dx',
            '-x', '0',
            '-y', '0',
            '-z', '0.5'
        ],
        output='screen'
    )
    
    return LaunchDescription([
        start_gazebo_server,
        start_gazebo_client,
        spawn_entity
    ]) 
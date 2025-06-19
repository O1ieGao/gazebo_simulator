# Gazebo simulator

本项目是一个 ROS 2 工作空间，用于在 Gazebo 中对 Pioneer 2-DX 机器人进行仿真。

## 项目结构

本工作空间主要包含以下功能包：

*   `src/robot`: 核心功能包，包含了用于启动仿真、定义机器人模型和配置可视化环境的所有文件。

## 环境要求

*   ROS 2 Humble
*   Gazebo
    ```bash
    sudo apt install ros-humble-gazebo-ros-pkgs
    ```

## 编译工作空间

在第一次使用或修改代码后，需要在工作空间根目录（`ros2_ws/`）下执行以下命令进行编译：

```bash
colcon build
```

## 运行仿真

每次运行前，请确保在新终端中已经配置好工作空间环境：

```bash
source install/setup.bash
```

本项目提供了多种启动配置，以满足不同需求。

### 1. 仅启动Gazebo和机器人模型

此命令会启动Gazebo仿真环境，并在其中生成一个Pioneer机器人模型。这适用于只希望观察机器人物理行为的场景。

```bash
ros2 launch robot gazebo_pioneer.launch.py
```

### 2. 启动单激光雷达机器人仿真 (带RViz)

这是标准的、完整的单激光雷达机器人仿真启动脚本。它会：
*   启动Gazebo和机器人模型。
*   启动 `robot_state_publisher` 来发布机器人的TF坐标变换。
*   启动 RViz 并加载预设配置，用于可视化机器人模型和传感器数据。

```bash
ros2 launch robot pioneer_3d.launch.py
```

### 3. 启动双激光雷达机器人仿真 (带RViz)

此命令用于启动配备双激光雷达的机器人版本。功能与上一个类似，但加载的是双激光雷达的机器人模型和对应的RViz配置。

```bash
ros2 launch robot pioneer_camera_2d_3d.launch.py
```

## `robot` 包详细结构

*   `launch/`: 包含所有用于启动仿真的 `.launch.py` 文件。
*   `urdf/`: 包含机器人的模型文件。
    *   `.sdf` 文件：用于Gazebo加载物理模型和插件。
    *   `.urdf` 文件：用于 `robot_state_publisher` 读取机器人关节结构并发布TF。
*   `config/`: 包含RViz的配置文件 (`.rviz`)。 
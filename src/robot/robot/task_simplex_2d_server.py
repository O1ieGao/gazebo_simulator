#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from hrstek_msgs.srv import TaskSimplex2dSrv
from hrstek_msgs.msg import TaskSimplex2dMsg, TrajectoryPoint2dMsg, Trajectory2dMsg, Action
from std_msgs.msg import Header
from geometry_msgs.msg import Twist, PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import math
import time
import threading

class PIDController:
    """简单的PID控制器"""
    def __init__(self, kp, ki, kd, max_output=None, min_output=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.min_output = min_output
        
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = None
    
    def compute(self, error, current_time=None):
        if current_time is None:
            current_time = time.time()
        
        if self.prev_time is None:
            self.prev_time = current_time
            dt = 0.1  # 默认时间步长
        else:
            dt = current_time - self.prev_time
            if dt <= 0:
                dt = 0.001  # 防止除零
        
        # 积分项
        self.integral += error * dt
        
        # 微分项
        derivative = (error - self.prev_error) / dt
        
        # PID输出
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        
        # 限制输出范围
        if self.max_output is not None:
            output = min(output, self.max_output)
        if self.min_output is not None:
            output = max(output, self.min_output)
        
        # 更新状态
        self.prev_error = error
        self.prev_time = current_time
        
        return output
    
    def reset(self):
        """重置PID控制器状态"""
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = None

class TaskSimplex2dServer(Node):
    def __init__(self):
        super().__init__('task_simplex_2d_server')
        
        # 创建service server
        self.service = self.create_service(
            TaskSimplex2dSrv,
            'uagentsim/execute_navigation',
            self.handle_task_simplex_request
        )
        
        # 创建发布器用于控制机器人
        self.cmd_vel_publisher = self.create_publisher(
            Twist, 
            '/uagentsim/cmd_vel', 
            10
        )
        
        # 订阅机器人当前位置
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/uagentsim/odom',
            self.odom_callback,
            10
        )
        
        # 机器人当前状态 (从odometry获取)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.odom_received = False
        self.odom_lock = threading.Lock()  # 保护odometry数据的锁
        
        # 移动参数
        self.move_tolerance = 0.15  # 位置容差(米) - 稍微放宽
        self.angle_tolerance = 0.08  # 角度容差(弧度) - 稍微放宽
        self.max_linear_speed = 0.8    # 最大线速度(米/秒)
        self.max_angular_speed = 1.0   # 最大角速度(弧度/秒)
        
        # 稳定性检查参数
        self.stability_check_duration = 0.5  # 稳定性检查持续时间(秒)
        self.stability_check_samples = 5     # 连续满足条件的次数
        
        # PID控制器
        # 线速度PID: 控制前进速度
        self.linear_pid = PIDController(
            kp=1.0,    # 比例增益
            ki=0.1,    # 积分增益
            kd=0.05,   # 微分增益
            max_output=self.max_linear_speed,
            min_output=0.0
        )
        
        # 角速度PID: 控制转向速度
        self.angular_pid = PIDController(
            kp=2.0,    # 比例增益 - 角度控制需要更强的响应
            ki=0.2,    # 积分增益
            kd=0.1,    # 微分增益
            max_output=self.max_angular_speed,
            min_output=-self.max_angular_speed
        )
        
        self.get_logger().info('TaskSimplex2d服务器启动，监听服务: uagentsim/execute_navigation')
        self.get_logger().info('订阅odometry: /uagentsim/odom')
        self.get_logger().info('发布控制命令: /uagentsim/cmd_vel')
        self.get_logger().info('使用PID控制器进行精确控制')
        self.get_logger().info('使用服务类型: hrstek_msgs/srv/TaskSimplex2dSrv')
    
    def odom_callback(self, msg):
        """处理odometry消息，更新机器人当前位置"""
        
        with self.odom_lock:
            # 获取位置
            self.current_x = msg.pose.pose.position.x
            self.current_y = msg.pose.pose.position.y
            
            # 获取朝向（从四元数转换为欧拉角）
            orientation = msg.pose.pose.orientation
            quat = [orientation.x, orientation.y, orientation.z, orientation.w]
            _, _, self.current_theta = euler_from_quaternion(quat)
            
            self.odom_received = True
    
    def get_current_pose(self):
        """线程安全地获取当前位置"""
        with self.odom_lock:
            return self.current_x, self.current_y, self.current_theta
    
    def wait_for_odometry(self, timeout=5.0):
        """等待接收到odometry数据"""
        
        start_time = time.time()
        while not self.odom_received and (time.time() - start_time) < timeout:
            time.sleep(0.1)
        
        return self.odom_received
    
    def wait_for_stability(self, target_x, target_y, target_theta, check_position=True, check_orientation=True):
        """等待机器人稳定在目标位置"""
        
        self.get_logger().info('等待机器人稳定...')
        stable_count = 0
        required_stable_count = self.stability_check_samples
        
        start_time = time.time()
        max_wait_time = 2.0  # 最大等待时间
        
        while (time.time() - start_time) < max_wait_time and stable_count < required_stable_count:
            current_x, current_y, current_theta = self.get_current_pose()
            
            position_ok = True
            orientation_ok = True
            
            if check_position:
                distance_error = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
                position_ok = distance_error < self.move_tolerance
            
            if check_orientation:
                angle_error = abs(self.normalize_angle(target_theta - current_theta))
                orientation_ok = angle_error < self.angle_tolerance
            
            if position_ok and orientation_ok:
                stable_count += 1
                self.get_logger().info(f'稳定性检查 {stable_count}/{required_stable_count}')
            else:
                stable_count = 0  # 重置计数器
                if check_position and not position_ok:
                    self.get_logger().info(f'位置不稳定，距离误差: {distance_error:.3f}m')
                if check_orientation and not orientation_ok:
                    self.get_logger().info(f'朝向不稳定，角度误差: {math.degrees(angle_error):.1f}°')
            
            time.sleep(0.1)  # 100ms间隔检查
        
        is_stable = stable_count >= required_stable_count
        self.get_logger().info(f'稳定性检查完成: {"稳定" if is_stable else "不稳定"}')
        return is_stable
    
    def handle_task_simplex_request(self, request, response):
        """处理TaskSimplex2d服务请求"""
        
        task = request.task_simplex
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('接收到TaskSimplex2d请求')
        self.get_logger().info(f'任务ID: {task.id}, 任务名称: "{task.name}"')
        
        # 等待接收到odometry数据
        if not self.wait_for_odometry():
            self.get_logger().error('未能接收到odometry数据，无法执行任务')
            response.confirm = False
            return response
        
        current_x, current_y, current_theta = self.get_current_pose()
        self.get_logger().info(f'当前机器人位置: ({current_x:.3f}, {current_y:.3f}, {math.degrees(current_theta):.1f}°)')
        
        # 解析task_trajectory中的points
        trajectory_points = task.task_trajectory.points
        self.get_logger().info(f'轨迹包含 {len(trajectory_points)} 个路径点')
        
        if len(trajectory_points) == 0:
            self.get_logger().warning('轨迹为空，无需执行')
            response.confirm = True
            return response
        
        # 在单独的线程中执行轨迹，避免阻塞service回调
        def execute_trajectory_thread():
            success = self.execute_trajectory(trajectory_points)
            if success:
                self.get_logger().info('轨迹执行完成')
            else:
                self.get_logger().error('轨迹执行失败')
        
        # 启动执行线程
        trajectory_thread = threading.Thread(target=execute_trajectory_thread)
        trajectory_thread.daemon = True
        trajectory_thread.start()
        
        # 立即返回确认（表示服务已接收请求）
        response.confirm = True
        self.get_logger().info('任务已接收，开始执行')
        self.get_logger().info('=' * 60)
        
        return response
    
    def execute_trajectory(self, points):
        """执行轨迹中的所有点"""
        
        try:
            for i, point in enumerate(points):
                current_x, current_y, current_theta = self.get_current_pose()
                
                self.get_logger().info(f'执行第 {i+1}/{len(points)} 个路径点')
                self.get_logger().info(f'  当前位置: ({current_x:.3f}, {current_y:.3f}, {math.degrees(current_theta):.1f}°)')
                self.get_logger().info(f'  目标位置: ({point.x:.3f}, {point.y:.3f}, {math.degrees(point.theta):.1f}°)')
                
                # 重置PID控制器
                self.linear_pid.reset()
                self.angular_pid.reset()
                
                # 移动到目标位置
                if not self.move_to_position(point.x, point.y, point.theta):
                    self.get_logger().error(f'移动到第 {i+1} 个点失败')
                    return False
                
                # 执行该点的动作（如果有）
                if point.actions:
                    self.get_logger().info(f'  执行 {len(point.actions)} 个动作')
                    self.execute_actions(point.actions)
                
                self.get_logger().info(f'  第 {i+1} 个路径点执行完成')
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'执行轨迹时发生错误: {str(e)}')
            return False
    
    def move_to_position(self, target_x, target_y, target_theta):
        """使用PID控制器移动到目标位置"""
        
        self.get_logger().info(f'开始移动到位置: ({target_x:.3f}, {target_y:.3f}, {math.degrees(target_theta):.1f}°)')
        
        move_start_time = time.time()
        max_move_time = 30.0  # 最大移动时间30秒
        
        # 第一阶段：移动到目标位置（同时调整朝向）
        while time.time() - move_start_time < max_move_time:
            current_time = time.time()
            current_x, current_y, current_theta = self.get_current_pose()
            
            # 计算位置误差
            dx = target_x - current_x
            dy = target_y - current_y
            distance_error = math.sqrt(dx*dx + dy*dy)
            
            # 如果已经到达目标位置，退出移动循环
            if distance_error < self.move_tolerance:
                self.get_logger().info(f'接近目标位置，距离误差: {distance_error:.3f}m')
                break
            
            # 计算目标角度（朝向目标点的角度）
            target_angle_to_goal = math.atan2(dy, dx)
            angle_error = self.normalize_angle(target_angle_to_goal - current_theta)
            
            # 使用PID控制器计算速度
            linear_speed = self.linear_pid.compute(distance_error, current_time)
            angular_speed = self.angular_pid.compute(angle_error, current_time)
            
            # 如果角度误差太大，减少线速度，增强转向
            if abs(angle_error) > 0.5:  # 约28.6度
                linear_speed *= 0.3  # 大幅减少前进速度
            elif abs(angle_error) > 0.2:  # 约11.5度
                linear_speed *= 0.6  # 适当减少前进速度
            
            # 创建并发布控制命令
            twist = Twist()
            twist.linear.x = linear_speed
            twist.angular.z = angular_speed
            
            self.cmd_vel_publisher.publish(twist)
            
            # 每秒打印一次状态
            if int((current_time - move_start_time) * 10) % 10 == 0:
                self.get_logger().info(
                    f'移动中: 距离误差={distance_error:.3f}m, 角度误差={math.degrees(angle_error):.1f}°, '
                    f'线速度={linear_speed:.2f}m/s, 角速度={math.degrees(angular_speed):.1f}°/s'
                )
            
            time.sleep(0.05)  # 20Hz控制频率
        
        # 停止移动
        self.stop_robot()
        
        # 等待机器人稳定在目标位置
        if not self.wait_for_stability(target_x, target_y, target_theta, check_position=True, check_orientation=False):
            self.get_logger().warning('机器人在目标位置不够稳定，继续执行朝向调整')
        
        # 第二阶段：精确调整最终朝向
        self.get_logger().info('开始调整最终朝向')
        
        # 重置角度PID控制器
        self.angular_pid.reset()
        
        orient_start_time = time.time()
        max_orient_time = 10.0
        
        while time.time() - orient_start_time < max_orient_time:
            current_time = time.time()
            current_x, current_y, current_theta = self.get_current_pose()
            
            # 计算最终朝向误差
            final_angle_error = self.normalize_angle(target_theta - current_theta)
            
            if abs(final_angle_error) < self.angle_tolerance:
                self.get_logger().info(f'接近目标朝向，角度误差: {math.degrees(final_angle_error):.1f}°')
                break
            
            # 使用PID控制角速度
            angular_speed = self.angular_pid.compute(final_angle_error, current_time)
            
            twist = Twist()
            twist.linear.x = 0.0  # 只转向，不前进
            twist.angular.z = angular_speed
            
            self.cmd_vel_publisher.publish(twist)
            
            # 每秒打印一次状态
            if int((current_time - orient_start_time) * 5) % 5 == 0:
                self.get_logger().info(f'调整朝向: 角度误差={math.degrees(final_angle_error):.1f}°, 角速度={math.degrees(angular_speed):.1f}°/s')
            
            time.sleep(0.05)
        
        # 停止机器人
        self.stop_robot()
        
        # 等待机器人完全稳定
        if not self.wait_for_stability(target_x, target_y, target_theta, check_position=True, check_orientation=True):
            self.get_logger().warning('机器人未能在目标位置完全稳定')
        
        # 最终位置检查
        final_x, final_y, final_theta = self.get_current_pose()
        final_distance = math.sqrt((target_x - final_x)**2 + (target_y - final_y)**2)
        final_angle_diff = abs(self.normalize_angle(target_theta - final_theta))
        
        success = (final_distance < self.move_tolerance and final_angle_diff < self.angle_tolerance)
        
        if success:
            self.get_logger().info(f'成功到达目标位置！最终误差: 距离={final_distance:.3f}m, 角度={math.degrees(final_angle_diff):.1f}°')
        else:
            self.get_logger().warning(f'未能精确到达目标位置。最终误差: 距离={final_distance:.3f}m, 角度={math.degrees(final_angle_diff):.1f}°')
            # 即使不够精确，也不算失败，继续执行下一个点
            self.get_logger().info('继续执行下一个路径点')
            success = True
        
        return success
    
    def stop_robot(self):
        """停止机器人"""
        twist = Twist()
        self.cmd_vel_publisher.publish(twist)
        time.sleep(0.2)  # 确保停止命令被发送
    
    def normalize_angle(self, angle):
        """将角度标准化到[-π, π]范围"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def execute_actions(self, actions):
        """执行路径点的动作序列"""
        
        for i, action in enumerate(actions):
            self.get_logger().info(f'    执行动作 {i+1}: 命令={action.command}')
            
            if action.command == 101:  # 等待动作
                wait_duration = action.params_wait.duration
                self.get_logger().info(f'    等待 {wait_duration} 秒')
                time.sleep(wait_duration)
            
            # 这里可以添加更多动作类型的处理
            # elif action.command == 102:
            #     # 其他动作类型
            #     pass

def main(args=None):
    rclpy.init(args=args)
    task_server = TaskSimplex2dServer()
    
    try:
        rclpy.spin(task_server)
    except KeyboardInterrupt:
        task_server.get_logger().info('TaskSimplex2d服务器正在关闭...')
    
    task_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 
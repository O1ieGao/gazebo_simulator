#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from yhs_can_msgs.msg import BatteryInfo
from std_msgs.msg import Header
import math
import time

class BatteryPublisher(Node):
    def __init__(self):
        super().__init__('battery_publisher')
        
        # 创建发布器
        self.publisher = self.create_publisher(
            BatteryInfo, 
            '/uagentsim/hrstek_publisher/Battery_data_topic', 
            10
        )
        
        # 创建定时器，每秒发布一次
        self.timer = self.create_timer(1.0, self.publish_battery_data)
        
        # 初始化电量参数
        self.initial_percentage = 100  # 初始电量100%
        self.start_time = time.time()
        self.discharge_rate = 0.05  # 每分钟放电0.05%
        self.cycle_count = 25  # 充电循环次数
        
        self.get_logger().info('电量发布节点启动，发布到: /uagentsim/hrstek_publisher/Battery_data_topic')
        self.get_logger().info('使用消息类型: yhs_can_msgs/msg/BatteryInfo')
    
    def publish_battery_data(self):
        msg = BatteryInfo()
        
        # 设置消息头
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "battery"
        
        # 计算当前电量百分比（模拟缓慢放电）
        elapsed_minutes = (time.time() - self.start_time) / 60.0
        current_percentage = max(0, self.initial_percentage - int(self.discharge_rate * elapsed_minutes))
        
        if current_percentage == 0:
            self.start_time = time.time()
            current_percentage = 100

        # 基本电池信息
        msg.percentage = current_percentage  # 0-100
        msg.voltage = 20.5 + (current_percentage / 100.0) * 4.5  # 20.5V到25.0V (典型锂电池组)
        msg.current = -2.5 - (current_percentage / 100.0) * 1.0   # 电流消耗 -2.5A到-3.5A
        
        # 容量信息
        msg.capacity = 50.0  # 总容量 50Ah
        msg.capacity_remain = msg.capacity * (current_percentage / 100.0)  # 剩余容量
        
        # 充电状态
        msg.charging_status = False  # 不在充电
        msg.charger_connected = False  # 充电器未连接
        
        # 循环次数
        msg.circle_count = self.cycle_count
        
        # 温度（模拟根据负载变化）
        base_temp = 25.0  # 基础温度25°C
        load_temp = (100 - current_percentage) * 0.2  # 电量越低温度越高
        msg.temperature = base_temp + load_temp
        
        # 电源健康状态 (使用简单的健康指标)
        if current_percentage > 80:
            health = [1]  # 很好
        elif current_percentage > 50:
            health = [2]  # 良好  
        elif current_percentage > 20:
            health = [3]  # 一般
        elif current_percentage > 5:
            health = [4]  # 差
        else:
            health = [5]  # 很差
            
        msg.power_supply_health = health
        
        # 发布消息
        self.publisher.publish(msg)
        
        # 每隔10秒打印一次详细日志
        if int(elapsed_minutes * 60) % 10 == 0:
            self.get_logger().info(
                f'电量状态 - 百分比: {current_percentage}%, '
                f'电压: {msg.voltage:.2f}V, '
                f'电流: {msg.current:.2f}A, '
                f'剩余容量: {msg.capacity_remain:.1f}Ah, '
                f'温度: {msg.temperature:.1f}°C'
            )

def main(args=None):
    rclpy.init(args=args)
    battery_publisher = BatteryPublisher()
    
    try:
        rclpy.spin(battery_publisher)
    except KeyboardInterrupt:
        battery_publisher.get_logger().info('电量发布节点正在关闭...')
    
    battery_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 
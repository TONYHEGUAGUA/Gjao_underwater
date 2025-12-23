#!/usr/bin/env python3
# robot_controller.py - 机器人控制器（集成你的手动控制代码）

import rospy
import math
import threading
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class RobotController:
    def __init__(self):
        # 集成你的控制参数
        self.PWM_NEUTRAL = 1500
        self.PWM_MIN = 1000
        self.PWM_MAX = 2000
        self.pwm_drive_diff = 100
        self.pwm_turn_diff = 100
        
        # ROS发布器
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.vacuum_pub = rospy.Publisher('/vacuum_control', Bool, queue_size=10)
        
        # 控制状态
        self.current_speed = 0.0
        self.current_turn = 0.0
        self.vacuum_state = False
        
        # 控制参数
        self.max_linear_speed = 0.5  # m/s
        self.max_angular_speed = 1.0  # rad/s
        
        print("机器人控制器初始化完成")
    
    def pwm_to_velocity(self, pwm):
        """将PWM值转换为速度值"""
        # 将PWM值（1000-2000，中立1500）转换为速度（-1到1）
        if pwm == self.PWM_NEUTRAL:
            return 0.0
        elif pwm > self.PWM_NEUTRAL:
            # 前进方向
            normalized = (pwm - self.PWM_NEUTRAL) / (self.PWM_MAX - self.PWM_NEUTRAL)
            return normalized * self.max_linear_speed
        else:
            # 后退方向
            normalized = (self.PWM_NEUTRAL - pwm) / (self.PWM_NEUTRAL - self.PWM_MIN)
            return -normalized * self.max_linear_speed
    
    def velocity_to_pwm(self, velocity):
        """将速度值转换为PWM值"""
        # 将速度（-1到1）转换为PWM值（1000-2000）
        if velocity == 0:
            return self.PWM_NEUTRAL
        elif velocity > 0:
            # 前进
            normalized = velocity / self.max_linear_speed
            pwm = self.PWM_NEUTRAL + normalized * (self.PWM_MAX - self.PWM_NEUTRAL)
            return int(max(self.PWM_NEUTRAL, min(self.PWM_MAX, pwm)))
        else:
            # 后退
            normalized = abs(velocity) / self.max_linear_speed
            pwm = self.PWM_NEUTRAL - normalized * (self.PWM_NEUTRAL - self.PWM_MIN)
            return int(max(self.PWM_MIN, min(self.PWM_NEUTRAL, pwm)))
    
    def send_velocity_command(self, linear_x, angular_z):
        """发送速度命令"""
        try:
            # 限制速度范围
            linear_x = max(-self.max_linear_speed, min(self.max_linear_speed, linear_x))
            angular_z = max(-self.max_angular_speed, min(self.max_angular_speed, angular_z))
            
            # 创建ROS消息
            twist = Twist()
            twist.linear.x = linear_x
            twist.angular.z = angular_z
            
            # 发布命令
            self.cmd_vel_pub.publish(twist)
            
            # 更新当前状态
            self.current_speed = linear_x
            self.current_turn = angular_z
            
            # 打印调试信息
            pwm_left, pwm_right = self.calculate_pwm(linear_x, angular_z)
            print(f"速度命令: linear={linear_x:.2f}, angular={angular_z:.2f}, "
                  f"PWM: L={pwm_left}, R={pwm_right}")
            
            return True
            
        except Exception as e:
            print(f"发送速度命令失败: {e}")
            return False
    
    def calculate_pwm(self, linear_x, angular_z):
        """根据线速度和角速度计算左右轮PWM值"""
        # 基础速度转换
        base_pwm = self.velocity_to_pwm(linear_x)
        
        # 转向调整
        if angular_z == 0:
            # 直行
            left_pwm = base_pwm
            right_pwm = base_pwm
        elif angular_z > 0:
            # 左转：减小左轮，增加右轮
            turn_adjust = int(abs(angular_z) / self.max_angular_speed * self.pwm_turn_diff)
            left_pwm = base_pwm - turn_adjust
            right_pwm = base_pwm + turn_adjust
        else:
            # 右转：增加左轮，减小右轮
            turn_adjust = int(abs(angular_z) / self.max_angular_speed * self.pwm_turn_diff)
            left_pwm = base_pwm + turn_adjust
            right_pwm = base_pwm - turn_adjust
        
        # 确保在安全范围内
        left_pwm = max(self.PWM_MIN, min(self.PWM_MAX, left_pwm))
        right_pwm = max(self.PWM_MIN, min(self.PWM_MAX, right_pwm))
        
        return left_pwm, right_pwm
    
    def control_vacuum(self, state):
        """控制负压吸附"""
        try:
            msg = Bool()
            msg.data = bool(state)
            self.vacuum_pub.publish(msg)
            
            self.vacuum_state = bool(state)
            print(f"负压吸附状态: {'开启' if state else '关闭'}")
            
            return True
            
        except Exception as e:
            print(f"控制负压吸附失败: {e}")
            return False
    
    def get_status(self):
        """获取当前状态"""
        return {
            'speed': self.current_speed,
            'turn': self.current_turn,
            'vacuum': self.vacuum_state,
            'max_linear_speed': self.max_linear_speed,
            'max_angular_speed': self.max_angular_speed
        }
    
    def emergency_stop(self):
        """紧急停止"""
        self.send_velocity_command(0, 0)
        print("紧急停止")

# 全局机器人控制器实例
robot = None

def get_robot_controller():
    """获取机器人控制器实例（单例模式）"""
    global robot
    if robot is None:
        robot = RobotController()
    return robot
#!/usr/bin/env python3
# robot_controller.py - 机器人控制器（直接串口控制）

import rospy
import math
import threading
import serial
import time
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
        
        # 电机方向修正（根据您的硬件实际情况设置）
        self.REVERSE_LEFT_MOTOR = True
        self.REVERSE_RIGHT_MOTOR = True
        
        # 串口初始化
        self.serial_port = rospy.get_param('~serial_port', '/dev/ttyUSB0')
        self.baud_rate = rospy.get_param('~baud_rate', 115200)
        self.ser = None
        
        try:
            self.ser = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=0.1
            )
            rospy.loginfo(f"成功连接到串口 {self.serial_port}")
            
            # 发送初始化命令（可选）
            self.send_serial_command("INIT\n")
            time.sleep(0.5)
            
        except Exception as e:
            rospy.logerr(f"无法打开串口 {self.serial_port}: {e}")
            rospy.logwarn("将在模拟模式下运行")
        
        # ROS发布器（保持原有接口）
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.vacuum_pub = rospy.Publisher('/vacuum_control', Bool, queue_size=10)
        
        # 控制状态
        self.current_speed = 0.0
        self.current_turn = 0.0
        self.vacuum_state = False
        
        # 当前PWM状态
        self.current_left_pwm = self.PWM_NEUTRAL
        self.current_right_pwm = self.PWM_NEUTRAL
        
        # 控制参数
        self.max_linear_speed = 0.5  # m/s
        self.max_angular_speed = 1.0  # rad/s
        
        print("机器人控制器初始化完成（直接串口控制）")
    
    def send_serial_command(self, command):
        """发送串口命令"""
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(command.encode())
                return True
            except Exception as e:
                rospy.logerr(f"发送串口命令失败: {e}")
                return False
        else:
            rospy.logwarn("串口未连接，模拟发送命令")
            print(f"[模拟] 发送命令: {command.strip()}")
            return False
    
    def pwm_to_velocity(self, pwm):
        """将PWM值转换为速度值（保持原有逻辑）"""
        if pwm == self.PWM_NEUTRAL:
            return 0.0
        elif pwm > self.PWM_NEUTRAL:
            normalized = (pwm - self.PWM_NEUTRAL) / (self.PWM_MAX - self.PWM_NEUTRAL)
            return normalized * self.max_linear_speed
        else:
            normalized = (self.PWM_NEUTRAL - pwm) / (self.PWM_NEUTRAL - self.PWM_MIN)
            return -normalized * self.max_linear_speed
    
    def velocity_to_pwm(self, velocity):
        """将速度值转换为PWM值（保持原有逻辑）"""
        if velocity == 0:
            return self.PWM_NEUTRAL
        elif velocity > 0:
            normalized = velocity / self.max_linear_speed
            pwm = self.PWM_NEUTRAL + normalized * (self.PWM_MAX - self.PWM_NEUTRAL)
            return int(max(self.PWM_NEUTRAL, min(self.PWM_MAX, pwm)))
        else:
            normalized = abs(velocity) / self.max_linear_speed
            pwm = self.PWM_NEUTRAL - normalized * (self.PWM_NEUTRAL - self.PWM_MIN)
            return int(max(self.PWM_MIN, min(self.PWM_NEUTRAL, pwm)))
    
    def send_velocity_command(self, linear_x, angular_z):
        """发送速度命令（改为直接串口输出）"""
        try:
            # 限制速度范围
            linear_x = max(-self.max_linear_speed, min(self.max_linear_speed, linear_x))
            angular_z = max(-self.max_angular_speed, min(self.max_angular_speed, angular_z))
            
            # 创建ROS消息（保持原有接口）
            twist = Twist()
            twist.linear.x = linear_x
            twist.angular.z = angular_z
            self.cmd_vel_pub.publish(twist)
            
            # 更新当前状态
            self.current_speed = linear_x
            self.current_turn = angular_z
            
            # 计算PWM值
            left_pwm, right_pwm = self.calculate_pwm(linear_x, angular_z)
            
            # 应用电机反转修正
            if self.REVERSE_LEFT_MOTOR:
                left_pwm = 2 * self.PWM_NEUTRAL - left_pwm
            if self.REVERSE_RIGHT_MOTOR:
                right_pwm = 2 * self.PWM_NEUTRAL - right_pwm
            
            # 确保在安全范围内
            left_pwm = max(self.PWM_MIN, min(self.PWM_MAX, int(left_pwm)))
            right_pwm = max(self.PWM_MIN, min(self.PWM_MAX, int(right_pwm)))
            
            # 发送串口命令 - 格式：M,left_pwm,right_pwm\n
            command = f"M,{left_pwm},{right_pwm}\n"
            success = self.send_serial_command(command)
            
            # 更新当前PWM状态
            self.current_left_pwm = left_pwm
            self.current_right_pwm = right_pwm
            
            # 打印调试信息
            print(f"速度命令: linear={linear_x:.2f}, angular={angular_z:.2f}, "
                  f"PWM: L={left_pwm}, R={right_pwm}")
            
            return success
            
        except Exception as e:
            print(f"发送速度命令失败: {e}")
            return False
    
    def calculate_pwm(self, linear_x, angular_z):
        """根据线速度和角速度计算左右轮PWM值（保持原有逻辑）"""
        # 基础速度转换
        base_pwm = self.velocity_to_pwm(linear_x)
        
        # 转向调整
        if angular_z == 0:
            left_pwm = base_pwm
            right_pwm = base_pwm
        elif angular_z > 0:
            turn_adjust = int(abs(angular_z) / self.max_angular_speed * self.pwm_turn_diff)
            left_pwm = base_pwm - turn_adjust
            right_pwm = base_pwm + turn_adjust
        else:
            turn_adjust = int(abs(angular_z) / self.max_angular_speed * self.pwm_turn_diff)
            left_pwm = base_pwm + turn_adjust
            right_pwm = base_pwm - turn_adjust
        
        # 确保在安全范围内
        left_pwm = max(self.PWM_MIN, min(self.PWM_MAX, left_pwm))
        right_pwm = max(self.PWM_MIN, min(self.PWM_MAX, right_pwm))
        
        return left_pwm, right_pwm
    
    def control_vacuum(self, state):
        """控制负压吸附（改为直接串口输出）"""
        try:
            # 创建ROS消息（保持原有接口）
            msg = Bool()
            msg.data = bool(state)
            self.vacuum_pub.publish(msg)
            
            # 发送串口命令
            if state:
                command = '!\n'  # 开启负压
            else:
                command = '%\n'  # 关闭负压
            
            success = self.send_serial_command(command)
            
            if success:
                self.vacuum_state = bool(state)
                print(f"负压吸附状态: {'开启' if state else '关闭'}")
            
            return success
            
        except Exception as e:
            print(f"控制负压吸附失败: {e}")
            return False
    
    def get_status(self):
        """获取当前状态（增强信息）"""
        return {
            'speed': self.current_speed,
            'turn': self.current_turn,
            'vacuum': self.vacuum_state,
            'left_pwm': self.current_left_pwm,
            'right_pwm': self.current_right_pwm,
            'max_linear_speed': self.max_linear_speed,
            'max_angular_speed': self.max_angular_speed,
            'serial_connected': self.ser is not None and self.ser.is_open
        }
    
    def emergency_stop(self):
        """紧急停止（改为直接串口输出）"""
        # 发送停止命令到串口
        command = f"M,{self.PWM_NEUTRAL},{self.PWM_NEUTRAL}\n"
        self.send_serial_command(command)
        
        # 更新状态
        self.current_speed = 0.0
        self.current_turn = 0.0
        self.current_left_pwm = self.PWM_NEUTRAL
        self.current_right_pwm = self.PWM_NEUTRAL
        
        # 发布ROS消息（保持原有接口）
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        
        print("紧急停止")
    
    def cleanup(self):
        """清理资源"""
        if self.ser and self.ser.is_open:
            self.emergency_stop()
            
            # 关闭负压吸附
            if self.vacuum_state:
                self.control_vacuum(False)
            
            try:
                self.ser.close()
                print("串口已关闭")
            except:
                pass

# 全局机器人控制器实例
robot = None

def get_robot_controller():
    """获取机器人控制器实例（单例模式）"""
    global robot
    if robot is None:
        robot = RobotController()
    return robot
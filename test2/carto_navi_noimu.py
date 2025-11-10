#!/usr/bin/env python3
import rospy
import math
import serial
import threading
import tf
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

class SimpleGoalNavigator:
    def __init__(self):
        rospy.init_node('simple_goal_navigator', anonymous=True)
        
        # 移除IMU相关初始化，保留串口用于控制
        # 串口初始化
        try:
            self.ser = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
            rospy.loginfo("成功连接到串口 /dev/ttyUSB1")
        except Exception as e:
            rospy.logerr(f"无法打开串口 /dev/ttyUSB1: {e}")
            return
        
        # 初始化负压吸附和速度
        self.initialize_car()
        
        # 状态变量
        self.current_pose = None
        self.initial_pose = None
        
        # 控制状态变量
        self.control_active = False
        self.target_reached = False
        
        # 控制参数
        self.position_tolerance = 0.08  # 8cm容差
        self.angle_tolerance = 0.15    # 约8.6度角度容差（稍微放宽）
        
        # 目标点导航相关变量
        self.target_point = None  # 目标点 [x, y]
        self.target_yaw = 0.0     # 目标朝向
        self.start_pose = None    # 导航开始时的位置
        self.initial_target_angle = 0.0  # 固定的初始目标角度
        self.target_distance = 0.0  # 需要前进的总距离
        self.turned_to_target = False  # 是否已经转向目标方向
        
        # TF监听器
        self.tf_listener = tf.TransformListener()
        
        # 订阅/move_base_simple/goal话题
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        
        # 高频控制循环 (50Hz)
        self.control_timer = rospy.Timer(rospy.Duration(0.02), self.control_loop)
        
        rospy.loginfo("简单目标导航器初始化完成 - 使用TF角度")
        rospy.loginfo("等待Foxglove发送目标点...")
        
    def goal_callback(self, msg):
        """处理Foxglove发送的目标点"""
        if self.current_pose is None:
            rospy.logwarn("无法开始导航: 等待定位数据")
            return
            
        self.stop_current_mission()
        rospy.sleep(0.5)
        
        # 提取目标位置
        target_x = msg.pose.position.x
        target_y = msg.pose.position.y
        
        # 提取目标朝向（从四元数转换为欧拉角）
        orientation = msg.pose.orientation
        _, _, self.target_yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        
        self.target_point = [target_x, target_y]
        self.start_pose = self.current_pose
        self.turned_to_target = False
        
        # 计算初始目标角度并固定它！
        dx = target_x - self.current_pose.position.x
        dy = target_y - self.current_pose.position.y
        self.initial_target_angle = math.atan2(dy, dx)  # 固定初始角度
        self.target_distance = math.sqrt(dx**2 + dy**2)
        
        rospy.loginfo(f"收到新目标点: ({target_x:.2f}, {target_y:.2f})")
        rospy.loginfo(f"目标朝向: {math.degrees(self.target_yaw):.1f}°")
        rospy.loginfo(f"目标距离: {self.target_distance:.2f}m")
        rospy.loginfo(f"固定目标角度: {math.degrees(self.initial_target_angle):.2f}°")
        
        self.control_active = True
    
    def get_current_yaw_from_tf(self):
        """从TF获取当前偏航角 - 替代磁力计"""
        try:
            (trans, rot) = self.tf_listener.lookupTransform('map', 'base_link', rospy.Time(0))
            # 从四元数提取偏航角
            _, _, yaw = euler_from_quaternion(rot)
            return yaw
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("获取TF角度失败")
            return 0.0
    
    def calculate_angle_error(self, target_yaw):
        """计算角度误差 - 使用TF数据"""
        current_yaw = self.get_current_yaw_from_tf()
        error = target_yaw - current_yaw
        
        # 将角度误差规范化到[-pi, pi]
        while error > math.pi:
            error -= 2 * math.pi
        while error < -math.pi:
            error += 2 * math.pi
        
        return error
    
    def turn_to_target(self):
        """转向目标点方向 - 使用固定的初始目标角度和TF角度"""
        # 使用固定的初始目标角度，而不是实时计算
        target_angle = self.initial_target_angle
        current_yaw = self.get_current_yaw_from_tf()
        angle_error = self.calculate_angle_error(target_angle)
        
        # 详细的调试信息
        rospy.loginfo(f"转向调试:")
        rospy.loginfo(f"  当前位姿: ({self.current_pose.position.x:.2f}, {self.current_pose.position.y:.2f})")
        rospy.loginfo(f"  目标位姿: ({self.target_point[0]:.2f}, {self.target_point[1]:.2f})")
        rospy.loginfo(f"  TF当前角度: {math.degrees(current_yaw):.2f}°")
        rospy.loginfo(f"  固定目标角度: {math.degrees(target_angle):.2f}°")
        rospy.loginfo(f"  角度误差: {math.degrees(angle_error):.2f}°")
        
        # 检查是否对准目标
        if abs(angle_error) < self.angle_tolerance:
            if not self.target_reached:
                self.stop_car()
                self.target_reached = True
                self.turned_to_target = True
                rospy.loginfo("✓ 已对准目标方向，开始前进")
                rospy.sleep(1.0)
            return
        
        # 更智能的转向逻辑
        self.target_reached = False
        turn_threshold = 0.2  # 约11.5度，大于这个角度才转向
        
        # 选择更优的转向方向
        if angle_error > turn_threshold:
            self.send_control_command('D')  # 左转
            rospy.loginfo("  动作: 左转")
        elif angle_error < -turn_threshold:
            self.send_control_command('C')  # 右转
            rospy.loginfo("  动作: 右转")
        else:
            self.stop_car()
            rospy.loginfo("  动作: 停止 - 接近目标方向")
    
    # 保留其他方法...
    def get_robot_pose(self):
        """通过TF获取机器人在map坐标系中的位置"""
        try:
            (trans, rot) = self.tf_listener.lookupTransform('map', 'base_link', rospy.Time(0))
            pose = PoseStamped().pose
            pose.position.x = trans[0]
            pose.position.y = trans[1]
            pose.position.z = trans[2]
            pose.orientation.x = rot[0]
            pose.orientation.y = rot[1]
            pose.orientation.z = rot[2]
            pose.orientation.w = rot[3]
            return pose
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None
    
    def stop_current_mission(self):
        """停止当前任务"""
        self.control_active = False
        self.target_reached = False
        self.turned_to_target = False
        self.stop_car()
        rospy.loginfo("当前任务已停止")
    
    def send_control_command(self, command):
        """发送控制命令"""
        try:
            self.ser.write(command.encode())
        except Exception as e:
            rospy.logerr(f"发送命令失败: {e}")
    
    def stop_car(self):
        """停止小车"""
        self.send_control_command('0')
    
    def initialize_car(self):
        """初始化小车：开启负压吸附并设置速度"""
        for i in range(9):
            self.ser.write('3'.encode())
            rospy.sleep(0.1)
        for i in range(6):
            self.ser.write('6'.encode())
            rospy.sleep(0.1)
        self.ser.write('!'.encode())
        rospy.loginfo("负压吸附已开启，速度已降低")
        rospy.sleep(1)
    
    def control_loop(self, event):
        """高频控制循环"""
        self.current_pose = self.get_robot_pose()
        
        if not self.control_active or self.current_pose is None:
            return
        
        self.point_navigation_control()
    
    def point_navigation_control(self):
        """目标点导航控制"""
        if self.target_point is None:
            return
        
        if not self.turned_to_target:
            self.turn_to_target()
        else:
            self.move_to_target()
    
    def move_to_target(self):
        """前进到目标点"""
        traveled_distance = self.calculate_traveled_distance()
        remaining_distance = self.target_distance - traveled_distance
        
        rospy.loginfo_throttle(1, f"前往目标: 已前进={traveled_distance:.3f}m, 剩余={remaining_distance:.3f}m, 总距离={self.target_distance:.3f}m")
        
        if remaining_distance <= self.position_tolerance:
            if not self.target_reached:
                self.stop_car()
                self.target_reached = True
                self.control_active = False
                rospy.loginfo(f"✓ 已到达目标点: ({self.target_point[0]:.2f}, {self.target_point[1]:.2f})")
            return
        
        self.target_reached = False
        self.send_control_command('A')
    
    def calculate_traveled_distance(self):
        """计算从开始位置走过的距离"""
        if self.start_pose is None or self.current_pose is None:
            return 0.0
        dx = self.current_pose.position.x - self.start_pose.position.x
        dy = self.current_pose.position.y - self.start_pose.position.y
        return math.sqrt(dx**2 + dy**2)
    
    def run(self):
        rospy.spin()
        self.cleanup()

    def cleanup(self):
        self.control_active = False
        self.stop_car()
        try:
            self.control_timer.shutdown()
            self.ser.write('0'.encode())
            self.ser.close()
        except:
            pass

if __name__ == '__main__':
    try:
        navigator = SimpleGoalNavigator()
        navigator.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"程序异常: {e}")
    finally:
        rospy.loginfo("程序结束")
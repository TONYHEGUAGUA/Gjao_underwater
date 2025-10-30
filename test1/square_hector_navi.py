#!/usr/bin/env python
import rospy
import math
import serial
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

class CarNavigation:
    def __init__(self):
        rospy.init_node('car_navigation', anonymous=True)
        
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
        self.initial_yaw = 0.0
        self.mission_state = 0  # 0:等待开始, 1:前进1米, 2:右转90度, 3:前进1米, 4:右转90度, 5:前进1米, 6:右转90度, 7:前进1米, 8:完成
        
        # 每个阶段的起始位置和角度
        self.phase_start_pose = None
        self.phase_start_yaw = 0.0
        
        # 控制参数
        self.position_tolerance = 0.05  # 5cm位置容差
        self.angle_tolerance = 0.05    # 约6度角度容差
        self.square_side_length = 0.3  # 正方形边长1米
        
        # 订阅slam_out_pose话题
        rospy.Subscriber('/slam_out_pose', PoseStamped, self.pose_callback)
        
        rospy.loginfo("导航节点初始化完成，等待定位数据...")
        
    def initialize_car(self):
        """初始化小车：开启负压吸附并设置速度"""
        # 发送9个3减速到10
        for i in range(9):
            self.ser.write('3'.encode())
            rospy.sleep(0.1)
        for i in range(6):
            self.ser.write('6'.encode())
            rospy.sleep(0.1)
        
        # 开启负压吸附
        self.ser.write('!'.encode())
        rospy.loginfo("负压吸附已开启，速度已降低")
        rospy.sleep(1)
    
    def pose_callback(self, msg):
        """处理定位数据"""
        self.current_pose = msg.pose
        
        if self.initial_pose is None:
            self.initial_pose = msg.pose
            self.initial_yaw = self.get_yaw_from_pose(self.initial_pose)
            rospy.loginfo(f"收到初始位姿，位置: ({self.initial_pose.position.x:.3f}, {self.initial_pose.position.y:.3f})")
            rospy.loginfo(f"初始偏航角: {math.degrees(self.initial_yaw):.2f}° (作为正北)")
            
            # 设置第一阶段起始点
            self.phase_start_pose = self.initial_pose
            self.phase_start_yaw = self.initial_yaw
            
            self.mission_state = 1
            return
        
        # 执行导航状态机
        self.navigation_controller()
    
    def get_yaw_from_pose(self, pose):
        """从四元数中提取偏航角"""
        orientation = pose.orientation
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = euler_from_quaternion(quaternion)
        return yaw
    
    def calculate_distance_from_start(self):
        """计算从阶段起始点开始的直线距离"""
        if self.phase_start_pose is None or self.current_pose is None:
            return 0.0
            
        dx = self.current_pose.position.x - self.phase_start_pose.position.x
        dy = self.current_pose.position.y - self.phase_start_pose.position.y
        return math.sqrt(dx**2 + dy**2)
    
    def calculate_angle_error(self, target_yaw):
        """计算角度误差"""
        current_yaw = self.get_yaw_from_pose(self.current_pose)
        error = target_yaw - current_yaw
        
        # 将角度误差规范化到[-pi, pi]
        while error > math.pi:
            error -= 2 * math.pi
        while error < -math.pi:
            error += 2 * math.pi
            
        return error
    
    def send_control_command(self, command):
        """发送控制命令"""
        try:
            self.ser.write(command.encode())
            rospy.sleep(0.1)
            self.ser.write('!'.encode())
            rospy.logdebug(f"发送命令: {command}")
        except Exception as e:
            rospy.logerr(f"发送命令失败: {e}")
    
    def stop_car(self):
        """停止小车"""
        self.send_control_command('0')
        rospy.sleep(0.1)
    
    def navigation_controller(self):
        """导航主控制器"""
        if self.mission_state == 0 or self.current_pose is None:
            return
        
        if self.mission_state in [1, 3, 5, 7]:
            # 前进阶段 (1, 3, 5, 7)
            self.move_straight()
        elif self.mission_state in [2, 4, 6]:
            # 转弯阶段 (2, 4, 6)
            self.turn_right_90()
        elif self.mission_state == 8:
            # 任务完成
            self.stop_car()
            rospy.loginfo("正方形导航任务完成!")
            return
    
    def move_straight(self):
        """直线前进1米"""
        distance = self.calculate_distance_from_start()
        
        # 打印前进信息
        current_yaw = self.get_yaw_from_pose(self.current_pose)
        rospy.loginfo_throttle(1, f"阶段{self.mission_state}-前进: 已前进距离: {distance:.3f}m, 当前角度: {math.degrees(current_yaw):.2f}°")
        
        # 检查是否达到目标距离
        if distance >= self.square_side_length:
            self.stop_car()
            rospy.loginfo(f"完成第{(self.mission_state+1)//2}条边的前进")
            
            # 记录下一阶段的起始位置
            self.phase_start_pose = self.current_pose
            self.phase_start_yaw = self.get_yaw_from_pose(self.current_pose)
            
            self.mission_state += 1
            rospy.sleep(1)  # 短暂停顿
            return
        
        # 直线前进，不进行角度校正
        self.send_control_command('A')  # 使用A命令前进
        rospy.loginfo_throttle(1, "直线前进中...")
    
    def turn_right_90(self):
        """右转90度"""
        # 计算目标角度：相对于初始角度依次转90, 180, 270度
        target_yaw = self.initial_yaw - (self.mission_state // 2) * math.pi / 2
        
        angle_error = self.calculate_angle_error(target_yaw)
        
        current_yaw = self.get_yaw_from_pose(self.current_pose)
        rospy.loginfo_throttle(1, f"阶段{self.mission_state}-右转: 当前角度: {math.degrees(current_yaw):.2f}°, 目标角度: {math.degrees(target_yaw):.2f}°")
        rospy.loginfo_throttle(1, f"角度误差: {math.degrees(angle_error):.2f}°")
        
        if abs(angle_error) < self.angle_tolerance:
            self.stop_car()
            rospy.loginfo(f"完成第{self.mission_state//2}次90度右转")
            
            # 记录下一阶段的起始位置
            self.phase_start_pose = self.current_pose
            self.phase_start_yaw = self.get_yaw_from_pose(self.current_pose)
            
            self.mission_state += 1
            rospy.sleep(1)
            return
        
        # 执行右转
        self.send_control_command('C')
        rospy.loginfo_throttle(1, "右转中...")
    
    def run(self):
        """主循环"""
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            rate.sleep()
        
        # 程序退出时停止小车
        self.stop_car()
        self.ser.write('0'.encode())
        self.ser.close()

if __name__ == '__main__':
    try:
        navigator = CarNavigation()
        navigator.run()
    except rospy.ROSInterruptException:
        pass
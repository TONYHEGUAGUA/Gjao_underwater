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
        self.mission_state = 0  # 0:等待开始, 1:前往前方1米, 2:右转, 3:前往右侧1米, 4:完成
        
        # 控制参数
        self.position_tolerance = 0.05  # 5cm位置容差
        self.angle_tolerance = 0.1    # 约6度角度容差
        
        # 订阅slam_out_pose话题
        rospy.Subscriber('/slam_out_pose', PoseStamped, self.pose_callback)
        
        rospy.loginfo("导航节点初始化完成，等待定位数据...")
        
    def initialize_car(self):
        """初始化小车：开启负压吸附并设置速度"""
        # 发送9个3减速到10
        for i in range(3):
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
            initial_yaw = self.get_yaw_from_pose(self.initial_pose)
            rospy.loginfo(f"收到初始位姿，位置: ({self.initial_pose.position.x:.3f}, {self.initial_pose.position.y:.3f})")
            rospy.loginfo(f"初始偏航角: {math.degrees(initial_yaw):.2f}°")
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
    
    def calculate_position_error(self, target_x, target_y):
        """计算位置误差"""
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        
        error_x = target_x - current_x
        error_y = target_y - current_y
        distance_error = math.sqrt(error_x**2 + error_y**2)
        
        return distance_error, error_x, error_y
    
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
            rospy.logdebug(f"发送命令: {command}")
        except Exception as e:
            rospy.logerr(f"发送命令失败: {e}")
    
    def stop_car(self):
        """停止小车"""
        self.send_control_command('0')
        rospy.sleep(0.1)
    
    def send_move_toward_target(self, target_x, target_y):
        """根据目标位置智能选择前进或后退"""
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        
        # 计算到目标点的方向向量
        dx = target_x - current_x
        dy = target_y - current_y
        
        # 计算当前朝向
        current_yaw = self.get_yaw_from_pose(self.current_pose)
        
        # 计算目标方向的角度
        target_angle = math.atan2(dy, dx)
        
        # 计算角度误差
        angle_error = target_angle - current_yaw
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi
        
        # 实时打印位置信息
        rospy.loginfo(f"当前位置: ({current_x:.3f}, {current_y:.3f}), 目标位置: ({target_x:.3f}, {target_y:.3f})")
        rospy.loginfo(f"位置差值: X={dx:.3f}, Y={dy:.3f}, 距离={math.sqrt(dx**2+dy**2):.3f}")
        rospy.loginfo(f"目标方向: {math.degrees(target_angle):.2f}°, 当前朝向: {math.degrees(current_yaw):.2f}°, 角度误差: {math.degrees(angle_error):.2f}°")
        
        # 如果角度误差太大，先修正角度
        if abs(angle_error) > self.angle_tolerance:
            if angle_error > 0:
                self.send_control_command('D')  # 左转
                rospy.loginfo("角度修正: 左转")
            else:
                self.send_control_command('C')  # 右转
                rospy.loginfo("角度修正: 右转")
            return
        
        # 角度正确，判断应该前进还是后退
        # 由于小车的实际前进方向与坐标系X正方向相反，我们需要反转前进/后退逻辑
        if abs(angle_error) < math.pi/2:
            self.send_control_command('A')  # 使用A命令（原本是后退）来向目标移动
            rospy.loginfo("向目标移动（使用A命令）")
        else:
            self.send_control_command('B')  # 使用B命令（原本是前进）来向目标移动
            rospy.loginfo("向目标移动（使用B命令）")
    
    def navigation_controller(self):
        """导航主控制器"""
        if self.mission_state == 0 or self.current_pose is None:
            return
        
        if self.mission_state == 1:
            # 阶段1: 向前移动1米 (X轴正方向)
            self.move_forward_1m()
        elif self.mission_state == 2:
            # 阶段2: 右转90度
            self.turn_right_90()
        elif self.mission_state == 3:
            # 阶段3: 向右移动1米 (Y轴负方向)
            self.move_right_1m()
        elif self.mission_state == 4:
            # 任务完成
            self.stop_car()
            rospy.loginfo("导航任务完成!")
            return
    
    def move_forward_1m(self):
        """向前移动1米 (X轴正方向)"""
        target_x = self.initial_pose.position.x + 1.0  # X+方向前进1米
        target_y = self.initial_pose.position.y
        
        distance_error, error_x, error_y = self.calculate_position_error(target_x, target_y)
        
        # 打印详细的X位置信息
        current_x = self.current_pose.position.x
        rospy.loginfo_throttle(1, f"阶段1-前进: 当前X: {current_x:.3f}m, 目标X: {target_x:.3f}m, X差值: {error_x:.3f}m, 距离误差: {distance_error:.3f}m")
        
        # 位置控制
        if distance_error < self.position_tolerance:
            self.stop_car()
            rospy.loginfo("到达前方1米目标点")
            self.mission_state = 2
            rospy.sleep(1)  # 短暂停顿
            return
        
        # 使用智能移动控制
        self.send_move_toward_target(target_x, target_y)
    
    def turn_right_90(self):
        """右转90度"""
        initial_yaw = self.get_yaw_from_pose(self.initial_pose)
        target_yaw = initial_yaw - math.pi/2  # 右转90度
        
        angle_error = self.calculate_angle_error(target_yaw)
        
        rospy.loginfo_throttle(1, f"阶段2-右转: 角度误差: {math.degrees(angle_error):.2f}°")
        
        if abs(angle_error) < self.angle_tolerance:
            self.stop_car()
            rospy.loginfo("完成90度右转")
            self.mission_state = 3
            rospy.sleep(1)
            return
        
        # 执行右转
        self.send_control_command('C')
        rospy.loginfo_throttle(1, "右转中...")
    
    def move_right_1m(self):
        """向右移动1米 (Y轴负方向)"""
        target_x = self.initial_pose.position.x + 1.0
        target_y = self.initial_pose.position.y - 1.0
        
        distance_error, error_x, error_y = self.calculate_position_error(target_x, target_y)
        
        # 打印详细的Y位置信息
        current_y = self.current_pose.position.y
        rospy.loginfo_throttle(1, f"阶段3-右移: 当前Y: {current_y:.3f}m, 目标Y: {target_y:.3f}m, Y差值: {error_y:.3f}m, 距离误差: {distance_error:.3f}m")
        
        # 位置控制
        if distance_error < self.position_tolerance:
            self.stop_car()
            rospy.loginfo("到达右侧1米目标点")
            self.mission_state = 4
            return
        
        # 使用智能移动控制
        self.send_move_toward_target(target_x, target_y)
    
    def run(self):
        """主循环"""
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            rate.sleep()
        
        # 程序退出时停止小车
        self.stop_car()
        self.ser.close()

if __name__ == '__main__':
    try:
        navigator = CarNavigation()
        navigator.run()
    except rospy.ROSInterruptException:
        pass
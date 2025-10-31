#!/usr/bin/env python
import rospy
import math
import serial
import threading
from geometry_msgs.msg import PoseStamped

class CarNavigation:
    def __init__(self):
        rospy.init_node('car_navigation', anonymous=True)
        
        # IMU数据相关变量
        self.current_imu_yaw = 0.0
        self.last_imu_time = 0
        self.imu_initialized = False
        self.imu_initial_yaw = 0.0
        
        # 串口初始化 - 同时用于IMU读取和控制命令
        try:
            self.ser = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
            rospy.loginfo("成功连接到串口 /dev/ttyUSB1")
            
            # 启动IMU读取线程
            self.imu_running = True
            self.imu_thread = threading.Thread(target=self.read_imu_data)
            self.imu_thread.daemon = True
            self.imu_thread.start()
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
        
        # 控制参数 - 调整容差和边长
        self.position_tolerance = 0.08  # 增大到8cm容差
        self.angle_tolerance = 0.08    # 约9度角度容差
        self.square_side_length = 0.3  # 正方形边长0.3米
        
        # 订阅slam_out_pose话题（仅用于位置信息）
        rospy.Subscriber('/slam_out_pose', PoseStamped, self.pose_callback)
        
        rospy.loginfo("导航节点初始化完成，等待定位数据和IMU数据...")
        
    def calculate_checksum(self, data):
        """计算XOR校验和"""
        checksum = 0
        for byte in data.encode('ascii'):
            checksum ^= byte
        return checksum
    
    def verify_checksum(self, payload, received_checksum):
        """验证校验和"""
        try:
            calculated_checksum = self.calculate_checksum(payload)
            return calculated_checksum == int(received_checksum, 16)
        except:
            return False
    
    def parse_imu_frame(self, frame):
        """解析IMU数据帧"""
        try:
            # 移除首尾空白字符
            frame = frame.strip()
            
            # 检查帧格式
            if not frame.startswith('$IMU,'):
                return None
                
            # 分割字段
            parts = frame[1:].split(',')  # 去掉$号后分割
            if len(parts) != 3:
                return None
                
            frame_type, yaw_str, checksum_str = parts
            
            # 验证帧类型
            if frame_type != 'IMU':
                return None
                
            # 验证校验和
            payload = f"IMU,{yaw_str}"
            if not self.verify_checksum(payload, checksum_str):
                rospy.logwarn(f"IMU校验和错误: 计算值={self.calculate_checksum(payload):02X}, 接收值={checksum_str}")
                return None
            
            # 解析yaw值
            yaw = float(yaw_str)
            
            return {
                'type': 'IMU',
                'yaw': math.radians(yaw),  # 转换为弧度
                'checksum': checksum_str,
                'timestamp': rospy.get_time()
            }
            
        except ValueError as e:
            rospy.logwarn(f"IMU数据解析错误: {e}, 帧: {frame}")
            return None
        except Exception as e:
            rospy.logwarn(f"IMU解析异常: {e}, 帧: {frame}")
            return None
    
    def read_imu_data(self):
        """读取IMU串口数据"""
        buffer = ""
        while self.imu_running and not rospy.is_shutdown():
            try:
                if self.ser.in_waiting > 0:
                    # 读取所有可用数据
                    data = self.ser.read(self.ser.in_waiting).decode('ascii', errors='ignore')
                    buffer += data
                    
                    # 处理完整帧
                    while '\r\n' in buffer:
                        frame_end = buffer.find('\r\n')
                        frame = buffer[:frame_end]
                        buffer = buffer[frame_end + 2:]
                        
                        # 解析帧
                        if frame:
                            imu_data = self.parse_imu_frame(frame)
                            if imu_data:
                                self.current_imu_yaw = imu_data['yaw']
                                self.last_imu_time = rospy.get_time()
                                
                                # 如果是第一次收到IMU数据，设置初始偏航角
                                if not self.imu_initialized:
                                    self.imu_initial_yaw = self.current_imu_yaw
                                    self.imu_initialized = True
                                    rospy.loginfo(f"IMU初始化完成，初始偏航角: {math.degrees(self.imu_initial_yaw):.2f}°")
                                
                                rospy.logdebug(f"IMU Yaw: {math.degrees(self.current_imu_yaw):.2f}°")
                
                # 短暂休眠以减少CPU占用
                rospy.sleep(0.001)
                
            except Exception as e:
                rospy.logwarn(f"读取IMU数据错误: {e}")
                rospy.sleep(0.1)
    
    def get_current_yaw(self):
        """获取当前偏航角 - 仅使用IMU数据"""
        if self.imu_initialized:
            return self.current_imu_yaw
        else:
            rospy.logwarn("IMU数据未初始化，返回0角度")
            return 0.0
    
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
        """处理定位数据（仅位置信息）"""
        self.current_pose = msg.pose
        
        if self.initial_pose is None:
            self.initial_pose = msg.pose
            # 等待IMU数据初始化
            if not self.imu_initialized:
                rospy.loginfo("等待IMU数据初始化...")
                return
                
            self.initial_yaw = self.imu_initial_yaw
            rospy.loginfo(f"收到初始位姿，位置: ({self.initial_pose.position.x:.3f}, {self.initial_pose.position.y:.3f})")
            rospy.loginfo(f"初始偏航角: {math.degrees(self.initial_yaw):.2f}°")
            
            # 设置第一阶段起始点
            self.phase_start_pose = self.initial_pose
            self.phase_start_yaw = self.initial_yaw
            
            self.mission_state = 1
            rospy.loginfo("开始执行正方形路径")
            return
        
        # 执行导航状态机
        self.navigation_controller()
    
    def calculate_distance_from_start(self):
        """计算从阶段起始点开始的直线距离"""
        if self.phase_start_pose is None or self.current_pose is None:
            return 0.0
            
        dx = self.current_pose.position.x - self.phase_start_pose.position.x
        dy = self.current_pose.position.y - self.phase_start_pose.position.y
        return math.sqrt(dx**2 + dy**2)
    
    def calculate_angle_error(self, target_yaw):
        """计算角度误差 - 使用IMU数据"""
        current_yaw = self.get_current_yaw()
        error = target_yaw - current_yaw
        
        # 将角度误差规范化到[-pi, pi]
        while error > math.pi:
            error -= 2 * math.pi
        while error < -math.pi:
            error += 2 * math.pi
        
        rospy.logdebug(f"角度误差: {math.degrees(error):.2f}°")
            
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
        rospy.loginfo("停止小车")
        rospy.sleep(0.5)  # 增加停止后的稳定时间
    
    def navigation_controller(self):
        """导航主控制器"""
        if self.mission_state == 0 or self.current_pose is None or not self.imu_initialized:
            return
        
        if self.mission_state in [1, 3, 5, 7]:
            # 前进阶段
            self.move_straight()
        elif self.mission_state in [2, 4, 6]:
            # 转弯阶段
            self.turn_right_90()
        elif self.mission_state == 8:
            # 任务完成
            self.stop_car()
            rospy.loginfo("正方形导航任务完成!")
            rospy.signal_shutdown("任务完成")
    
    def move_straight(self):
        """直线前进"""
        distance = self.calculate_distance_from_start()
        
        current_yaw = self.get_current_yaw()
        rospy.loginfo_throttle(2, f"阶段{self.mission_state}-前进: 距离={distance:.3f}m, 目标={self.square_side_length}m, 角度={math.degrees(current_yaw):.2f}°")
        
        # 检查是否达到目标距离
        if distance >= self.square_side_length - self.position_tolerance:
            self.stop_car()
            rospy.loginfo(f"✓ 完成第{(self.mission_state+1)//2}条边的前进: {distance:.3f}m")
            
            # 记录下一阶段的起始位置
            self.phase_start_pose = self.current_pose
            self.phase_start_yaw = self.get_current_yaw()
            
            self.mission_state += 1
            rospy.sleep(1.5)  # 增加阶段间停顿
            return
        
        # 直线前进
        self.send_control_command('A')
    
    def turn_right_90(self):
        """右转90度 - 使用IMU数据，连续转弯"""
        # 目标角度计算
        turn_count = (self.mission_state // 2)  # 第1,2,3次转弯
        target_yaw = self.initial_yaw - turn_count * (math.pi / 2)
    
        angle_error = self.calculate_angle_error(target_yaw)
    
        current_yaw = self.get_current_yaw()
        rospy.loginfo_throttle(1, f"阶段{self.mission_state}-右转: 当前={math.degrees(current_yaw):.2f}°, 目标={math.degrees(target_yaw):.2f}°, 误差={math.degrees(angle_error):.2f}°")
    
        # 检查是否达到目标角度
        if abs(angle_error) < self.angle_tolerance:
            self.stop_car()
            rospy.loginfo(f"✓ 完成第{turn_count}次90度右转")
        
            # 记录下一阶段的起始位置
            self.phase_start_pose = self.current_pose
            self.phase_start_yaw = self.get_current_yaw()
        
            self.mission_state += 1
            rospy.sleep(1.5)  # 增加阶段间停顿
            return
    
        # 持续右转，直到达到目标角度
        self.send_control_command('C')  # 持续右转
    
    def run(self):
        """主循环"""
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            rate.sleep()
        
        # 程序退出时停止小车
        self.imu_running = False
        self.stop_car()
        try:
            self.ser.write('0'.encode())
            self.ser.close()
        except:
            pass

if __name__ == '__main__':
    try:
        navigator = CarNavigation()
        navigator.run()
    except rospy.ROSInterruptException:
        pass
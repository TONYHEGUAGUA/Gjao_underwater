#!/usr/bin/env python
import rospy
import math
import serial
import threading
import sys
import select
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
        
        # 控制状态变量
        self.control_active = False
        self.target_reached = False
        
        # 控制参数
        self.position_tolerance = 0.08  # 8cm容差
        self.angle_tolerance = 0.08    # 约9度角度容差
        
        # 目标点导航相关变量
        self.target_point = None  # 目标点 [x, y]
        self.start_pose = None    # 导航开始时的位置
        self.target_distance = 0.0  # 需要前进的总距离
        self.turned_to_target = False  # 是否已经转向目标方向
        
        # 订阅slam_out_pose话题（仅用于位置信息）
        rospy.Subscriber('/slam_out_pose', PoseStamped, self.pose_callback)
        
        # 高频控制循环 (50Hz)
        self.control_timer = rospy.Timer(rospy.Duration(0.02), self.control_loop)
        
        # 启动命令行输入监听
        self.input_thread = threading.Thread(target=self.listen_input)
        self.input_thread.daemon = True
        self.input_thread.start()
        
        rospy.loginfo("导航节点初始化完成，等待定位数据和IMU数据...")
        rospy.loginfo("输入格式: 'x y' 前往目标点 (例如: 2 2)")
        rospy.loginfo("输入 'stop' 停止当前任务")
        
    def listen_input(self):
        """监听命令行输入"""
        while not rospy.is_shutdown():
            try:
                # 检查是否有输入可用
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    user_input = sys.stdin.readline().strip()
                    self.process_user_input(user_input)
            except Exception as e:
                rospy.logwarn(f"输入处理错误: {e}")
    
    def process_user_input(self, user_input):
        """处理用户输入"""
        if not user_input:
            return
            
        if user_input.lower() == 'stop':
            self.stop_current_mission()
        else:
            # 尝试解析为目标点坐标
            try:
                parts = user_input.split()
                if len(parts) == 2:
                    x = float(parts[0])
                    y = float(parts[1])
                    self.start_point_navigation(x, y)
                else:
                    rospy.logwarn("输入格式错误，请使用: x y (例如: 2 2)")
            except ValueError:
                rospy.logwarn("坐标格式错误，请输入数字 (例如: 2 2)")
    
    def start_point_navigation(self, target_x, target_y):
        """开始目标点导航"""
        if self.current_pose is None or not self.imu_initialized:
            rospy.logwarn("无法开始导航: 等待定位和IMU数据初始化")
            return
            
        self.stop_current_mission()
        rospy.sleep(0.5)
        
        self.target_point = [target_x, target_y]
        self.start_pose = self.current_pose
        self.turned_to_target = False
        
        # 计算目标距离和方向
        dx = target_x - self.current_pose.position.x
        dy = target_y - self.current_pose.position.y
        self.target_distance = math.sqrt(dx**2 + dy**2)
        target_angle = math.atan2(dy, dx)
        
        rospy.loginfo(f"开始导航到目标点: ({target_x:.2f}, {target_y:.2f})")
        rospy.loginfo(f"目标距离: {self.target_distance:.2f}m, 目标方向: {math.degrees(target_angle):.1f}°")
        rospy.loginfo(f"当前位置: ({self.current_pose.position.x:.2f}, {self.current_pose.position.y:.2f})")
        
        self.control_active = True
    
    def stop_current_mission(self):
        """停止当前任务"""
        self.control_active = False
        self.target_reached = False
        self.turned_to_target = False
        self.stop_car()
        rospy.loginfo("当前任务已停止")
    
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
        """处理定位数据（仅位置信息）- 不执行控制逻辑"""
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
    
    def calculate_traveled_distance(self):
        """计算从开始位置走过的距离"""
        if self.start_pose is None or self.current_pose is None:
            return 0.0
            
        dx = self.current_pose.position.x - self.start_pose.position.x
        dy = self.current_pose.position.y - self.start_pose.position.y
        return math.sqrt(dx**2 + dy**2)
    
    def calculate_target_angle(self):
        """计算到目标点的角度"""
        if self.target_point is None or self.current_pose is None:
            return 0.0
            
        dx = self.target_point[0] - self.current_pose.position.x
        dy = self.target_point[1] - self.current_pose.position.y
        return math.atan2(dy, dx)
    
    def calculate_angle_error(self, target_yaw):
        """计算角度误差 - 使用IMU数据"""
        current_yaw = self.get_current_yaw()
        error = target_yaw - current_yaw
        
        # 将角度误差规范化到[-pi, pi]
        while error > math.pi:
            error -= 2 * math.pi
        while error < -math.pi:
            error += 2 * math.pi
        
        return error
    
    def send_control_command(self, command):
        """发送控制命令 - 移除不必要的休眠"""
        try:
            self.ser.write(command.encode())
            rospy.logdebug(f"发送命令: {command}")
        except Exception as e:
            rospy.logerr(f"发送命令失败: {e}")
    
    def stop_car(self):
        """停止小车"""
        self.send_control_command('0')
        rospy.loginfo("停止小车")
    
    def control_loop(self, event):
        """高频控制循环 (50Hz)"""
        if not self.control_active:
            return
            
        if self.current_pose is None or not self.imu_initialized:
            return
        
        # 目标点导航控制
        self.point_navigation_control()
    
    def point_navigation_control(self):
        """目标点导航控制"""
        if self.target_point is None:
            return
        
        # 阶段1: 转向目标方向
        if not self.turned_to_target:
            self.turn_to_target()
        # 阶段2: 前进到目标点
        else:
            self.move_to_target()
    
    def turn_to_target(self):
        """转向目标点方向"""
        target_angle = self.calculate_target_angle()
        angle_error = self.calculate_angle_error(target_angle)
        
        current_yaw = self.get_current_yaw()
        rospy.loginfo_throttle(1, f"转向目标: 当前角度={math.degrees(current_yaw):.2f}°, 目标角度={math.degrees(target_angle):.2f}°, 误差={math.degrees(angle_error):.2f}°")
        
        # 检查是否对准目标
        if abs(angle_error) < self.angle_tolerance:
            if not self.target_reached:
                self.stop_car()
                self.target_reached = True
                self.turned_to_target = True
                rospy.loginfo("✓ 已对准目标方向，开始前进")
                rospy.sleep(1.0)  # 短暂停顿后开始前进
            return
        
        # 根据角度误差选择转向方向
        self.target_reached = False
        if angle_error > 0:
            self.send_control_command('D')  # 左转
        else:
            self.send_control_command('C')  # 右转
    
    def move_to_target(self):
        """前进到目标点"""
        traveled_distance = self.calculate_traveled_distance()
        remaining_distance = self.target_distance - traveled_distance
        
        current_yaw = self.get_current_yaw()
        rospy.loginfo_throttle(1, f"前往目标: 已前进={traveled_distance:.3f}m, 剩余={remaining_distance:.3f}m, 总距离={self.target_distance:.3f}m")
        
        # 检查是否到达目标
        if remaining_distance <= self.position_tolerance:
            if not self.target_reached:
                self.stop_car()
                self.target_reached = True
                self.control_active = False
                rospy.loginfo(f"✓ 已到达目标点: ({self.target_point[0]:.2f}, {self.target_point[1]:.2f})")
                rospy.loginfo(f"实际前进距离: {traveled_distance:.3f}m, 目标距离: {self.target_distance:.3f}m")
            return
        
        # 前进
        self.target_reached = False
        self.send_control_command('A')
    
    def run(self):
        """主循环"""
        rospy.spin()
        
        # 程序退出时停止小车
        self.cleanup()

    def cleanup(self):
        """清理资源"""
        self.imu_running = False
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
        navigator = CarNavigation()
        navigator.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"程序异常: {e}")
    finally:
        rospy.loginfo("程序结束")
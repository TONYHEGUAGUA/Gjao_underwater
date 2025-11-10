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
        
        # IMU数据相关变量
        self.current_imu_yaw = 0.0
        self.last_imu_time = 0
        self.imu_initialized = False
        self.imu_initial_yaw = 0.0
        self.imu_calibration_offset = 0.0  # IMU校准偏移量
        
        # 串口初始化
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
        
        # 控制状态变量
        self.control_active = False
        self.target_reached = False
        
        # 控制参数
        self.position_tolerance = 0.08  # 8cm容差
        self.angle_tolerance = 0.08    # 约9度角度容差
        
        # 目标点导航相关变量
        self.target_point = None  # 目标点 [x, y]
        self.target_yaw = 0.0     # 目标朝向
        self.start_pose = None    # 导航开始时的位置
        self.target_distance = 0.0  # 需要前进的总距离
        self.turned_to_target = False  # 是否已经转向目标方向
        
        # TF监听器 - 用于获取机器人在map坐标系中的位置
        self.tf_listener = tf.TransformListener()
        
        # 等待TF数据稳定
        self.wait_for_tf_stabilize()
        
        # 执行IMU校准
        self.calibrate_imu_with_slam()
        
        # 订阅/move_base_simple/goal话题
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        
        # 高频控制循环 (50Hz)
        self.control_timer = rospy.Timer(rospy.Duration(0.02), self.control_loop)
        
        rospy.loginfo("简单目标导航器初始化完成")
        rospy.loginfo("等待Foxglove发送目标点...")
    
    def wait_for_tf_stabilize(self):
        """等待TF数据稳定"""
        rospy.loginfo("等待TF数据稳定...")
        for i in range(50):  # 等待最多5秒
            try:
                self.tf_listener.waitForTransform('map', 'base_link', rospy.Time(0), rospy.Duration(1.0))
                rospy.loginfo("TF数据已稳定")
                return
            except:
                if i % 10 == 0:
                    rospy.loginfo(f"等待TF数据... ({i/10 + 1}s)")
                rospy.sleep(0.1)
        rospy.logwarn("TF数据等待超时，继续初始化...")
    
    def calibrate_imu_with_slam(self):
        """使用SLAM的四元数yaw校准IMU"""
        rospy.loginfo("开始IMU校准...")
        
        # 等待IMU数据初始化
        start_time = rospy.get_time()
        while not self.imu_initialized and (rospy.get_time() - start_time) < 10.0:
            rospy.sleep(0.1)
        
        if not self.imu_initialized:
            rospy.logwarn("IMU数据未初始化，跳过校准")
            return
        
        # 获取SLAM的当前yaw
        slam_yaw = self.get_slam_yaw()
        if slam_yaw is None:
            rospy.logwarn("无法获取SLAM yaw，跳过校准")
            return
        
        # 获取当前IMU yaw
        imu_yaw = self.current_imu_yaw
        
        # 计算校准偏移量
        self.imu_calibration_offset = slam_yaw - imu_yaw
        
        # 规范化偏移量到[-pi, pi]范围
        while self.imu_calibration_offset > math.pi:
            self.imu_calibration_offset -= 2 * math.pi
        while self.imu_calibration_offset < -math.pi:
            self.imu_calibration_offset += 2 * math.pi
        
        rospy.loginfo("=== IMU校准完成 ===")
        rospy.loginfo(f"SLAM角度: {math.degrees(slam_yaw):.2f}°")
        rospy.loginfo(f"IMU原始角度: {math.degrees(imu_yaw):.2f}°")
        rospy.loginfo(f"校准偏移量: {math.degrees(self.imu_calibration_offset):.2f}°")
        rospy.loginfo(f"校准后IMU角度: {math.degrees(imu_yaw + self.imu_calibration_offset):.2f}°")
        
    def get_slam_yaw(self):
        """从SLAM获取当前偏航角"""
        try:
            (trans, rot) = self.tf_listener.lookupTransform('map', 'base_link', rospy.Time(0))
            # 从四元数提取偏航角
            _, _, yaw = euler_from_quaternion(rot)
            return yaw
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn(f"获取SLAM yaw失败: {e}")
            return None
        
    def goal_callback(self, msg):
        """处理Foxglove发送的目标点"""
        if self.current_pose is None or not self.imu_initialized:
            rospy.logwarn("无法开始导航: 等待定位和IMU数据初始化")
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
        
        # 计算目标距离
        dx = target_x - self.current_pose.position.x
        dy = target_y - self.current_pose.position.y
        self.target_distance = math.sqrt(dx**2 + dy**2)
        
        rospy.loginfo(f"收到新目标点: ({target_x:.2f}, {target_y:.2f})")
        rospy.loginfo(f"目标朝向: {math.degrees(self.target_yaw):.1f}°")
        rospy.loginfo(f"目标距离: {self.target_distance:.2f}m")
        
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
            frame = frame.strip()
            if not frame.startswith('$IMU,'):
                return None
                
            parts = frame[1:].split(',')
            if len(parts) != 3:
                return None
                
            frame_type, yaw_str, checksum_str = parts
            
            if frame_type != 'IMU':
                return None
                
            payload = f"IMU,{yaw_str}"
            if not self.verify_checksum(payload, checksum_str):
                return None
            
            yaw = float(yaw_str)
            
            return {
                'type': 'IMU',
                'yaw': math.radians(yaw),  # 转换为弧度
                'checksum': checksum_str,
                'timestamp': rospy.get_time()
            }
            
        except ValueError:
            return None
        except Exception:
            return None
    
    def read_imu_data(self):
        """读取IMU串口数据"""
        buffer = ""
        while self.imu_running and not rospy.is_shutdown():
            try:
                if self.ser.in_waiting > 0:
                    data = self.ser.read(self.ser.in_waiting).decode('ascii', errors='ignore')
                    buffer += data
                    
                    while '\r\n' in buffer:
                        frame_end = buffer.find('\r\n')
                        frame = buffer[:frame_end]
                        buffer = buffer[frame_end + 2:]
                        
                        if frame:
                            imu_data = self.parse_imu_frame(frame)
                            if imu_data:
                                self.current_imu_yaw = imu_data['yaw']
                                self.last_imu_time = rospy.get_time()
                                
                                if not self.imu_initialized:
                                    self.imu_initial_yaw = self.current_imu_yaw
                                    self.imu_initialized = True
                                    rospy.loginfo(f"IMU初始化完成，原始偏航角: {math.degrees(self.imu_initial_yaw):.2f}°")
                
                rospy.sleep(0.001)
                
            except Exception as e:
                rospy.logwarn(f"读取IMU数据错误: {e}")
                rospy.sleep(0.1)
    
    def get_current_yaw(self):
        """获取当前偏航角 - 使用校准后的IMU数据"""
        if self.imu_initialized:
            calibrated_yaw = self.current_imu_yaw + self.imu_calibration_offset
            
            # 规范化角度到[-pi, pi]范围
            while calibrated_yaw > math.pi:
                calibrated_yaw -= 2 * math.pi
            while calibrated_yaw < -math.pi:
                calibrated_yaw += 2 * math.pi
                
            return calibrated_yaw
        else:
            return 0.0
    
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
    
    def get_robot_pose(self):
        """通过TF获取机器人在map坐标系中的位置"""
        try:
            (trans, rot) = self.tf_listener.lookupTransform('map', 'base_link', rospy.Time(0))
            # 创建Pose消息来保持接口兼容
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
        """计算角度误差 - 使用校准后的IMU数据"""
        current_yaw = self.get_current_yaw()
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
        rospy.loginfo("停止小车")
    
    def control_loop(self, event):
        """高频控制循环 (50Hz)"""
        # 更新当前位置
        self.current_pose = self.get_robot_pose()
        
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
        current_raw_imu = self.current_imu_yaw if self.imu_initialized else 0.0
        
        rospy.loginfo_throttle(1, f"转向目标: 校准角度={math.degrees(current_yaw):.2f}°, 原始IMU={math.degrees(current_raw_imu):.2f}°, 目标角度={math.degrees(target_angle):.2f}°, 误差={math.degrees(angle_error):.2f}°")
        
        # 检查是否对准目标
        if abs(angle_error) < self.angle_tolerance:
            if not self.target_reached:
                self.stop_car()
                self.target_reached = True
                self.turned_to_target = True
                rospy.loginfo("✓ 已对准目标方向，开始前进")
                rospy.sleep(1.0)
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
        rospy.loginfo_throttle(1, f"前往目标: 已前进={traveled_distance:.3f}m, 剩余={remaining_distance:.3f}m, 总距离={self.target_distance:.3f}m, 当前角度={math.degrees(current_yaw):.1f}°")
        
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
        navigator = SimpleGoalNavigator()
        navigator.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"程序异常: {e}")
    finally:
        rospy.loginfo("程序结束")
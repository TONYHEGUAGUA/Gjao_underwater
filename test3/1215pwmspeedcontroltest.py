#!/usr/bin/env python3
import rospy
import math
import serial
import threading
import tf
import numpy as np
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

class AdvancedGoalNavigator:
    def __init__(self):
        rospy.init_node('advanced_goal_navigator', anonymous=True)
        
        # IMU数据相关变量
        self.current_imu_yaw = 0.0
        self.last_imu_time = 0
        self.imu_initialized = False
        self.imu_calibration_offset = 0.0
        
        # PWM控制参数
        self.PWM_NEUTRAL = 1500  # 中立/停止 PWM值
        self.PWM_MIN = 1000      # 最小PWM值
        self.PWM_MAX = 2000      # 最大PWM值
        self.MAX_SPEED_PWM = 200  # 最大前进PWM偏移量
        self.MIN_SPEED_PWM = 30   # 最小前进PWM偏移量
        self.MIN_OPERATING_PWM = 80  # 最低工作PWM
        self.MAX_TURN_PWM = 150   # 转向PWM偏移量
        
        # 转向参数（新增）
        self.TURN_SPEED_PWM = 80  # 转向时单侧电机速度
        self.TURN_ANGLE_THRESHOLD = math.radians(30)  # 30度，超过这个角度先转向
        self.TURN_COMPLETE_THRESHOLD = math.radians(5)  # 5度，转向完成阈值
        
        # 电机方向修正
        self.REVERSE_LEFT_MOTOR = True 
        self.REVERSE_RIGHT_MOTOR = True
        
        # 当前PWM状态
        self.current_left_pwm = self.PWM_NEUTRAL
        self.current_right_pwm = self.PWM_NEUTRAL
        
        # PID控制器参数
        self.angle_pid = PIDController(kp=2.0, ki=0.02, kd=0.3, output_limit=1.0)
        
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
        
        # 初始化负压吸附
        self.initialize_vacuum()
        
        # 状态变量
        self.current_pose = None
        self.initial_pose = None
        
        # 控制状态
        self.control_active = False
        self.target_reached = False
        self.is_turning = False  # 是否正在转向
        self.turn_direction = 0  # 转向方向：1=左转，-1=右转
        self.turn_start_time = 0
        
        # 控制参数
        self.position_tolerance = 0.15  # 15cm位置容差
        
        # 目标点导航变量
        self.target_point = None
        self.target_yaw = 0.0
        self.start_pose = None
        self.target_distance = 0.0
        
        # 速度控制参数 - 修改为三个阶段
        self.fast_distance_threshold = 1.0  # 快速行驶距离阈值
        self.medium_distance_threshold = 0.5  # 中速行驶距离阈值
        self.slow_distance_threshold = 0.3  # 慢速行驶距离阈值 - 从0.2改为0.3
        self.min_operating_distance = 0.1  # 最小操作距离
        
        # 各阶段的目标PWM平均值
        self.fast_target_avg_pwm = 1650  # 快速阶段平均PWM
        self.medium_target_avg_pwm = 1600  # 中速阶段平均PWM
        self.slow_target_avg_pwm = 1580  # 慢速阶段平均PWM
        
        # TF监听器
        self.tf_listener = tf.TransformListener()
        
        # 等待TF数据稳定
        self.wait_for_tf_stabilize()
        
        # 执行IMU校准
        self.calibrate_imu_with_slam()
        
        # 订阅目标点话题
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        
        # 高频控制循环 (50Hz)
        self.control_timer = rospy.Timer(rospy.Duration(0.02), self.control_loop)
        
        rospy.loginfo("高级目标导航器初始化完成")
        rospy.loginfo("等待Foxglove发送目标点...")
    
    def wait_for_tf_stabilize(self):
        """等待TF数据稳定"""
        rospy.loginfo("等待TF数据稳定...")
        for i in range(50):
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
        
        # 多次采样取平均值
        samples = []
        for _ in range(20):
            slam_yaw = self.get_slam_yaw()
            imu_yaw = self.current_imu_yaw
            if slam_yaw is not None:
                samples.append((slam_yaw, imu_yaw))
            rospy.sleep(0.05)
        
        if len(samples) < 5:
            rospy.logwarn("采样数据不足，跳过校准")
            return
        
        # 计算平均偏移量
        avg_offset = np.mean([s[0] - s[1] for s in samples])
        
        # 规范化偏移量到[-pi, pi]范围
        self.imu_calibration_offset = self.normalize_angle(avg_offset)
        
        rospy.loginfo("=== IMU校准完成 ===")
        rospy.loginfo(f"校准偏移量: {math.degrees(self.imu_calibration_offset):.2f}°")
        
    def get_slam_yaw(self):
        """从SLAM获取当前偏航角"""
        try:
            (trans, rot) = self.tf_listener.lookupTransform('map', 'base_link', rospy.Time(0))
            _, _, yaw = euler_from_quaternion(rot)
            return yaw
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None
        
    def goal_callback(self, msg):
        """处理Foxglove发送的目标点"""
        if self.current_pose is None or not self.imu_initialized:
            rospy.logwarn("无法开始导航: 等待定位和IMU数据初始化")
            return
            
        self.stop_current_mission()
        rospy.sleep(0.5)
        
        # 提取目标位置和朝向
        target_x = msg.pose.position.x
        target_y = msg.pose.position.y
        orientation = msg.pose.orientation
        _, _, target_yaw = euler_from_quaternion([orientation.x, orientation.y, 
                                                  orientation.z, orientation.w])
        
        self.target_point = [target_x, target_y]
        self.target_yaw = target_yaw
        self.start_pose = self.current_pose
        
        # 计算初始距离和角度
        dx = target_x - self.current_pose.position.x
        dy = target_y - self.current_pose.position.y
        self.target_distance = math.sqrt(dx**2 + dy**2)
        
        # 计算初始角度误差
        target_angle = math.atan2(dy, dx)
        current_yaw = self.get_current_yaw()
        initial_angle_error = target_angle - current_yaw
        initial_angle_error = self.normalize_angle(initial_angle_error)
        
        # 重置PID控制器
        self.angle_pid.reset()
        
        # 重置转向状态
        self.is_turning = False
        self.turn_direction = 0
        
        rospy.loginfo("=" * 50)
        rospy.loginfo(f"收到新目标点: ({target_x:.2f}, {target_y:.2f})")
        rospy.loginfo(f"目标朝向: {math.degrees(target_yaw):.1f}°")
        rospy.loginfo(f"初始距离: {self.target_distance:.2f}m")
        rospy.loginfo(f"初始角度误差: {math.degrees(initial_angle_error):.1f}°")
        
        # 检查是否需要先转向
        if abs(initial_angle_error) > self.TURN_ANGLE_THRESHOLD:
            rospy.logwarn(f"初始角度误差较大({math.degrees(initial_angle_error):.1f}°)，将先转向对准目标点方向")
            # 设置转向方向
            if initial_angle_error > 0:
                self.turn_direction = 1  # 左转
            else:
                self.turn_direction = -1  # 右转
        
        rospy.loginfo("=" * 50)
        
        self.control_active = True
        self.target_reached = False
    
    def stop_current_mission(self):
        """停止当前任务"""
        self.control_active = False
        self.target_reached = False
        self.is_turning = False
        self.turn_direction = 0
        self.send_pwm_command(self.PWM_NEUTRAL, self.PWM_NEUTRAL)
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
                'yaw': math.radians(yaw),
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
                                    self.imu_initialized = True
                                    rospy.loginfo("IMU初始化完成")
                
                rospy.sleep(0.001)
                
            except Exception as e:
                rospy.logwarn(f"读取IMU数据错误: {e}")
                rospy.sleep(0.1)
    
    def normalize_angle(self, angle):
        """规范化角度到[-π, π]范围"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def get_current_yaw(self):
        """获取当前偏航角 - 使用校准后的IMU数据"""
        if self.imu_initialized:
            calibrated_yaw = self.current_imu_yaw + self.imu_calibration_offset
            return self.normalize_angle(calibrated_yaw)
        else:
            return 0.0
    
    def initialize_vacuum(self):
        """初始化负压吸附"""
        for i in range(9):
            self.ser.write('3'.encode())
            rospy.sleep(0.1)
        self.ser.write('!'.encode())
        rospy.loginfo("负压吸附已开启")
        rospy.sleep(0.5)
    
    def send_pwm_command(self, left_pwm, right_pwm):
        """发送PWM控制命令"""
        try:
            # 限制PWM值在安全范围内
            left_pwm = max(self.PWM_MIN, min(self.PWM_MAX, int(left_pwm)))
            right_pwm = max(self.PWM_MIN, min(self.PWM_MAX, int(right_pwm)))
            
            # 应用电机反转修正
            if self.REVERSE_LEFT_MOTOR:
                left_pwm = 2 * self.PWM_NEUTRAL - left_pwm
            if self.REVERSE_RIGHT_MOTOR:
                right_pwm = 2 * self.PWM_NEUTRAL - right_pwm
            
            # 再次限制反转后的PWM值
            left_pwm = max(self.PWM_MIN, min(self.PWM_MAX, int(left_pwm)))
            right_pwm = max(self.PWM_MIN, min(self.PWM_MAX, int(right_pwm)))
            
            # 格式化命令：M,left_pwm,right_pwm\n
            command = f"M,{left_pwm},{right_pwm}\n"
            self.ser.write(command.encode())
            
            self.current_left_pwm = left_pwm
            self.current_right_pwm = right_pwm
            
        except Exception as e:
            rospy.logerr(f"发送PWM命令失败: {e}")
    
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
    
    def calculate_remaining_distance(self):
        """计算到目标点的剩余距离"""
        if self.target_point is None or self.current_pose is None:
            return float('inf')
            
        dx = self.target_point[0] - self.current_pose.position.x
        dy = self.target_point[1] - self.current_pose.position.y
        return math.sqrt(dx**2 + dy**2)
    
    def calculate_target_angle(self):
        """计算到目标点的角度"""
        if self.target_point is None or self.current_pose is None:
            return 0.0
            
        dx = self.target_point[0] - self.current_pose.position.x
        dy = self.target_point[1] - self.current_pose.position.y
        return math.atan2(dy, dx)
    
    def calculate_angle_error(self, target_angle):
        """计算当前航向与目标航向的误差"""
        current_yaw = self.get_current_yaw()
        error = target_angle - current_yaw
        return self.normalize_angle(error)
    
    def get_target_speed_pwm(self, distance):
        """根据距离获取目标PWM值"""
        if distance >= self.fast_distance_threshold:
            target_avg_pwm = self.fast_target_avg_pwm
        elif distance >= self.medium_distance_threshold:
            ratio = (distance - self.medium_distance_threshold) / (self.fast_distance_threshold - self.medium_distance_threshold)
            target_avg_pwm = self.medium_target_avg_pwm + ratio * (self.fast_target_avg_pwm - self.medium_target_avg_pwm)
        elif distance >= self.slow_distance_threshold:
            target_avg_pwm = self.medium_target_avg_pwm
        elif distance >= self.min_operating_distance:
            ratio = (distance - self.min_operating_distance) / (self.slow_distance_threshold - self.min_operating_distance)
            target_avg_pwm = self.slow_target_avg_pwm + ratio * (self.medium_target_avg_pwm - self.slow_target_avg_pwm)
        else:
            target_avg_pwm = self.slow_target_avg_pwm
        
        base_pwm = target_avg_pwm - self.PWM_NEUTRAL
        base_pwm = max(self.MIN_OPERATING_PWM, base_pwm)
        
        return base_pwm
    
    def calculate_pwm_for_turning(self, turn_direction):
        """计算转向时的PWM值"""
        if turn_direction == 1:  # 左转
            # 改为：向右转的逻辑
            left_pwm = self.PWM_NEUTRAL + self.TURN_SPEED_PWM  # 前进
            right_pwm = self.PWM_NEUTRAL - self.TURN_SPEED_PWM  # 后退
        else:  # 右转
            # 改为：向左转的逻辑
            left_pwm = self.PWM_NEUTRAL - self.TURN_SPEED_PWM  # 后退
            right_pwm = self.PWM_NEUTRAL + self.TURN_SPEED_PWM  # 前进
    
        # 确保PWM在安全范围内
        left_pwm = max(self.PWM_MIN, min(self.PWM_MAX, left_pwm))
        right_pwm = max(self.PWM_MIN, min(self.PWM_MAX, right_pwm))   
        avg_pwm = (left_pwm + right_pwm) / 2   
        return left_pwm, right_pwm, avg_pwm
    
    def calculate_pwm_for_moving(self, distance, turn_output, angle_error):
        """计算前进时的PWM值"""
        # 获取基础速度PWM
        base_speed = self.get_target_speed_pwm(distance)
        
        # 动态转向增益
        angle_gain = 1.0
        if abs(angle_error) > math.radians(60):  # 约60度
            angle_gain = 1.5
        elif abs(angle_error) > math.radians(30):  # 约30度
            angle_gain = 1.2
        
        # 限制转向输出
        turn_output = max(-0.8, min(0.8, turn_output))
        
        # 计算转向偏移量
        turn_offset = turn_output * self.MAX_TURN_PWM * angle_gain
        
        left_pwm = self.PWM_NEUTRAL + base_speed + turn_offset
        right_pwm = self.PWM_NEUTRAL + base_speed - turn_offset
        
        # 安全限制
        min_pwm = self.PWM_NEUTRAL + 20
        max_pwm = self.PWM_NEUTRAL + self.MAX_SPEED_PWM
        
        left_pwm = max(min_pwm, min(max_pwm, left_pwm))
        right_pwm = max(min_pwm, min(max_pwm, right_pwm))
        
        avg_pwm = (left_pwm + right_pwm) / 2
        
        return left_pwm, right_pwm, avg_pwm, angle_gain
    
    def control_loop(self, event):
        """主控制循环"""
        # 更新当前位置
        self.current_pose = self.get_robot_pose()
        
        if not self.control_active or self.target_reached:
            return
            
        if self.current_pose is None or not self.imu_initialized:
            return
        
        # 计算剩余距离
        remaining_distance = self.calculate_remaining_distance()
        
        # 检查是否到达目标
        if remaining_distance <= self.position_tolerance:
            if not self.target_reached:
                self.target_reached = True
                self.control_active = False
                self.send_pwm_command(self.PWM_NEUTRAL, self.PWM_NEUTRAL)
                rospy.loginfo(f"✓ 已到达目标点！")
                rospy.loginfo(f"位置误差: {remaining_distance:.3f}m")
            return
        
        # 计算当前到目标点的角度误差
        target_angle = self.calculate_target_angle()
        angle_error_to_target = self.calculate_angle_error(target_angle)
        
        # 计算最终目标朝向误差
        final_angle_error = self.calculate_angle_error(self.target_yaw)
        
        # 状态机：检查是否需要转向
        if not self.is_turning:
            # 检查角度误差是否超过阈值
            if abs(angle_error_to_target) > self.TURN_ANGLE_THRESHOLD:
                self.is_turning = True
                self.turn_start_time = rospy.get_time()
                # 确定转向方向
                if angle_error_to_target > 0:
                    self.turn_direction = 1  # 左转
                    turn_dir_str = "左转"
                else:
                    self.turn_direction = -1  # 右转
                    turn_dir_str = "右转"
                
                rospy.loginfo(f"角度误差过大({math.degrees(angle_error_to_target):.1f}°)，开始{turn_dir_str}对准目标点方向")
        
        if self.is_turning:
            # 转向阶段
            # 检查是否已经对准
            if abs(angle_error_to_target) <= self.TURN_COMPLETE_THRESHOLD:
                self.is_turning = False
                self.turn_direction = 0
                rospy.loginfo(f"转向完成，角度误差: {math.degrees(angle_error_to_target):.1f}°")
            
            # 计算转向PWM
            left_pwm, right_pwm, avg_pwm = self.calculate_pwm_for_turning(self.turn_direction)
            
            # 发送PWM命令
            self.send_pwm_command(left_pwm, right_pwm)
            
            # 输出转向状态
            current_time = rospy.get_time()
            if hasattr(self, 'last_status_time'):
                if current_time - self.last_status_time > 0.5:  # 每0.5秒输出一次
                    turn_dir = "左转" if self.turn_direction == 1 else "右转"
                    rospy.loginfo_throttle(0.5, 
                        f"{turn_dir}中: 距离={remaining_distance:.3f}m | "
                        f"目标点角度误差={math.degrees(angle_error_to_target):.1f}° | "
                        f"最终朝向误差={math.degrees(final_angle_error):.1f}° | "
                        f"平均PWM={avg_pwm:.0f} | PWM=({int(left_pwm)}, {int(right_pwm)})")
            else:
                self.last_status_time = current_time
                
        else:
            # 前进阶段
            # 计算转向输出
            turn_output = self.angle_pid.update(angle_error_to_target)
            
            # 计算前进PWM
            left_pwm, right_pwm, avg_pwm, angle_gain = self.calculate_pwm_for_moving(
                remaining_distance, turn_output, angle_error_to_target
            )
            
            # 发送PWM命令
            self.send_pwm_command(left_pwm, right_pwm)
            
            # 定期输出状态信息
            current_time = rospy.get_time()
            if hasattr(self, 'last_status_time'):
                if current_time - self.last_status_time > 0.5:  # 每0.5秒输出一次
                    # 显示速度阶段 - 修改为只有三个阶段
                    if remaining_distance >= self.fast_distance_threshold:
                        speed_stage = "快速"
                    elif remaining_distance >= self.medium_distance_threshold:
                        speed_stage = "中速"
                    else:  # 距离 < 0.5m
                        speed_stage = "慢速"
                    
                    # 显示角度控制信息
                    angle_info = ""
                    if angle_gain > 1.0:
                        angle_info = f" 转向增益: x{angle_gain:.1f}"
                    
                    rospy.loginfo_throttle(0.5, 
                        f"前进({speed_stage}): 距离={remaining_distance:.3f}m | "
                        f"目标点角度误差={math.degrees(angle_error_to_target):.1f}° | "
                        f"最终朝向误差={math.degrees(final_angle_error):.1f}° | "
                        f"平均PWM={avg_pwm:.0f} | 转向={turn_output:.2f}{angle_info} | "
                        f"PWM=({int(left_pwm)}, {int(right_pwm)})")
            else:
                self.last_status_time = current_time
    
    def run(self):
        """主循环"""
        rospy.spin()
        self.cleanup()

    def cleanup(self):
        """清理资源"""
        self.imu_running = False
        self.control_active = False
        self.send_pwm_command(self.PWM_NEUTRAL, self.PWM_NEUTRAL)
        try:
            self.control_timer.shutdown()
            self.ser.close()
        except:
            pass


class PIDController:
    """PID控制器"""
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, output_limit=1.0, integral_limit=1.0, windup_limit=0.5):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        self.integral_limit = integral_limit
        self.windup_limit = windup_limit
        
        self.reset()
    
    def reset(self):
        """重置PID控制器"""
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = rospy.get_time() if rospy.get_time() > 0 else 0.0
        self.last_output = 0.0
    
    def update(self, error):
        """PID更新"""
        current_time = rospy.get_time()
        dt = current_time - self.last_time if self.last_time > 0 else 0.02
        
        # 避免dt过小或过大
        dt = max(0.001, min(dt, 0.1))
        
        # 积分项
        self.integral += error * dt
        
        # 积分限幅和抗饱和
        if self.ki > 0:
            if abs(self.last_output) >= self.output_limit * 0.8:
                if error * self.last_output <= 0:
                    self.integral += error * dt * 0.3
                else:
                    self.integral *= 0.6
            
            self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))
        
        # 微分项
        derivative = (error - self.last_error) / dt if dt > 0 else 0.0
        
        # 计算输出
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        
        # 输出限幅
        output = max(-self.output_limit, min(self.output_limit, output))
        
        # 更新状态
        self.last_error = error
        self.last_time = current_time
        self.last_output = output
        
        return output


if __name__ == '__main__':
    try:
        navigator = AdvancedGoalNavigator()
        navigator.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"程序异常: {e}")
    finally:
        rospy.loginfo("程序结束")
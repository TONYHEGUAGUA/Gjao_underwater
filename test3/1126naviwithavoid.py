#!/usr/bin/env python3
import rospy
import math
import serial
import threading
import tf
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from tf.transformations import euler_from_quaternion

class StablePathNavigator:
    def __init__(self):
        rospy.init_node('stable_path_navigator', anonymous=True)
        
        # IMU数据相关变量
        self.current_imu_yaw = 0.0
        self.last_imu_time = 0
        self.imu_initialized = False
        self.imu_calibration_offset = 0.0
        
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
        
        # 路径跟踪相关变量
        self.waypoints = []  # 关键点列表
        self.current_waypoint_index = -1
        self.control_active = False
        self.path_received = False  # 标记是否已收到路径
        
        # 控制参数
        self.position_tolerance = 0.10  # 10cm容差
        self.angle_tolerance = 0.10    # 约11.5度角度容差
        self.final_angle_tolerance = 0.05  # 最终朝向容差
        
        # 控制状态机
        self.control_state = "IDLE"  # IDLE, TURNING, MOVING, ADJUSTING_FINAL
        
        # 移动相关变量
        self.start_pose = None  # 移动开始时的位置
        self.target_distance = 0.0  # 需要移动的距离
        
        # TF监听器
        self.tf_listener = tf.TransformListener()
        
        # 等待TF数据稳定
        self.wait_for_tf_stabilize()
        
        # 执行IMU校准
        self.calibrate_imu_with_slam()
        
        # 订阅话题
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        rospy.Subscriber('/move_base/TrajectoryPlannerROS/global_plan', Path, self.global_plan_callback)
        
        # 发布简化后的路径（用于可视化）
        self.simplified_path_pub = rospy.Publisher('/simplified_path', Path, queue_size=10)
        
        # 高频控制循环 (50Hz)
        self.control_timer = rospy.Timer(rospy.Duration(0.02), self.control_loop)
        
        rospy.loginfo("稳定路径导航器初始化完成")
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
        
        slam_yaw = self.get_slam_yaw()
        if slam_yaw is None:
            rospy.logwarn("无法获取SLAM yaw，跳过校准")
            return
        
        imu_yaw = self.current_imu_yaw
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
    
    def get_slam_yaw(self):
        """从SLAM获取当前偏航角"""
        try:
            (trans, rot) = self.tf_listener.lookupTransform('map', 'base_link', rospy.Time(0))
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
        
        # 重置路径接收标志
        self.path_received = False
        
        # 提取目标位置和朝向
        target_x = msg.pose.position.x
        target_y = msg.pose.position.y
        orientation = msg.pose.orientation
        _, _, target_yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        
        # 设置最终目标点
        self.final_target = {'x': target_x, 'y': target_y, 'yaw': target_yaw}
        
        rospy.loginfo(f"收到新目标点: ({target_x:.2f}, {target_y:.2f}), 朝向: {math.degrees(target_yaw):.1f}°")
        rospy.loginfo("等待全局路径规划...")
    
    def global_plan_callback(self, msg):
        """处理全局路径规划结果 - 只使用第一次收到的路径"""
        if not hasattr(self, 'final_target') or self.path_received:
            return
        
        rospy.loginfo(f"收到全局路径，包含 {len(msg.poses)} 个路径点")
        
        # 标记已收到路径，忽略后续更新
        self.path_received = True
        
        # 提取关键点
        self.extract_key_waypoints(msg.poses)
        
        if len(self.waypoints) > 0:
            # 添加最终目标点
            self.waypoints.append(self.final_target)
            
            rospy.loginfo(f"路径简化完成: {len(msg.poses)} → {len(self.waypoints)} 个关键点")
            for i, wp in enumerate(self.waypoints):
                rospy.loginfo(f"  关键点 {i}: ({wp['x']:.2f}, {wp['y']:.2f})")
            
            # 发布简化路径用于可视化
            self.publish_simplified_path()
            
            # 开始导航
            self.current_waypoint_index = 0
            self.control_active = True
            self.control_state = "TURNING"  # 初始状态为转向
            
            rospy.loginfo(f"开始导航到关键点 0/{len(self.waypoints)-1}")
        else:
            rospy.logwarn("无法提取关键点")
    
    def extract_key_waypoints(self, poses):
        """从全局路径中提取关键点 - 使用更激进的简化"""
        self.waypoints = []
        
        if len(poses) < 2:
            return
        
        # 方法：只取起点、中间几个关键转折点和终点
        total_points = len(poses)
        
        # 总是包含起点
        start_pose = poses[0].pose
        self.waypoints.append({
            'x': start_pose.position.x,
            'y': start_pose.position.y,
            'yaw': self.get_yaw_from_pose(start_pose)
        })
        
        # 如果路径很长，取1/3和2/3处的点作为关键点
        if total_points > 10:
            mid1_index = total_points // 3
            mid2_index = 2 * total_points // 3
            
            mid1_pose = poses[mid1_index].pose
            mid2_pose = poses[mid2_index].pose
            
            self.waypoints.append({
                'x': mid1_pose.position.x,
                'y': mid1_pose.position.y,
                'yaw': self.get_yaw_from_pose(mid1_pose)
            })
            
            self.waypoints.append({
                'x': mid2_pose.position.x,
                'y': mid2_pose.position.y,
                'yaw': self.get_yaw_from_pose(mid2_pose)
            })
        
        # 对于中等长度路径，只取中间一个点
        elif total_points > 5:
            mid_index = total_points // 2
            mid_pose = poses[mid_index].pose
            self.waypoints.append({
                'x': mid_pose.position.x,
                'y': mid_pose.position.y,
                'yaw': self.get_yaw_from_pose(mid_pose)
            })
        
        rospy.loginfo(f"提取了 {len(self.waypoints)} 个关键点")
    
    def get_yaw_from_pose(self, pose):
        """从Pose消息中提取偏航角"""
        orientation = pose.orientation
        _, _, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        return yaw
    
    def publish_simplified_path(self):
        """发布简化后的路径用于可视化"""
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()
        
        for waypoint in self.waypoints:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "map"
            pose_stamped.pose.position.x = waypoint['x']
            pose_stamped.pose.position.y = waypoint['y']
            pose_stamped.pose.position.z = 0.0
            path_msg.poses.append(pose_stamped)
        
        self.simplified_path_pub.publish(path_msg)
        rospy.loginfo("已发布简化路径到 /simplified_path")
    
    def stop_current_mission(self):
        """停止当前任务"""
        self.control_active = False
        self.path_received = False
        self.waypoints = []
        self.current_waypoint_index = -1
        self.control_state = "IDLE"
        self.stop_car()
        rospy.loginfo("当前任务已停止")
    
    def calculate_checksum(self, data):
        """计算XOR校验和"""
        checksum = 0
        for byte in data.encode('ascii'):
            checksum ^= byte
        return checksum
    
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
    
    def verify_checksum(self, payload, received_checksum):
        """验证校验和"""
        try:
            calculated_checksum = self.calculate_checksum(payload)
            return calculated_checksum == int(received_checksum, 16)
        except:
            return False
    
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
                                    rospy.loginfo(f"IMU初始化完成，原始偏航角: {math.degrees(self.current_imu_yaw):.2f}°")
                
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
    
    def calculate_distance_to_waypoint(self, waypoint):
        """计算到目标点的距离"""
        if self.current_pose is None:
            return float('inf')
            
        dx = waypoint['x'] - self.current_pose.position.x
        dy = waypoint['y'] - self.current_pose.position.y
        return math.sqrt(dx**2 + dy**2)
    
    def calculate_angle_to_waypoint(self, waypoint):
        """计算到目标点的角度"""
        if self.current_pose is None:
            return 0.0
            
        dx = waypoint['x'] - self.current_pose.position.x
        dy = waypoint['y'] - self.current_pose.position.y
        return math.atan2(dy, dx)
    
    def calculate_angle_error(self, target_yaw):
        """计算角度误差"""
        current_yaw = self.get_current_yaw()
        error = target_yaw - current_yaw
        
        # 将角度误差规范化到[-pi, pi]
        while error > math.pi:
            error -= 2 * math.pi
        while error < -math.pi:
            error += 2 * math.pi
        
        return error
    
    def calculate_distance_traveled(self):
        """计算从开始位置移动的距离"""
        if self.current_pose is None or self.start_pose is None:
            return 0.0
            
        dx = self.current_pose.position.x - self.start_pose.position.x
        dy = self.current_pose.position.y - self.start_pose.position.y
        return math.sqrt(dx**2 + dy**2)
    
    def send_control_command(self, command):
        """发送控制命令"""
        try:
            self.ser.write(command.encode())
        except Exception as e:
            rospy.logerr(f"发送命令失败: {e}")
    
    def stop_car(self):
        """停止小车"""
        self.send_control_command('0')
    
    def control_loop(self, event):
        """高频控制循环"""
        # 更新当前位置
        self.current_pose = self.get_robot_pose()
        
        if not self.control_active or self.current_waypoint_index < 0:
            return
            
        if self.current_pose is None or not self.imu_initialized:
            return
        
        # 路径点导航控制
        self.waypoint_navigation_control()
    
    def waypoint_navigation_control(self):
        """路径点导航控制 - 改进的状态机逻辑"""
        if self.current_waypoint_index >= len(self.waypoints):
            rospy.loginfo("🎉 所有路径点导航完成！")
            self.control_active = False
            self.stop_car()
            return
        
        current_waypoint = self.waypoints[self.current_waypoint_index]
        
        # 状态机控制
        if self.control_state == "TURNING":
            self.handle_turning_state(current_waypoint)
        elif self.control_state == "MOVING":
            self.handle_moving_state(current_waypoint)
        elif self.control_state == "ADJUSTING_FINAL":
            self.handle_adjusting_final_state()
    
    def handle_turning_state(self, waypoint):
        """处理转向状态"""
        target_angle = self.calculate_angle_to_waypoint(waypoint)
        angle_error = self.calculate_angle_error(target_angle)
        
        rospy.loginfo_throttle(2, 
            f"转向状态: 路径点 {self.current_waypoint_index}, "
            f"角度误差={math.degrees(angle_error):.1f}°")
        
        # 如果角度对准，切换到移动状态
        if abs(angle_error) < self.angle_tolerance:
            self.stop_car()
            rospy.sleep(0.2)  # 短暂停止
            
            # 计算需要移动的距离
            distance = self.calculate_distance_to_waypoint(waypoint)
            self.target_distance = distance
            self.start_pose = self.current_pose  # 记录开始位置
            
            self.control_state = "MOVING"
            rospy.loginfo(f"角度对准，开始移动 {distance:.2f}米")
            return
        
        # 转向控制 - 只转向，不前进
        if angle_error > 0:
            self.send_control_command('D')  # 左转
        else:
            self.send_control_command('C')  # 右转
    
    def handle_moving_state(self, waypoint):
        """处理移动状态"""
        distance_traveled = self.calculate_distance_traveled()
        remaining_distance = self.target_distance - distance_traveled
        
        rospy.loginfo_throttle(2, 
            f"移动状态: 路径点 {self.current_waypoint_index}, "
            f"已移动={distance_traveled:.2f}m, 剩余={remaining_distance:.2f}m")
        
        # 检查是否到达目标距离
        if remaining_distance <= self.position_tolerance:
            self.stop_car()
            rospy.sleep(0.2)  # 短暂停止
            
            rospy.loginfo(f"✓ 到达路径点 {self.current_waypoint_index}")
            
            # 移动到下一个路径点
            self.current_waypoint_index += 1
            
            if self.current_waypoint_index < len(self.waypoints):
                # 如果是最后一个点，可能需要调整最终朝向
                if self.current_waypoint_index == len(self.waypoints) - 1:
                    self.control_state = "ADJUSTING_FINAL"
                    rospy.loginfo("开始调整最终朝向")
                else:
                    self.control_state = "TURNING"
                    rospy.loginfo(f"开始导航到路径点 {self.current_waypoint_index}")
            else:
                self.control_active = False
                rospy.loginfo("🎉 导航任务全部完成！")
            return
        
        # 前进控制 - 只前进，不转向
        self.send_control_command('A')  # 前进
    
    def handle_adjusting_final_state(self):
        """处理最终朝向调整状态"""
        if self.current_waypoint_index >= len(self.waypoints):
            return
            
        final_waypoint = self.waypoints[self.current_waypoint_index]
        target_yaw = final_waypoint['yaw']
        angle_error = self.calculate_angle_error(target_yaw)
        
        rospy.loginfo_throttle(2, 
            f"最终朝向调整: 角度误差={math.degrees(angle_error):.1f}°")
        
        if abs(angle_error) < self.final_angle_tolerance:
            self.stop_car()
            self.control_active = False
            rospy.loginfo("🎉 最终朝向调整完成！导航任务全部完成！")
            return
        
        # 最终朝向调整
        if angle_error > 0:
            self.send_control_command('D')  # 左转
        else:
            self.send_control_command('C')  # 右转
    
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
        navigator = StablePathNavigator()
        navigator.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"程序异常: {e}")
    finally:
        rospy.loginfo("程序结束")
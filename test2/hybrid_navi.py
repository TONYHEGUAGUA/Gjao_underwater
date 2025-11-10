#!/usr/bin/env python3
import rospy
import math
import serial
import threading
import tf
import sys
import select
import tty
import termios
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

class HybridNavigator:
    def __init__(self):
        rospy.init_node('hybrid_navigator', anonymous=True)
        
        # IMU数据相关变量
        self.current_imu_yaw = 0.0
        self.last_imu_time = 0
        self.imu_initialized = False
        self.imu_initial_yaw = 0.0
        
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
        
        # 控制模式
        self.control_mode = "AUTO"  # AUTO: 自动导航, MANUAL: 手动控制
        
        # 自动导航相关变量
        self.current_pose = None
        self.control_active = False
        self.target_reached = False
        self.position_tolerance = 0.08
        self.angle_tolerance = 0.08
        
        # 目标点导航相关变量
        self.target_point = None
        self.target_yaw = 0.0
        self.start_pose = None
        self.target_distance = 0.0
        self.turned_to_target = False
        
        # 手动控制命令映射
        self.command_map = {
            'w': 'A',  # 前进
            's': 'B',  # 后退
            'a': 'D',  # 左转
            'd': 'C',  # 右转
            'x': '0',  # 停止
            'm': 'toggle_mode',  # 切换模式
            'q': 'quit' # 退出
        }
        
        # TF监听器
        self.tf_listener = tf.TransformListener()
        
        # 订阅话题
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        
        # 控制循环
        self.control_timer = rospy.Timer(rospy.Duration(0.02), self.control_loop)
        
        # 启动键盘监听线程
        self.keyboard_running = True
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()
        
        rospy.loginfo("混合导航系统初始化完成")
        rospy.loginfo(f"当前模式: {self.control_mode}")
        rospy.loginfo("手动控制命令: W(前进), S(后退), A(左转), D(右转), X(停止), M(切换模式), Q(退出)")
        rospy.loginfo("自动模式: 等待Foxglove发送目标点...")
    
    def toggle_control_mode(self):
        """切换控制模式"""
        if self.control_mode == "AUTO":
            self.control_mode = "MANUAL"
            self.control_active = False  # 停止自动导航
            self.stop_car()
            rospy.loginfo("切换到手动控制模式")
        else:
            self.control_mode = "AUTO"
            self.stop_car()
            rospy.loginfo("切换到自动导航模式")
    
    def goal_callback(self, msg):
        """处理Foxglove发送的目标点"""
        if self.control_mode != "AUTO":
            rospy.logwarn("当前为手动模式，忽略目标点。按 M 切换到自动模式")
            return
            
        if self.current_pose is None or not self.imu_initialized:
            rospy.logwarn("无法开始导航: 等待定位和IMU数据初始化")
            return
            
        self.stop_current_mission()
        rospy.sleep(0.5)
        
        # 提取目标位置
        target_x = msg.pose.position.x
        target_y = msg.pose.position.y
        
        # 提取目标朝向
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
        """停止当前自动导航任务"""
        self.control_active = False
        self.target_reached = False
        self.turned_to_target = False
        self.stop_car()
        rospy.loginfo("自动导航任务已停止")
    
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
                                    self.imu_initial_yaw = self.current_imu_yaw
                                    self.imu_initialized = True
                                    rospy.loginfo(f"IMU初始化完成，初始偏航角: {math.degrees(self.imu_initial_yaw):.2f}°")
                
                rospy.sleep(0.001)
                
            except Exception as e:
                rospy.logwarn(f"读取IMU数据错误: {e}")
                rospy.sleep(0.1)
    
    def get_current_yaw(self):
        """获取当前偏航角"""
        if self.imu_initialized:
            return self.current_imu_yaw
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
        """计算角度误差"""
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
        rospy.logdebug("停止小车")
    
    def get_key(self):
        """获取键盘输入（非阻塞）"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            [i, o, e] = select.select([sys.stdin], [], [], 0.1)
            if i:
                key = sys.stdin.read(1)
                return key.lower()
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return None
    
    def keyboard_listener(self):
        """键盘监听线程"""
        while self.keyboard_running and not rospy.is_shutdown():
            key = self.get_key()
            if key:
                self.handle_keyboard_input(key)
            rospy.sleep(0.01)
    
    def handle_keyboard_input(self, key):
        """处理键盘输入"""
        if key in self.command_map:
            command = self.command_map[key]
            
            if command == 'quit':
                rospy.loginfo("收到退出命令")
                rospy.signal_shutdown("用户退出")
                return
            elif command == 'toggle_mode':
                self.toggle_control_mode()
                return
            
            # 手动控制命令 - 立即执行，停止自动导航
            if self.control_mode == "AUTO":
                rospy.loginfo("手动控制指令触发，切换到手动模式")
                self.control_mode = "MANUAL"
                self.control_active = False
            
            self.send_control_command(command)
            rospy.loginfo(f"手动控制: {key.upper()} -> {command}")
    
    def control_loop(self, event):
        """控制循环"""
        # 更新当前位置
        self.current_pose = self.get_robot_pose()
        
        # 自动导航控制
        if self.control_mode == "AUTO" and self.control_active:
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
    
    def display_status(self):
        """显示状态信息"""
        if self.current_pose is not None:
            x = self.current_pose.position.x
            y = self.current_pose.position.y
            yaw_deg = math.degrees(self.get_current_yaw())
            
            status = f"模式: {self.control_mode} | 位置: ({x:.3f}, {y:.3f}) | 角度: {yaw_deg:.2f}°"
            
            if self.control_mode == "AUTO" and self.control_active:
                if self.target_point:
                    status += f" | 导航中: ({self.target_point[0]:.2f}, {self.target_point[1]:.2f})"
            
            print(f"\r{status}", end='', flush=True)
    
    def run(self):
        """主循环"""
        rate = rospy.Rate(10)  # 10Hz状态显示
        
        print("\n=== 混合导航系统 ===")
        print("手动控制命令: W(前进), S(后退), A(左转), D(右转), X(停止), M(切换模式), Q(退出)")
        print("=" * 50)
        
        try:
            while not rospy.is_shutdown():
                self.display_status()
                rate.sleep()
        except KeyboardInterrupt:
            rospy.loginfo("用户中断")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """清理资源"""
        self.keyboard_running = False
        self.imu_running = False
        self.control_active = False
        self.stop_car()
        try:
            self.control_timer.shutdown()
            self.ser.write('0'.encode())
            self.ser.close()
            rospy.loginfo("资源清理完成")
        except:
            pass

if __name__ == '__main__':
    try:
        navigator = HybridNavigator()
        navigator.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"程序异常: {e}")
    finally:
        rospy.loginfo("程序结束")
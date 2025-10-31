#!/usr/bin/env python
import rospy
import math
import serial
import threading
import sys
import select
import tty
import termios
from geometry_msgs.msg import PoseStamped

class CarManualControl:
    def __init__(self):
        rospy.init_node('car_manual_control', anonymous=True)
        
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
        
        # 控制命令映射
        self.command_map = {
            'w': 'A',  # 前进
            's': 'B',  # 后退
            'a': 'D',  # 左转
            'd': 'C',  # 右转
            'x': '0',  # 停止
            'q': 'quit' # 退出
        }
        
        # 订阅slam_out_pose话题（用于显示位置信息）
        rospy.Subscriber('/slam_out_pose', PoseStamped, self.pose_callback)
        self.current_pose = None
        
        rospy.loginfo("手动控制节点初始化完成")
        rospy.loginfo("使用 WASD 控制移动，X 停止，Q 退出")
        rospy.loginfo("W:前进, S:后退, A:左转, D:右转, X:停止")
        
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
    
    def send_control_command(self, command):
        """发送控制命令"""
        try:
            self.ser.write(command.encode())
            rospy.sleep(0.1)
            self.ser.write('!'.encode())
            rospy.loginfo(f"执行命令: {command}")
        except Exception as e:
            rospy.logerr(f"发送命令失败: {e}")
    
    def stop_car(self):
        """停止小车"""
        self.send_control_command('0')
        rospy.loginfo("停止小车")
    
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
    
    def display_status(self):
        """显示当前状态"""
        if self.current_pose is not None:
            x = self.current_pose.position.x
            y = self.current_pose.position.y
            yaw_deg = math.degrees(self.get_current_yaw())
            print(f"\r位置: ({x:.3f}, {y:.3f}) | 角度: {yaw_deg:.2f}° | 等待命令...", end='', flush=True)
        else:
            yaw_deg = math.degrees(self.get_current_yaw())
            print(f"\r角度: {yaw_deg:.2f}° | 等待命令...", end='', flush=True)
    
    def run_manual_control(self):
        """运行手动控制"""
        rate = rospy.Rate(10)  # 10Hz
        
        print("\n=== 机器人手动控制 ===")
        print("W:前进, S:后退, A:左转, D:右转, X:停止, Q:退出")
        print("=" * 30)
        
        while not rospy.is_shutdown():
            # 显示状态
            self.display_status()
            
            # 获取键盘输入
            key = self.get_key()
            
            if key:
                if key in self.command_map:
                    command = self.command_map[key]
                    
                    if command == 'quit':
                        print("\n退出手动控制")
                        break
                    else:
                        # 发送控制命令
                        self.send_control_command(command)
                        print(f" -> 执行: {key.upper()}")
                else:
                    print(f"\n未知命令: {key}")
                    print("可用命令: W(前进), S(后退), A(左转), D(右转), X(停止), Q(退出)")
            
            rate.sleep()
    
    def run(self):
        """主函数"""
        try:
            self.run_manual_control()
        except KeyboardInterrupt:
            print("\n用户中断")
        finally:
            # 清理工作
            self.cleanup()
    
    def cleanup(self):
        """清理资源"""
        rospy.loginfo("正在清理资源...")
        self.imu_running = False
        self.stop_car()
        try:
            self.ser.write('0'.encode())
            self.ser.close()
            rospy.loginfo("串口已关闭")
        except:
            pass

if __name__ == '__main__':
    try:
        controller = CarManualControl()
        controller.run()
    except rospy.ROSInterruptException:
        pass
#!/usr/bin/env python3
import rospy
import math
import serial
import threading
import sys
import select
import tty
import termios

class PureManualControl:
    def __init__(self):
        rospy.init_node('pure_manual_control', anonymous=True)
        
        # IMU数据相关变量
        self.current_imu_yaw = 0.0
        self.last_imu_time = 0
        self.imu_initialized = False
        
        # PWM控制参数 - 可以根据实际调整这些值
        self.PWM_NEUTRAL = 1500  # 中立/停止 PWM值
        self.PWM_MIN = 1000      # 最小PWM值
        self.PWM_MAX = 2000      # 最大PWM值
        self.pwm_drive_diff = 100  # 前进/后退的PWM变化量 (油门)
        self.pwm_turn_diff = 100   # 转向的PWM变化量 (方向灵敏度)
        self.PWM_ADJUST_STEP = 10  # 每次调整的步长
        
        # 方向修正标志 - 如果方向反了，可以调整这些值
        self.REVERSE_LEFT_MOTOR = False  # 左电机是否反转
        self.REVERSE_RIGHT_MOTOR = False  # 右电机是否反转
        
        # 控制状态
        self.current_left_pwm = self.PWM_NEUTRAL
        self.current_right_pwm = self.PWM_NEUTRAL
        self.last_command_time = rospy.get_time()
        self.command_timeout = 0.5  # 命令超时时间（秒）
        
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
            'w': 'forward',   # 前进
            's': 'backward',  # 后退
            'a': 'left',      # 左转
            'd': 'right',     # 右转
            'x': 'stop',      # 停止
            '+': 'inc_speed', # 增加速度
            '-': 'dec_speed', # 减少速度
            ']': 'inc_turn',  # 增加转向
            '[': 'dec_turn',  # 减少转向
            'q': 'quit'       # 退出
        }
        
        rospy.loginfo("纯手动控制节点初始化完成")
        rospy.loginfo("使用 WASD 控制移动，X 停止，Q 退出")
        rospy.loginfo(f"当前速度: {self.pwm_drive_diff}, 转向: {self.pwm_turn_diff}")
    
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
                                    rospy.loginfo(f"IMU初始化完成")
                
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
        # 发送停止命令初始化
        self.send_pwm_command(self.PWM_NEUTRAL, self.PWM_NEUTRAL)
        rospy.sleep(0.5)
        
        # 开启负压吸附（如果还需要）
        # self.ser.write('!'.encode())
        rospy.loginfo("初始化完成，准备接收控制命令")
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
            
            # 格式化命令：M,left_pwm,right_pwm\n
            command = f"M,{left_pwm},{right_pwm}\n"
            self.ser.write(command.encode())
            
            self.current_left_pwm = left_pwm
            self.current_right_pwm = right_pwm
            self.last_command_time = rospy.get_time()
            
            rospy.logdebug(f"发送PWM命令: {command.strip()}")
        except Exception as e:
            rospy.logerr(f"发送PWM命令失败: {e}")
    
    def stop_car(self):
        """停止小车"""
        self.send_pwm_command(self.PWM_NEUTRAL, self.PWM_NEUTRAL)
        rospy.loginfo("停止小车")
    
    def check_timeout(self):
        """检查命令是否超时，超时则停止"""
        current_time = rospy.get_time()
        if current_time - self.last_command_time > self.command_timeout:
            if self.current_left_pwm != self.PWM_NEUTRAL or self.current_right_pwm != self.PWM_NEUTRAL:
                self.stop_car()
                return True
        return False
    
    def calculate_pwm_for_movement(self, direction, turn=None):
        """根据方向和转向计算PWM值"""
        # 基础驱动偏移
        drive_offset = 0
        if direction == 'forward':
            drive_offset = self.pwm_drive_diff
        elif direction == 'backward':
            drive_offset = -self.pwm_drive_diff
        
        # 转向偏移 - 修正转向逻辑
        turn_offset = 0
        if turn == 'left':
            # 左转：左轮减速，右轮加速
            turn_offset = -self.pwm_turn_diff
        elif turn == 'right':
            # 右转：左轮加速，右轮减速
            turn_offset = self.pwm_turn_diff
        
        # 计算左右轮PWM值
        # 前进/后退 + 转向的组合
        left_pwm = self.PWM_NEUTRAL + drive_offset + turn_offset
        right_pwm = self.PWM_NEUTRAL + drive_offset - turn_offset
        
        return left_pwm, right_pwm
    
    def calculate_pwm_for_movement_v2(self, direction, turn=None):
        """另一种差速转向计算方法（如果上面的不正确）"""
        # 基础PWM值
        left_pwm = self.PWM_NEUTRAL
        right_pwm = self.PWM_NEUTRAL
        
        if direction == 'forward':
            left_pwm += self.pwm_drive_diff
            right_pwm += self.pwm_drive_diff
        elif direction == 'backward':
            left_pwm -= self.pwm_drive_diff
            right_pwm -= self.pwm_drive_diff
        
        if turn == 'left':
            # 左转：减小左轮，增加右轮
            left_pwm -= self.pwm_turn_diff
            right_pwm += self.pwm_turn_diff
        elif turn == 'right':
            # 右转：增加左轮，减小右轮
            left_pwm += self.pwm_turn_diff
            right_pwm -= self.pwm_turn_diff
        
        return left_pwm, right_pwm
    
    def calculate_pwm_for_movement_v3(self, direction, turn=None):
        """第三种方法：如果前进后退也反了"""
        # 基础驱动偏移
        drive_offset = 0
        if direction == 'forward':
            drive_offset = -self.pwm_drive_diff  # 符号相反
        elif direction == 'backward':
            drive_offset = self.pwm_drive_diff   # 符号相反
        
        # 转向偏移
        turn_offset = 0
        if turn == 'left':
            turn_offset = self.pwm_turn_diff  # 符号相反
        elif turn == 'right':
            turn_offset = -self.pwm_turn_diff  # 符号相反
        
        # 计算左右轮PWM值
        left_pwm = self.PWM_NEUTRAL + drive_offset + turn_offset
        right_pwm = self.PWM_NEUTRAL + drive_offset - turn_offset
        
        return left_pwm, right_pwm
    
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
        yaw_deg = math.degrees(self.get_current_yaw())
        print(f"\r角度: {yaw_deg:6.2f}° | 左轮: {self.current_left_pwm:4d} | 右轮: {self.current_right_pwm:4d} | 速度: {self.pwm_drive_diff:3d} | 转向: {self.pwm_turn_diff:3d}", end='', flush=True)
    
    def run_manual_control(self):
        """运行手动控制"""
        rate = rospy.Rate(20)  # 20Hz，更高的更新频率
        
        print("\n" + "="*80)
        print("                         纯手动控制模式 (PWM控制)")
        print("="*80)
        print("移动控制:")
        print("  W: 前进          S: 后退          A: 左转          D: 右转")
        print("  X: 停止          Q: 退出程序")
        print("\n参数调整:")
        print("  +: 增加速度      -: 减少速度      ]: 增加转向      [: 减少转向")
        print("\n调试选项:")
        print("  1: 前进测试      2: 后退测试      3: 左转测试      4: 右转测试")
        print("  r: 重置方向设置  t: 切换转向模型")
        print("="*80)
        
        # 存储当前按下的键
        current_keys = {
            'move': None,  # 移动方向: forward/backward/None
            'turn': None   # 转向方向: left/right/None
        }
        
        # 选择转向模型
        steering_model = 1  # 1, 2, 3 对应不同的计算方法
        
        while not rospy.is_shutdown():
            # 显示状态
            self.display_status()
            
            # 检查超时
            self.check_timeout()
            
            # 获取键盘输入
            key = self.get_key()
            
            if key:
                # 调试命令
                if key == '1':
                    print("\n -> 前进测试")
                    self.send_pwm_command(self.PWM_NEUTRAL + self.pwm_drive_diff, 
                                         self.PWM_NEUTRAL + self.pwm_drive_diff)
                    continue
                elif key == '2':
                    print("\n -> 后退测试")
                    self.send_pwm_command(self.PWM_NEUTRAL - self.pwm_drive_diff, 
                                         self.PWM_NEUTRAL - self.pwm_drive_diff)
                    continue
                elif key == '3':
                    print("\n -> 左转测试")
                    self.send_pwm_command(self.PWM_NEUTRAL - self.pwm_turn_diff, 
                                         self.PWM_NEUTRAL + self.pwm_turn_diff)
                    continue
                elif key == '4':
                    print("\n -> 右转测试")
                    self.send_pwm_command(self.PWM_NEUTRAL + self.pwm_turn_diff, 
                                         self.PWM_NEUTRAL - self.pwm_turn_diff)
                    continue
                elif key == 'r':
                    print("\n -> 重置方向设置")
                    self.REVERSE_LEFT_MOTOR = not self.REVERSE_LEFT_MOTOR
                    self.REVERSE_RIGHT_MOTOR = not self.REVERSE_RIGHT_MOTOR
                    print(f"左电机反转: {self.REVERSE_LEFT_MOTOR}, 右电机反转: {self.REVERSE_RIGHT_MOTOR}")
                    continue
                elif key == 't':
                    steering_model = steering_model % 3 + 1
                    print(f"\n -> 切换到转向模型 {steering_model}")
                    continue
                
                # 正常控制命令
                if key in self.command_map:
                    command = self.command_map[key]
                    
                    if command == 'quit':
                        print("\n退出程序")
                        break
                    elif command == 'stop':
                        current_keys['move'] = None
                        current_keys['turn'] = None
                        self.stop_car()
                        print(f"\n -> 停止")
                    elif command == 'inc_speed':
                        self.pwm_drive_diff = min(500, self.pwm_drive_diff + self.PWM_ADJUST_STEP)
                        print(f"\n -> 速度增加: {self.pwm_drive_diff}")
                    elif command == 'dec_speed':
                        self.pwm_drive_diff = max(0, self.pwm_drive_diff - self.PWM_ADJUST_STEP)
                        print(f"\n -> 速度减少: {self.pwm_drive_diff}")
                    elif command == 'inc_turn':
                        self.pwm_turn_diff = min(500, self.pwm_turn_diff + self.PWM_ADJUST_STEP)
                        print(f"\n -> 转向增加: {self.pwm_turn_diff}")
                    elif command == 'dec_turn':
                        self.pwm_turn_diff = max(0, self.pwm_turn_diff - self.PWM_ADJUST_STEP)
                        print(f"\n -> 转向减少: {self.pwm_turn_diff}")
                    elif command in ['forward', 'backward']:
                        current_keys['move'] = command
                        print(f"\n -> {command}")
                    elif command in ['left', 'right']:
                        current_keys['turn'] = command
                        print(f"\n -> {command}")
                
                # 处理组合键释放（当按下停止或相反方向时）
                if key == 'x':
                    current_keys['move'] = None
                    current_keys['turn'] = None
                elif key == 'w' and current_keys['move'] == 'backward':
                    current_keys['move'] = 'forward'
                elif key == 's' and current_keys['move'] == 'forward':
                    current_keys['move'] = 'backward'
                elif key == 'a' and current_keys['turn'] == 'right':
                    current_keys['turn'] = 'left'
                elif key == 'd' and current_keys['turn'] == 'left':
                    current_keys['turn'] = 'right'
            
            # 根据当前按下的键计算PWM值并发送
            if current_keys['move'] or current_keys['turn']:
                if steering_model == 1:
                    left_pwm, right_pwm = self.calculate_pwm_for_movement(
                        current_keys['move'], current_keys['turn']
                    )
                elif steering_model == 2:
                    left_pwm, right_pwm = self.calculate_pwm_for_movement_v2(
                        current_keys['move'], current_keys['turn']
                    )
                else:
                    left_pwm, right_pwm = self.calculate_pwm_for_movement_v3(
                        current_keys['move'], current_keys['turn']
                    )
                
                self.send_pwm_command(left_pwm, right_pwm)
            
            rate.sleep()
    
    def run(self):
        """主函数"""
        try:
            self.run_manual_control()
        except KeyboardInterrupt:
            print("\n用户中断")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """清理资源"""
        rospy.loginfo("正在清理资源...")
        self.imu_running = False
        self.stop_car()
        try:
            self.ser.close()
            rospy.loginfo("串口已关闭")
        except:
            pass

if __name__ == '__main__':
    try:
        controller = PureManualControl()
        controller.run()
    except rospy.ROSInterruptException:
        pass
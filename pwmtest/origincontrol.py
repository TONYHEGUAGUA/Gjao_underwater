import serial
import time
import threading
import keyboard
import sys

class PWMTester:
    def __init__(self, port='/dev/ttyUSB1', baudrate=115200):
        # --- 可配置参数 ---
        self.PWM_NEUTRAL = 1500  # 中立/停止 PWM值
        self.PWM_MIN = 1000      # 最小PWM值
        self.PWM_MAX = 2000      # 最大PWM值

        # --- 运行时可调参数 ---
        self.pwm_drive_diff = 100 # 当前前进/后退的PWM变化量 (油门)
        self.pwm_turn_diff = 100  # 当前转向的PWM变化量 (方向灵敏度)
        self.PWM_ADJUST_STEP = 10 # 每次按+/-键调整的步长

        # --- 串口和线程设置 ---
        self.ser = None
        self.port = port
        self.baudrate = baudrate
        self.connected = False
        self.running = False
        self.lock = threading.Lock()
        self.last_sent_command = ""
        self.last_action_key = ''

    def connect(self):
        """连接到串口"""
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            print("正在连接，等待STM32设备响应...")
            time.sleep(2) # 等待STM32重启完成
            self.connected = True
            self.running = True
            print(f"连接到 {self.port} 成功!")
            self.read_thread = threading.Thread(target=self._read_serial)
            self.read_thread.daemon = True
            self.read_thread.start()
            return True
        except serial.SerialException as e:
            print(f"连接失败: {e}")
            print(f"提示: 请检查设备管理器，确认无人船的串口号是否为 {self.port}。")
            return False

    def disconnect(self):
        """断开连接并发送最终的停止指令"""
        self.running = False
        if self.ser and self.ser.is_open:
            try:
                stop_command = f"M,{self.PWM_NEUTRAL},{self.PWM_NEUTRAL}\n"
                with self.lock:
                    self.ser.write(stop_command.encode())
                print(f"\n已发送停止命令: {stop_command.strip()}")
            except Exception as e:
                print(f"发送停止命令失败: {e}")
            self.ser.close()
        self.connected = False
        print("连接已断开。")

    def _read_serial(self):
        """后台线程，用于读取并打印来自STM32的调试信息"""
        print("\n--- 来自无人船的实时反馈 ---")
        while self.running and self.ser.is_open:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    print(f"<<< {line}")
            except (serial.SerialException, TypeError):
                # 串口关闭时可能会抛出异常，安全退出
                break
        print("--- 反馈线程已停止 ---")

    def send_pwm_command(self, left_pwm, right_pwm):
        """格式化PWM指令并发送到串口"""
        if not self.connected or not self.running:
            return
        
        command = f"M,{left_pwm},{right_pwm}\n"
        # 只有在指令变化时才发送，避免刷屏STM32的串口缓冲区
        if command != self.last_sent_command:
            try:
                with self.lock:
                    self.ser.write(command.encode())
                self.last_sent_command = command
            except serial.SerialException:
                print("写入串口失败，可能已断开连接。")
                self.running = False

    def run_control_loop(self):
        """主控制循环，处理键盘输入并计算PWM值"""
        print("\n" + "="*60)
        print("        无人船PWM实时键盘测试脚本 (PC版 V3.0)")
        print("="*60)
        print(f"油门: {self.pwm_drive_diff} | 转向: {self.pwm_turn_diff}")
        print("------------------------------------------------------------")
        print("移动: W/S (前进/后退) | A/D (左/右转向)")
        print("调速: +/- (增/减油门) | [/] (增/减转向灵敏度)")
        print("急停: Space (所有PWM设为1500)")
        print("退出: ESC")
        print("="*60)
        
        try:
            while self.running:
                # --- 1. 计算期望的PWM值 ---
                drive_offset = 0
                turn_offset = 0

                # 检测持续按下的移动键
                if keyboard.is_pressed('w'):
                    drive_offset = self.pwm_drive_diff
                elif keyboard.is_pressed('s'):
                    drive_offset = -self.pwm_drive_diff
                
                if keyboard.is_pressed('a'):
                    turn_offset = -self.pwm_turn_diff # 左转：左轮慢，右轮快
                elif keyboard.is_pressed('d'):
                    turn_offset = self.pwm_turn_diff  # 右转：左轮快，右轮慢
                
                if keyboard.is_pressed('space'):
                    drive_offset = 0
                    turn_offset = 0

                # 根据差速模型计算左右轮最终PWM
                left_pwm = self.PWM_NEUTRAL + drive_offset + turn_offset
                right_pwm = self.PWM_NEUTRAL + drive_offset - turn_offset

                # 限制在安全范围内
                left_pwm = max(self.PWM_MIN, min(self.PWM_MAX, left_pwm))
                right_pwm = max(self.PWM_MIN, min(self.PWM_MAX, right_pwm))

                self.send_pwm_command(int(left_pwm), int(right_pwm))

                # --- 2. 处理只需要按一下的调速命令 ---
                action_map = {'=': '+', '+': '+', '-': '-', '[': '[', ']': ']'}
                current_action_key = ''
                for key in action_map:
                    if keyboard.is_pressed(key):
                        current_action_key = key
                        break
                
                if current_action_key and current_action_key != self.last_action_key:
                    if current_action_key in ['=', '+']:
                        self.pwm_drive_diff = min(500, self.pwm_drive_diff + self.PWM_ADJUST_STEP)
                        print(f"油门增加: {self.pwm_drive_diff}")
                    elif current_action_key == '-':
                        self.pwm_drive_diff = max(0, self.pwm_drive_diff - self.PWM_ADJUST_STEP)
                        print(f"油门减少: {self.pwm_drive_diff}")
                    elif current_action_key == ']':
                        self.pwm_turn_diff = min(500, self.pwm_turn_diff + self.PWM_ADJUST_STEP)
                        print(f"转向增加: {self.pwm_turn_diff}")
                    elif current_action_key == '[':
                        self.pwm_turn_diff = max(0, self.pwm_turn_diff - self.PWM_ADJUST_STEP)
                        print(f"转向减少: {self.pwm_turn_diff}")
                
                self.last_action_key = current_action_key

                # --- 3. 处理退出 ---
                if keyboard.is_pressed('esc'):
                    self.running = False
                    break
                
                # 控制循环频率，约50Hz
                time.sleep(0.02)

        except KeyboardInterrupt:
            print("\n检测到 Ctrl+C...")
        finally:
            self.disconnect()

if __name__ == "__main__":
    # 交互式输入串口号，默认为/dev/ttyUSB1
    port_input = input(f"请输入串口号 (默认: /dev/ttyUSB1): ").strip()
    port = port_input or '/dev/ttyUSB1'
    
    tester = PWMTester(port=port)
    
    if tester.connect():
        tester.run_control_loop()
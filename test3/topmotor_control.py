#!/usr/bin/env python3
import serial
import time
import sys
import argparse

class TopMotorController:
    def __init__(self, port='/dev/ttyUSB1', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        
    def connect(self):
        """连接串口"""
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            print(f"成功连接到串口 {self.port}")
            return True
        except Exception as e:
            print(f"无法打开串口 {self.port}: {e}")
            return False
    
    def disconnect(self):
        """断开串口连接"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("串口已关闭")
    
    def send_command(self, command):
        """发送命令到串口"""
        try:
            if self.ser and self.ser.is_open:
                self.ser.write(command.encode())
                print(f"发送命令: '{command}'")
                time.sleep(0.5)  # 等待命令执行
                return True
            else:
                print("串口未连接")
                return False
        except Exception as e:
            print(f"发送命令失败: {e}")
            return False
    
    def turn_on(self):
        """开启负压吸附"""
        print("正在开启负压吸附...")
        if self.send_command('!'):
            print("✓ 负压吸附已开启")
            return True
        return False
    
    def turn_off(self):
        """关闭负压吸附"""
        print("正在关闭负压吸附...")
        if self.send_command('%'):
            print("✓ 负压吸附已关闭")
            return True
        return False
    
    def set_speed(self):
        """设置速度（初始化）"""
        print("正在设置速度...")
        # 发送9个3减速到10
        for i in range(9):
            if not self.send_command('3'):
                return False
        # 发送6个6进一步减速
        for i in range(6):
            if not self.send_command('6'):
                return False
        print("✓ 速度设置完成")
        return True

def main():
    parser = argparse.ArgumentParser(description='负压吸附控制器')
    parser.add_argument('action', choices=['on', 'off', 'toggle'], 
                       help='控制动作: on(开启), off(关闭), toggle(切换)')
    parser.add_argument('--port', '-p', default='/dev/ttyUSB1',
                       help='串口设备路径 (默认: /dev/ttyUSB1)')
    parser.add_argument('--baudrate', '-b', type=int, default=115200,
                       help='波特率 (默认: 115200)')
    parser.add_argument('--init-speed', action='store_true',
                       help='初始化速度设置')
    
    args = parser.parse_args()
    
    # 创建控制器实例
    controller = TopMotorController(port=args.port, baudrate=args.baudrate)
    
    # 连接串口
    if not controller.connect():
        sys.exit(1)
    
    try:
        # 如果需要初始化速度
        if args.init_speed:
            if not controller.set_speed():
                print("速度设置失败")
                sys.exit(1)
        
        # 执行控制动作
        success = False
        if args.action == 'on':
            success = controller.turn_on()
        elif args.action == 'off':
            success = controller.turn_off()
        elif args.action == 'toggle':
            # 这里可以添加状态检测逻辑，目前简单实现为开启
            print("切换模式 - 默认开启负压吸附")
            success = controller.turn_on()
        
        if success:
            print("操作成功完成")
        else:
            print("操作失败")
            sys.exit(1)
            
    except KeyboardInterrupt:
        print("\n用户中断操作")
    finally:
        controller.disconnect()

if __name__ == '__main__':
    main()
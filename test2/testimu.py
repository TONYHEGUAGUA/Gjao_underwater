#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import serial
import time
import threading

class IMUReader:
    def __init__(self, port='/dev/ttyUSB1', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.running = False
        self.thread = None
        self.current_yaw = 0.0
        self.last_valid_time = 0
        
    def calculate_checksum(self, data):
        """计算XOR校验和"""
        checksum = 0
        for byte in data.encode('ascii'):
            checksum ^= byte
        return checksum
    
    def verify_checksum(self, payload, received_checksum):
        """验证校验和"""
        calculated_checksum = self.calculate_checksum(payload)
        return calculated_checksum == int(received_checksum, 16)
    
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
                print(f"校验和错误: 计算值={self.calculate_checksum(payload):02X}, 接收值={checksum_str}")
                return None
            
            # 解析yaw值
            yaw = float(yaw_str)
            
            return {
                'type': 'IMU',
                'yaw': yaw,
                'checksum': checksum_str,
                'timestamp': time.time()
            }
            
        except ValueError as e:
            print(f"数据解析错误: {e}, 帧: {frame}")
            return None
        except Exception as e:
            print(f"解析异常: {e}, 帧: {frame}")
            return None
    
    def read_serial_data(self):
        """读取串口数据"""
        buffer = ""
        while self.running:
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
                                self.current_yaw = imu_data['yaw']
                                self.last_valid_time = time.time()
                                
                                # 实时打印数据
                                print(f"IMU数据 - Yaw: {imu_data['yaw']:>7.2f}°, "
                                      f"校验和: {imu_data['checksum']}, "
                                      f"时间: {time.strftime('%H:%M:%S')}")
                
                # 短暂休眠以减少CPU占用
                time.sleep(0.001)
                
            except Exception as e:
                print(f"读取串口数据错误: {e}")
                time.sleep(0.1)
    
    def start(self):
        """启动IMU读取"""
        try:
            print(f"正在连接串口 {self.port}...")
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False
            )
            
            if self.ser.is_open:
                print(f"成功连接到 {self.port}, 波特率 {self.baudrate}")
                
                # 清空缓冲区
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
                
                # 启动读取线程
                self.running = True
                self.thread = threading.Thread(target=self.read_serial_data)
                self.thread.daemon = True
                self.thread.start()
                
                print("IMU数据读取已启动，按 Ctrl+C 停止")
                return True
            else:
                print("无法打开串口")
                return False
                
        except serial.SerialException as e:
            print(f"串口连接失败: {e}")
            return False
        except Exception as e:
            print(f"启动失败: {e}")
            return False
    
    def stop(self):
        """停止IMU读取"""
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=2)
        
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("串口已关闭")
    
    def get_current_yaw(self):
        """获取当前yaw值"""
        return self.current_yaw
    
    def is_data_fresh(self, timeout=1.0):
        """检查数据是否新鲜"""
        return (time.time() - self.last_valid_time) < timeout

def main():
    imu_reader = IMUReader('/dev/ttyUSB1', 115200)
    
    try:
        if imu_reader.start():
            # 主循环，可以在这里添加其他处理逻辑
            while True:
                # 检查数据是否新鲜
                if not imu_reader.is_data_fresh():
                    print("警告: IMU数据已超时")
                
                time.sleep(1)
                
    except KeyboardInterrupt:
        print("\n正在停止IMU读取...")
    finally:
        imu_reader.stop()

if __name__ == "__main__":
    main()
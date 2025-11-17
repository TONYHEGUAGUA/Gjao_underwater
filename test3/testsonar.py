#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import serial
import time
import threading

class SonarReader:
    def __init__(self, port='/dev/ttyUSB1', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.running = False
        self.thread = None
        self.current_sonar_data = {
            'U2': 0,
            'U4': 0, 
            'U5': 0,
            'U6': 0
        }
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
    
    def parse_sonar_frame(self, frame):
        """解析声纳数据帧"""
        try:
            # 移除首尾空白字符
            frame = frame.strip()
            
            # 检查帧格式
            if not frame.startswith('$SONAR,'):
                return None
                
            # 分割字段
            parts = frame[1:].split(',')  # 去掉$号后分割
            if len(parts) < 6:  # SONAR + 4个传感器 + 校验和
                return None
                
            frame_type = parts[0]
            checksum_str = parts[-1]  # 最后一个字段是校验和
            sensor_data_parts = parts[1:-1]  # 中间是传感器数据
            
            # 验证帧类型
            if frame_type != 'SONAR':
                return None
                
            # 验证校验和
            payload = f"SONAR,{','.join(sensor_data_parts)}"
            if not self.verify_checksum(payload, checksum_str):
                print(f"校验和错误: 计算值={self.calculate_checksum(payload):02X}, 接收值={checksum_str}")
                return None
            
            # 解析传感器数据
            sonar_data = {}
            for sensor_part in sensor_data_parts:
                if ':' in sensor_part:
                    sensor_name, distance_str = sensor_part.split(':')
                    try:
                        distance = int(distance_str)
                        sonar_data[sensor_name] = distance
                    except ValueError:
                        print(f"距离值解析错误: {sensor_part}")
                        continue
            
            return {
                'type': 'SONAR',
                'sensors': sonar_data,
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
                            # 尝试解析声纳数据
                            sonar_data = self.parse_sonar_frame(frame)
                            if sonar_data:
                                # 更新当前声纳数据
                                self.current_sonar_data.update(sonar_data['sensors'])
                                self.last_valid_time = time.time()
                                
                                # 实时打印数据
                                sensor_info = []
                                for sensor, distance in sonar_data['sensors'].items():
                                    sensor_info.append(f"{sensor}:{distance:4d}mm")
                                
                                print(f"声纳数据 - {', '.join(sensor_info)}, "
                                      f"校验和: {sonar_data['checksum']}, "
                                      f"时间: {time.strftime('%H:%M:%S')}")
                
                # 短暂休眠以减少CPU占用
                time.sleep(0.001)
                
            except Exception as e:
                print(f"读取串口数据错误: {e}")
                time.sleep(0.1)
    
    def start(self):
        """启动声纳读取"""
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
                
                print("声纳数据读取已启动，按 Ctrl+C 停止")
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
        """停止声纳读取"""
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=2)
        
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("串口已关闭")
    
    def get_current_sonar_data(self):
        """获取当前声纳数据"""
        return self.current_sonar_data.copy()
    
    def get_sensor_distance(self, sensor_name):
        """获取指定传感器的距离值"""
        return self.current_sonar_data.get(sensor_name, 0)
    
    def is_data_fresh(self, timeout=1.0):
        """检查数据是否新鲜"""
        return (time.time() - self.last_valid_time) < timeout

def main():
    sonar_reader = SonarReader('/dev/ttyUSB1', 115200)
    
    try:
        if sonar_reader.start():
            # 主循环，可以在这里添加其他处理逻辑
            while True:
                # 检查数据是否新鲜
                if not sonar_reader.is_data_fresh():
                    print("警告: 声纳数据已超时")
                else:
                    # 可以在这里添加基于声纳数据的处理逻辑
                    current_data = sonar_reader.get_current_sonar_data()
                    # 示例：检查是否有障碍物
                    for sensor, distance in current_data.items():
                        if distance < 500:  # 500mm阈值
                            print(f"警告: {sensor}检测到近距离障碍物: {distance}mm")
                
                time.sleep(1)
                
    except KeyboardInterrupt:
        print("\n正在停止声纳读取...")
    finally:
        sonar_reader.stop()

if __name__ == "__main__":
    main()
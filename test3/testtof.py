import serial
import time
import struct

def read_tof_data(port='/dev/ttyUSB2', baudrate=115200):
    """
    读取TOF雷达数据并解析，20Hz输出
    """
    try:
        # 打开串口
        ser = serial.Serial(port, baudrate, timeout=1)
        print(f"成功打开串口: {port}")
        print("开始读取数据 (20Hz输出)...")
        print("距离(mm) | 信号强度 | 温度(℃) | 置信度 | 校验和")
        print("-" * 55)
        
        # 控制输出频率为20Hz (每0.05秒输出一次)
        output_interval = 0.05  # 秒
        last_output_time = 0
        
        while True:
            # 查找数据帧头 0x59 0x59
            while True:
                byte1 = ser.read(1)
                if byte1 == b'\x59':
                    byte2 = ser.read(1)
                    if byte2 == b'\x59':
                        break
            
            # 读取剩余的数据帧（7个字节）
            data = ser.read(7)
            if len(data) != 7:
                continue
            
            # 完整的9字节数据帧
            frame = b'\x59\x59' + data
            
            # 解析数据
            dist_l, dist_h, peak_l, peak_h, temp, confidence, checksum = struct.unpack('<BBBBBBB', data)
            
            # 计算距离（单位：mm）
            distance = dist_l + dist_h * 256
            
            # 计算信号强度
            peak = peak_l + peak_h * 256
            
            # 计算温度（有符号数）
            if temp > 127:
                temperature = temp - 256  # 负数温度
            else:
                temperature = temp
            
            # 验证校验和
            calculated_checksum = sum(frame[:-1]) & 0xFF
            checksum_valid = (calculated_checksum == checksum)
            
            # 控制输出频率为20Hz
            current_time = time.time()
            if current_time - last_output_time >= output_interval:
                # 简化输出，只显示数据值
                print(f"{distance:6d} | {peak:8d} | {temperature:7d} | {confidence:6d} | {'有效' if checksum_valid else '无效'}")
                last_output_time = current_time
            
    except serial.SerialException as e:
        print(f"串口错误: {e}")
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    except Exception as e:
        print(f"发生错误: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("串口已关闭")

if __name__ == "__main__":
    # 检查设备是否存在
    import os
    if not os.path.exists('/dev/ttyUSB2'):
        print("错误: 设备 /dev/ttyUSB2 不存在")
        print("请使用 'ls /dev/ttyUSB*' 命令查看可用设备")
        exit(1)
    
    print("TOF雷达数据读取测试 - 20Hz输出")
    print("按 Ctrl+C 退出程序")
    read_tof_data()
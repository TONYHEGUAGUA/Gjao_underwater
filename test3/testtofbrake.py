#!/usr/bin/env python3
import rospy
import serial
import threading
import struct
import time

class TOFNavigationTest:
    def __init__(self):
        rospy.init_node('tof_navigation_test', anonymous=True)
        
        # TOF雷达串口初始化
        try:
            self.tof_ser = serial.Serial('/dev/ttyUSB2', 115200, timeout=1)
            rospy.loginfo("成功连接到TOF雷达串口 /dev/ttyUSB2")
        except Exception as e:
            rospy.logerr(f"无法打开TOF雷达串口 /dev/ttyUSB2: {e}")
            return
        
        # 机器人控制串口初始化
        try:
            self.robot_ser = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
            rospy.loginfo("成功连接到机器人控制串口 /dev/ttyUSB1")
        except Exception as e:
            rospy.logerr(f"无法打开机器人控制串口 /dev/ttyUSB1: {e}")
            return
        
        # 初始化机器人
        self.initialize_robot()
        
        # TOF数据相关变量
        self.current_distance = float('inf')  # 初始化为最大值
        self.tof_running = True
        self.tof_initialized = False
        
        # 控制参数
        self.stop_distance = 0.5  # 停止距离：0.5米
        self.running = True
        
        # 启动TOF读取线程
        self.tof_thread = threading.Thread(target=self.read_tof_data)
        self.tof_thread.daemon = True
        self.tof_thread.start()
        
        # 等待TOF数据初始化
        rospy.loginfo("等待TOF数据初始化...")
        start_time = time.time()
        while not self.tof_initialized and (time.time() - start_time) < 10.0:
            rospy.sleep(0.1)
        
        if not self.tof_initialized:
            rospy.logwarn("TOF数据初始化超时")
        else:
            rospy.loginfo("TOF数据初始化完成")
        
        # 主控制循环
        self.control_loop()
    
    def initialize_robot(self):
        """初始化机器人：开启负压吸附并设置速度"""
        rospy.loginfo("初始化机器人...")
        for i in range(9):
            self.robot_ser.write('3'.encode())
            rospy.sleep(0.1)
        for i in range(6):
            self.robot_ser.write('6'.encode())
            rospy.sleep(0.1)
        
        self.robot_ser.write('!'.encode())
        rospy.loginfo("负压吸附已开启，速度已降低")
        rospy.sleep(1)
    
    def read_tof_data(self):
        """读取TOF雷达数据"""
        rospy.loginfo("开始读取TOF数据...")
        
        while self.tof_running and not rospy.is_shutdown():
            try:
                # 查找数据帧头 0x59 0x59
                while self.tof_running:
                    byte1 = self.tof_ser.read(1)
                    if byte1 == b'\x59':
                        byte2 = self.tof_ser.read(1)
                        if byte2 == b'\x59':
                            break
                
                # 读取剩余的数据帧（7个字节）
                data = self.tof_ser.read(7)
                if len(data) != 7:
                    continue
                
                # 完整的9字节数据帧
                frame = b'\x59\x59' + data
                
                # 解析数据
                dist_l, dist_h, peak_l, peak_h, temp, confidence, checksum = struct.unpack('<BBBBBBB', data)
                
                # 计算距离（单位：mm）
                distance_mm = dist_l + dist_h * 256
                
                # 转换为米
                distance_m = distance_mm / 1000.0
                
                # 计算信号强度
                peak = peak_l + peak_h * 256
                
                # 验证校验和
                calculated_checksum = sum(frame[:-1]) & 0xFF
                checksum_valid = (calculated_checksum == checksum)
                
                # 只使用校验和有效且信号强度足够的数据
                if checksum_valid and peak >= 30:
                    self.current_distance = distance_m
                    if not self.tof_initialized:
                        self.tof_initialized = True
                        rospy.loginfo(f"TOF初始化完成，初始距离: {distance_m:.3f}m")
                
                # 限制读取频率
                rospy.sleep(0.01)
                
            except Exception as e:
                rospy.logwarn(f"读取TOF数据错误: {e}")
                rospy.sleep(0.1)
    
    def send_control_command(self, command):
        """发送控制命令到机器人"""
        try:
            self.robot_ser.write(command.encode())
            rospy.logdebug(f"发送命令: {command}")
        except Exception as e:
            rospy.logerr(f"发送命令失败: {e}")
    
    def stop_robot(self):
        """停止机器人"""
        self.send_control_command('0')
        rospy.loginfo("机器人已停止")
    
    def control_loop(self):
        """主控制循环"""
        rospy.loginfo("=== TOF导航测试开始 ===")
        rospy.loginfo(f"机器人将前进，直到TOF检测到距离小于 {self.stop_distance} 米")
        rospy.loginfo("按 Ctrl+C 停止测试")
        
        try:
            # 等待TOF数据稳定
            rospy.sleep(1.0)
            
            # 开始前进
            rospy.loginfo("开始前进...")
            self.send_control_command('A')  # 前进
            
            # 主循环
            rate = rospy.Rate(10)  # 10Hz
            while self.running and not rospy.is_shutdown():
                # 记录当前距离（限制日志频率）
                rospy.loginfo_throttle(1, f"当前距离: {self.current_distance:.3f}m")
                
                # 检查是否到达停止距离
                if self.current_distance <= self.stop_distance:
                    self.stop_robot()
                    rospy.loginfo(f"🎯 到达停止距离！当前距离: {self.current_distance:.3f}m")
                    break
                
                # 检查TOF数据是否有效
                if self.current_distance == float('inf'):
                    rospy.logwarn_throttle(2, "TOF数据无效，等待有效数据...")
                
                rate.sleep()
            
            # 测试完成
            rospy.loginfo("=== TOF导航测试完成 ===")
            
        except KeyboardInterrupt:
            rospy.loginfo("测试被用户中断")
        except Exception as e:
            rospy.logerr(f"控制循环错误: {e}")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """清理资源"""
        self.running = False
        self.tof_running = False
        self.stop_robot()
        
        try:
            if hasattr(self, 'tof_ser') and self.tof_ser.is_open:
                self.tof_ser.close()
                rospy.loginfo("TOF串口已关闭")
        except:
            pass
        
        try:
            if hasattr(self, 'robot_ser') and self.robot_ser.is_open:
                self.robot_ser.close()
                rospy.loginfo("机器人串口已关闭")
        except:
            pass

if __name__ == '__main__':
    try:
        tester = TOFNavigationTest()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"程序异常: {e}")
    finally:
        rospy.loginfo("程序结束")
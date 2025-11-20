#!/usr/bin/env python3
import rospy
import serial
import threading
import struct
import time
import math

class PoolCleaningTest:
    def __init__(self):
        rospy.init_node('pool_cleaning_test', anonymous=True)
        
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
        
        # IMU数据相关变量
        self.current_imu_yaw = 0.0
        self.last_imu_time = 0
        self.imu_initialized = False
        self.imu_running = True
        
        # 启动IMU读取线程
        self.imu_thread = threading.Thread(target=self.read_imu_data)
        self.imu_thread.daemon = True
        self.imu_thread.start()
        
        # 初始化机器人
        self.initialize_robot()
        
        # TOF数据相关变量
        self.current_distance = float('inf')
        self.tof_running = True
        self.tof_initialized = False
        
        # 控制参数
        self.stop_distance = 0.5  # 停止距离：0.5米
        self.target_turn_angle = math.pi / 2  # 90度
        self.angle_tolerance = 0.05  # 约3度角度容差
        
        # 状态变量
        self.running = True
        self.current_state = "INIT"
        self.turn_direction = "RIGHT"
        self.turn_start_yaw = 0.0  # 转向开始时的角度
        self.turn_target_yaw = 0.0  # 转向目标角度
        self.turn_count = 0
        
        # 启动TOF读取线程
        self.tof_thread = threading.Thread(target=self.read_tof_data)
        self.tof_thread.daemon = True
        self.tof_thread.start()
        
        # 等待传感器数据初始化
        self.wait_for_sensors_ready()
        
        # 主控制循环
        self.control_loop()
    
    def wait_for_sensors_ready(self):
        """等待所有传感器数据就绪"""
        rospy.loginfo("等待传感器数据初始化...")
        
        # 等待IMU
        start_time = time.time()
        while not self.imu_initialized and (time.time() - start_time) < 10.0:
            rospy.sleep(0.1)
        if self.imu_initialized:
            rospy.loginfo("IMU初始化完成")
        else:
            rospy.logwarn("IMU数据初始化超时")
        
        # 等待TOF
        start_time = time.time()
        while not self.tof_initialized and (time.time() - start_time) < 10.0:
            rospy.sleep(0.1)
        if self.tof_initialized:
            rospy.loginfo("TOF初始化完成")
        else:
            rospy.logwarn("TOF数据初始化超时")
    
    def read_imu_data(self):
        """读取IMU串口数据"""
        buffer = ""
        while self.imu_running and not rospy.is_shutdown():
            try:
                if self.robot_ser.in_waiting > 0:
                    data = self.robot_ser.read(self.robot_ser.in_waiting).decode('ascii', errors='ignore')
                    buffer += data
                    
                    while '\r\n' in buffer:
                        frame_end = buffer.find('\r\n')
                        frame = buffer[:frame_end]
                        buffer = buffer[frame_end + 2:]
                        
                        if frame and frame.startswith('$IMU,'):
                            try:
                                parts = frame[1:].split(',')
                                if len(parts) == 3 and parts[0] == 'IMU':
                                    yaw_str, checksum_str = parts[1], parts[2]
                                    # 简化的校验和验证
                                    if len(checksum_str) == 2:
                                        yaw = float(yaw_str)
                                        self.current_imu_yaw = math.radians(yaw)
                                        self.last_imu_time = rospy.get_time()
                                        
                                        if not self.imu_initialized:
                                            self.imu_initialized = True
                                            rospy.loginfo(f"IMU初始化完成，偏航角: {yaw:.2f}°")
                            except ValueError:
                                continue
                
                rospy.sleep(0.001)
                
            except Exception as e:
                rospy.logwarn(f"读取IMU数据错误: {e}")
                rospy.sleep(0.1)
    
    def get_current_yaw(self):
        """获取当前偏航角"""
        if self.imu_initialized:
            # 规范化角度到[-pi, pi]范围
            yaw = self.current_imu_yaw
            while yaw > math.pi:
                yaw -= 2 * math.pi
            while yaw < -math.pi:
                yaw += 2 * math.pi
            return yaw
        else:
            return 0.0
    
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
                
                rospy.sleep(0.01)
                
            except Exception as e:
                rospy.logwarn(f"读取TOF数据错误: {e}")
                rospy.sleep(0.1)
    
    def send_control_command(self, command):
        """发送控制命令到机器人"""
        try:
            self.robot_ser.write(command.encode())
        except Exception as e:
            rospy.logerr(f"发送命令失败: {e}")
    
    def stop_robot(self):
        """停止机器人"""
        self.send_control_command('0')
    
    def start_forward(self):
        """开始前进"""
        self.send_control_command('A')
        rospy.loginfo("开始前进")
    
    def start_turn(self, direction):
        """开始转向"""
        if direction == "RIGHT":
            self.send_control_command('C')  # 右转
        else:  # LEFT
            self.send_control_command('D')  # 左转
    
    def control_loop(self):
        """主控制循环 - 使用IMU精确角度控制"""
        rospy.loginfo("=== 水池清理任务开始 ===")
        rospy.loginfo(f"停止距离: {self.stop_distance}米")
        rospy.loginfo("使用IMU角度反馈进行精确转向控制")
        rospy.loginfo("按 Ctrl+C 停止测试")
        
        try:
            # 等待数据稳定
            rospy.sleep(1.0)
            
            # 初始状态：前进
            self.current_state = "FORWARD"
            self.start_forward()
            
            # 主循环
            rate = rospy.Rate(20)  # 20Hz
            while self.running and not rospy.is_shutdown():
                
                if self.current_state == "FORWARD":
                    self.handle_forward_state()
                elif self.current_state == "TURNING":
                    self.handle_turning_state()
                
                # 记录当前状态（限制日志频率）
                current_yaw = math.degrees(self.get_current_yaw())
                rospy.loginfo_throttle(2, 
                    f"状态: {self.current_state}, 距离: {self.current_distance:.3f}m, "
                    f"当前角度: {current_yaw:.1f}°, 转向计数: {self.turn_count}")
                
                rate.sleep()
            
            rospy.loginfo("=== 水池清理任务完成 ===")
            
        except KeyboardInterrupt:
            rospy.loginfo("测试被用户中断")
        except Exception as e:
            rospy.logerr(f"控制循环错误: {e}")
        finally:
            self.cleanup()
    
    def handle_forward_state(self):
        """处理前进状态"""
        if self.current_distance <= self.stop_distance:
            self.stop_robot()
            rospy.loginfo(f"🎯 检测到障碍物！距离: {self.current_distance:.3f}m")
            
            # 切换到转向状态
            self.current_state = "TURNING"
            self.turn_start_yaw = self.get_current_yaw()
            self.turn_count = 0
            
            # 确定转向方向（轮流）
            if self.turn_direction == "RIGHT":
                self.turn_direction = "LEFT"
            else:
                self.turn_direction = "RIGHT"
            
            rospy.loginfo(f"开始转向序列: {self.turn_direction}")
            self.start_first_turn()
    
    def start_first_turn(self):
        """开始第一次转向"""
        self.turn_count = 1
        self.turn_target_yaw = self.turn_start_yaw
        
        if self.turn_direction == "RIGHT":
            self.turn_target_yaw -= self.target_turn_angle  # 右转为负角度
        else:
            self.turn_target_yaw += self.target_turn_angle  # 左转为正角度
        
        # 规范化目标角度
        while self.turn_target_yaw > math.pi:
            self.turn_target_yaw -= 2 * math.pi
        while self.turn_target_yaw < -math.pi:
            self.turn_target_yaw += 2 * math.pi
        
        rospy.loginfo(f"第{self.turn_count}次转向: {math.degrees(self.turn_start_yaw):.1f}° → {math.degrees(self.turn_target_yaw):.1f}°")
        self.start_turn(self.turn_direction)
    
    def handle_turning_state(self):
        """处理转向状态 - 使用IMU角度反馈"""
        if not self.imu_initialized:
            rospy.logwarn("IMU未初始化，使用时间控制")
            # 回退到时间控制
            self.fallback_time_control()
            return
        
        current_yaw = self.get_current_yaw()
        angle_error = self.calculate_angle_error(self.turn_target_yaw)
        
        # 检查是否到达目标角度
        if abs(angle_error) <= self.angle_tolerance:
            self.stop_robot()
            rospy.loginfo(f"转向 {self.turn_count} 完成！当前角度: {math.degrees(current_yaw):.1f}°")
            
            # 短暂停顿
            rospy.sleep(0.5)
            
            if self.turn_count < 2:
                # 开始第二次转向
                self.start_second_turn()
            else:
                # 完成两次转向，继续前进
                rospy.loginfo("转向序列完成，继续前进")
                self.current_state = "FORWARD"
                rospy.sleep(0.5)
                self.start_forward()
    
    def start_second_turn(self):
        """开始第二次转向"""
        self.turn_count += 1
        current_yaw = self.get_current_yaw()
        self.turn_target_yaw = current_yaw  # 从当前位置开始
        
        if self.turn_direction == "RIGHT":
            self.turn_target_yaw -= self.target_turn_angle
        else:
            self.turn_target_yaw += self.target_turn_angle
        
        # 规范化目标角度
        while self.turn_target_yaw > math.pi:
            self.turn_target_yaw -= 2 * math.pi
        while self.turn_target_yaw < -math.pi:
            self.turn_target_yaw += 2 * math.pi
        
        rospy.loginfo(f"第{self.turn_count}次转向: {math.degrees(current_yaw):.1f}° → {math.degrees(self.turn_target_yaw):.1f}°")
        self.start_turn(self.turn_direction)
    
    def fallback_time_control(self):
        """回退方案：时间控制"""
        rospy.logwarn("使用时间控制转向")
        self.stop_robot()
        rospy.sleep(0.5)
        
        if self.turn_count < 2:
            self.turn_count += 1
            rospy.loginfo(f"开始第 {self.turn_count} 次转向（时间控制）")
            self.start_turn(self.turn_direction)
            rospy.sleep(2.0)  # 固定时间转向
        else:
            rospy.loginfo("转向序列完成，继续前进")
            self.current_state = "FORWARD"
            rospy.sleep(0.5)
            self.start_forward()
    
    def cleanup(self):
        """清理资源"""
        self.running = False
        self.tof_running = False
        self.imu_running = False
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
        cleaner = PoolCleaningTest()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"程序异常: {e}")
    finally:
        rospy.loginfo("程序结束")
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
        self.forward_distance = 0.2  # 转弯后前进距离：0.2米
        self.target_turn_angle = math.pi / 2  # 90度
        self.angle_tolerance = 0.05  # 约3度角度容差
        
        # 状态变量 - 关键修改！
        self.running = True
        self.current_state = "INIT"
        self.current_turn_direction = "LEFT"  # 起始转向方向
        self.initial_yaw = 0.0  # 初始角度（绝对0度基准）
        self.turn_target_yaw = 0.0  # 转向目标角度
        self.edge_count = 0  # 边计数
        self.forward_start_time = 0.0
        self.forward_progress = 0.0
        self.task_completed = False
        self.final_edge_started = False  # 标记是否开始最后一条边
        
        # 启动TOF读取线程
        self.tof_thread = threading.Thread(target=self.read_tof_data)
        self.tof_thread.daemon = True
        self.tof_thread.start()
        
        # 等待传感器数据初始化
        self.wait_for_sensors_ready()
        
        # 记录初始角度
        self.record_initial_yaw()
        
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
    
    def record_initial_yaw(self):
        """记录初始角度作为绝对0度基准"""
        if self.imu_initialized:
            # 取多个采样点平均以提高精度
            yaw_sum = 0.0
            sample_count = 10
            for i in range(sample_count):
                yaw_sum += self.get_current_yaw()
                rospy.sleep(0.1)
            
            self.initial_yaw = yaw_sum / sample_count
            rospy.loginfo(f"🎯 初始角度已记录: {math.degrees(self.initial_yaw):.2f}° (绝对0度基准)")
        else:
            rospy.logwarn("IMU未初始化，无法记录初始角度")
            self.initial_yaw = 0.0
    
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
        """主控制循环 - 弓字形巡检"""
        rospy.loginfo("=== 水池清理任务开始 ===")
        rospy.loginfo(f"停止距离: {self.stop_distance}米")
        rospy.loginfo(f"侧移距离: {self.forward_distance}米")
        rospy.loginfo("🔄 使用弓字形巡检模式")
        rospy.loginfo(f"起始转向方向: {self.current_turn_direction}")
        rospy.loginfo("按 Ctrl+C 停止测试")
        
        try:
            # 等待数据稳定
            rospy.sleep(1.0)
            
            # 初始状态：前进
            self.current_state = "FORWARD"
            self.start_forward()
            
            # 主循环
            rate = rospy.Rate(20)  # 20Hz
            while self.running and not rospy.is_shutdown() and not self.task_completed:
                
                if self.current_state == "FORWARD":
                    self.handle_forward_state()
                elif self.current_state == "TURN_AFTER_EDGE":
                    self.handle_turn_after_edge_state()
                elif self.current_state == "SIDEWAYS_MOVE":
                    self.handle_sideways_move_state()
                elif self.current_state == "TURN_FOR_NEXT_EDGE":
                    self.handle_turn_for_next_edge_state()
                elif self.current_state == "FINAL_TURN":
                    self.handle_final_turn_state()
                elif self.current_state == "FINAL_FORWARD":
                    self.handle_final_forward_state()
                
                # 记录当前状态（限制日志频率）
                current_yaw = math.degrees(self.get_current_yaw())
                rospy.loginfo_throttle(2, 
                    f"状态: {self.current_state}, 距离: {self.current_distance:.3f}m, "
                    f"当前角度: {current_yaw:.1f}°, 边计数: {self.edge_count}, 转向方向: {self.current_turn_direction}")
                
                rate.sleep()
            
            if self.task_completed:
                rospy.loginfo("🎉 弓字形巡检任务完成！")
            else:
                rospy.loginfo("=== 水池清理任务结束 ===")
            
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
            rospy.loginfo(f"🎯 检测到边界！距离: {self.current_distance:.3f}m")
            
            if self.final_edge_started:
                rospy.loginfo("🎉 到达最终边界，任务完成！")
                self.task_completed = True
                return
            
            # 切换到转向状态
            self.current_state = "TURN_AFTER_EDGE"
            self.edge_count += 1
            
            rospy.loginfo(f"开始第{self.edge_count}条边结束转向: {self.current_turn_direction}")
            self.start_turn_after_edge()
    
    def start_turn_after_edge(self):
        """开始边结束后的转向"""
        current_yaw = self.get_current_yaw()
        
        if self.current_turn_direction == "LEFT":
            self.turn_target_yaw = current_yaw + self.target_turn_angle
        else:
            self.turn_target_yaw = current_yaw - self.target_turn_angle
        
        rospy.loginfo(f"边结束转向: {math.degrees(current_yaw):.1f}° → {math.degrees(self.turn_target_yaw):.1f}°")
        self.start_turn(self.current_turn_direction)
    
    def handle_turn_after_edge_state(self):
        """处理边结束后的转向状态"""
        if not self.imu_initialized:
            rospy.logwarn("IMU未初始化，使用时间控制")
            self.fallback_turn_after_edge()
            return
        
        current_yaw = self.get_current_yaw()
        angle_error = self.calculate_angle_error(self.turn_target_yaw)
        
        # 检查是否到达目标角度
        if abs(angle_error) <= self.angle_tolerance:
            self.stop_robot()
            rospy.loginfo(f"✅ 边结束转向完成！当前角度: {math.degrees(current_yaw):.1f}°")
            
            # 短暂停顿后开始侧向移动
            rospy.sleep(0.5)
            self.start_sideways_move()
    
    def start_sideways_move(self):
        """开始侧向移动（前进0.2米）"""
        self.current_state = "SIDEWAYS_MOVE"
        self.forward_start_time = time.time()
        self.forward_progress = 0.0
        self.start_forward()
        rospy.loginfo(f"开始侧向移动0.2米")
    
    def handle_sideways_move_state(self):
        """处理侧向移动状态"""
        # 计算前进进度（基于时间估算）
        elapsed_time = time.time() - self.forward_start_time
        estimated_progress = elapsed_time * 0.1  # 假设速度约为0.1m/s
        
        # 更新前进进度
        self.forward_progress = estimated_progress
        
        # 检查是否到达目标距离
        if estimated_progress >= self.forward_distance:
            self.stop_robot()
            rospy.loginfo(f"侧向移动0.2米完成")
            self.current_state = "TURN_FOR_NEXT_EDGE"
            rospy.sleep(0.5)
            self.start_turn_for_next_edge()
            return
        
        # 检查是否遇到紧急停止条件（提前遇到边界）
        if self.current_distance <= self.stop_distance:
            self.stop_robot()
            rospy.loginfo(f"🚨 检测到侧向边界！开始最后一条边的行走")
            self.final_edge_started = True
            rospy.sleep(0.5)
            
            # 需要再转90度才能开始最后一条边
            self.current_state = "FINAL_TURN"
            rospy.loginfo("开始最终转向")
            self.start_final_turn()
    
    def start_turn_for_next_edge(self):
        """开始为下一条边转向"""
        # 这次转向方向与上次相同（完成U形转弯）
        current_yaw = self.get_current_yaw()
        if self.current_turn_direction == "LEFT":
            self.turn_target_yaw = current_yaw + self.target_turn_angle
        else:
            self.turn_target_yaw = current_yaw - self.target_turn_angle
        
        rospy.loginfo(f"下一条边转向: {math.degrees(current_yaw):.1f}° → {math.degrees(self.turn_target_yaw):.1f}°")
        self.start_turn(self.current_turn_direction)
    
    def handle_turn_for_next_edge_state(self):
        """处理为下一条边转向的状态"""
        if not self.imu_initialized:
            rospy.logwarn("IMU未初始化，使用时间控制")
            self.fallback_turn_for_next_edge()
            return
        
        current_yaw = self.get_current_yaw()
        angle_error = self.calculate_angle_error(self.turn_target_yaw)
        
        # 检查是否到达目标角度
        if abs(angle_error) <= self.angle_tolerance:
            self.stop_robot()
            rospy.loginfo(f"✅ 下一条边转向完成！当前角度: {math.degrees(current_yaw):.1f}°")
            
            # 切换转向方向，为下一次做准备
            if self.current_turn_direction == "LEFT":
                self.current_turn_direction = "RIGHT"
            else:
                self.current_turn_direction = "LEFT"
            
            rospy.sleep(0.5)
            self.current_state = "FORWARD"
            self.start_forward()
            rospy.loginfo(f"转向方向切换为: {self.current_turn_direction}，开始下一条边")
    
    def start_final_turn(self):
        """开始最终转向（完成最后一条边）"""
        current_yaw = self.get_current_yaw()
        # 最终转向与当前转向方向相同
        if self.current_turn_direction == "LEFT":
            self.turn_target_yaw = current_yaw + self.target_turn_angle
        else:
            self.turn_target_yaw = current_yaw - self.target_turn_angle
        
        rospy.loginfo(f"最终转向: {math.degrees(current_yaw):.1f}° → {math.degrees(self.turn_target_yaw):.1f}°")
        self.start_turn(self.current_turn_direction)
    
    def handle_final_turn_state(self):
        """处理最终转向状态"""
        if not self.imu_initialized:
            rospy.logwarn("IMU未初始化，使用时间控制")
            self.fallback_final_turn()
            return
        
        current_yaw = self.get_current_yaw()
        angle_error = self.calculate_angle_error(self.turn_target_yaw)
        
        # 检查是否到达目标角度
        if abs(angle_error) <= self.angle_tolerance:
            self.stop_robot()
            rospy.loginfo(f"✅ 最终转向完成！当前角度: {math.degrees(current_yaw):.1f}°")
            
            # 短暂停顿后开始最后一条边的前进
            rospy.sleep(0.5)
            self.current_state = "FINAL_FORWARD"
            rospy.loginfo("开始最后一条边的前进")
            self.start_forward()
    
    def handle_final_forward_state(self):
        """处理最终前进状态（最后一条边的前进）"""
        # 在最后一条边前进时，检测到障碍物就结束任务
        if self.current_distance <= self.stop_distance:
            self.stop_robot()
            rospy.loginfo(f"🎉 到达最终边界！距离: {self.current_distance:.3f}m")
            self.task_completed = True
            rospy.loginfo("弓字形巡检任务完成！")
    
    def fallback_turn_after_edge(self):
        """回退方案：边结束转向时间控制"""
        rospy.logwarn("使用时间控制边结束转向")
        self.stop_robot()
        rospy.sleep(0.5)
        self.start_turn(self.current_turn_direction)
        rospy.sleep(2.0)  # 固定时间转向
        self.stop_robot()
        rospy.sleep(0.5)
        self.start_sideways_move()
    
    def fallback_turn_for_next_edge(self):
        """回退方案：下一条边转向时间控制"""
        rospy.logwarn("使用时间控制下一条边转向")
        self.stop_robot()
        rospy.sleep(0.5)
        self.start_turn(self.current_turn_direction)
        rospy.sleep(2.0)  # 固定时间转向
        self.stop_robot()
        rospy.sleep(0.5)
        
        # 切换转向方向
        if self.current_turn_direction == "LEFT":
            self.current_turn_direction = "RIGHT"
        else:
            self.current_turn_direction = "LEFT"
        
        self.current_state = "FORWARD"
        self.start_forward()
        rospy.loginfo(f"转向方向切换为: {self.current_turn_direction}，开始下一条边")
    
    def fallback_final_turn(self):
        """回退方案：最终转向时间控制"""
        rospy.logwarn("使用时间控制最终转向")
        self.stop_robot()
        rospy.sleep(0.5)
        self.start_turn(self.current_turn_direction)
        rospy.sleep(2.0)  # 固定时间转向
        self.stop_robot()
        rospy.sleep(0.5)
        self.current_state = "FINAL_FORWARD"
        rospy.loginfo("开始最后一条边的前进")
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
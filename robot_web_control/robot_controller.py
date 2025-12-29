#!/usr/bin/env python3
# robot_controller.py - 机器人控制器（直接串口控制）
# 说明：增强了异常处理与日志、串口写入锁、串口重试、并确保不会引用未绑定的变量。
# 新增：serial_verbose 参数（~serial_verbose），写入时始终打印 "[SERIAL OUT] ..."，并尝试读取串口返回 "[SERIAL IN] ..."

import rospy
import math
import threading
import serial
import time
import traceback
import sys
import logging
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

logger = logging.getLogger(__name__)

class RobotController:
    def __init__(self):
        # 集成你的控制参数
        self.PWM_NEUTRAL = 1500
        self.PWM_MIN = 1000
        self.PWM_MAX = 2000
        self.pwm_drive_diff = 100
        self.pwm_turn_diff = 100

        # 电机方向修正（根据您的硬件实际情况设置）
        self.REVERSE_LEFT_MOTOR = True
        self.REVERSE_RIGHT_MOTOR = True

        # 串口初始化参数（可通过 ROS 参数调整）
        self.serial_port = rospy.get_param('~serial_port', '/dev/ttyUSB2')
        self.baud_rate = rospy.get_param('~baud_rate', 115200)
        self.ser = None
        self.serial_lock = threading.Lock()
        self._serial_retry_count = rospy.get_param('~serial_retry_count', 1)
        self._serial_retry_delay = rospy.get_param('~serial_retry_delay', 0.5)
        # 新增：是否在每次写入时打印发送内容和读取返回（默认 True）
        self.serial_verbose = rospy.get_param('~serial_verbose', True)
        # 最大等待串口返回的时长（秒）
        self._serial_read_timeout = rospy.get_param('~serial_read_timeout', 0.05)

        self._init_serial()

        # ROS发布器（保持原有接口）
        try:
            self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
            self.vacuum_pub = rospy.Publisher('/vacuum_control', Bool, queue_size=10)
        except Exception:
            # 在非 ROS 环境下也能导入并测试
            logger.warning("初始化 ROS publisher 失败（可能非 ROS 环境）:\n" + traceback.format_exc())
            self.cmd_vel_pub = None
            self.vacuum_pub = None

        # 控制状态
        self.current_speed = 0.0
        self.current_turn = 0.0
        self.vacuum_state = False

        # 当前PWM状态
        self.current_left_pwm = self.PWM_NEUTRAL
        self.current_right_pwm = self.PWM_NEUTRAL

        # 控制参数
        self.max_linear_speed = 0.5  # m/s
        self.max_angular_speed = 1.0  # rad/s

        rospy.loginfo("机器人控制器初始化完成（直接串口控制）")
        logger.debug("RobotController initialized: serial_port=%s baud=%s serial_verbose=%s",
                     self.serial_port, self.baud_rate, self.serial_verbose)

    def _init_serial(self):
        """尝试初始化串口，支持重试机制"""
        for attempt in range(1, max(1, self._serial_retry_count) + 1):
            try:
                self.ser = serial.Serial(
                    port=self.serial_port,
                    baudrate=self.baud_rate,
                    timeout=0.1
                )
                rospy.loginfo(f"成功连接到串口 {self.serial_port}")
                # 发送初始化命令（可选）
                self.send_serial_command("INIT\n")
                time.sleep(0.2)
                return
            except Exception as ex:
                rospy.logerr(f"尝试打开串口 {self.serial_port} 第 {attempt} 次失败: {ex}")
                logger.debug("Traceback for serial init:\n" + traceback.format_exc())
                self.ser = None
                if attempt < self._serial_retry_count:
                    time.sleep(self._serial_retry_delay)

        rospy.logwarn("串口打开失败，进入模拟模式（ser=None）")

    def send_serial_command(self, command, write_timeout=1.0):
        """
        发送串口命令并打印发送内容与可能的返回。
        - command: str，要发送的文本（例如 "M,1500,1500\n" 或 "!\n"）
        - 返回 True/False 表示写入是否成功（或已模拟写入）
        """
        # 规范化保证为字符串并以换行结尾（根据你的协议）
        if not isinstance(command, str):
            command = str(command)
        # 不强制添加换行，但通常命令以 '\n' 结尾；你可以按需修改
        out = command

        with self.serial_lock:
            if self.ser and getattr(self.ser, "is_open", False):
                try:
                    # 写入
                    self.ser.write(out.encode())
                    try:
                        self.ser.flush()
                    except Exception:
                        pass

                    # 打印发送内容，便于调试
                    if self.serial_verbose:
                        msg = f"[SERIAL OUT] {out.strip()}"
                        rospy.loginfo(msg)
                        print(msg)

                    # 尝试短时间读取串口返回（如果设备会回复）
                    if self._serial_read_timeout and self._serial_read_timeout > 0:
                        try:
                            # 等待一点时间让设备返回
                            time.sleep(self._serial_read_timeout)
                            # 读取所有可用字节
                            available = 0
                            try:
                                available = self.ser.in_waiting
                            except Exception:
                                # 某些环境下 in_waiting 可能不可用
                                available = 0
                            if available and available > 0:
                                data = self.ser.read(available)
                                try:
                                    decoded = data.decode(errors='ignore').strip()
                                except Exception:
                                    decoded = repr(data)
                                if self.serial_verbose:
                                    in_msg = f"[SERIAL IN] {decoded}"
                                    rospy.loginfo(in_msg)
                                    print(in_msg)
                            else:
                                # 如果没有可读数据，也打印一行（可选）
                                if self.serial_verbose:
                                    rospy.logdebug("[SERIAL IN] <no data>")
                        except Exception:
                            # 读取串口返回时不要让异常影响主流程
                            logger.debug("读取串口返回时出错:\n" + traceback.format_exc())

                    return True
                except Exception as e:
                    # 串口写入失败，记录异常和 traceback
                    rospy.logerr(f"发送串口命令失败: {e}")
                    logger.error("Traceback for serial write:\n%s", traceback.format_exc())
                    if self.serial_verbose:
                        print(f"[SERIAL OUT-ERR] {out.strip()} -> {e}")
                    return False
            else:
                # 串口不可用，模拟打印
                rospy.logwarn(f"串口未连接，模拟发送命令: {out.strip()}")
                print(f"[SIMULATED SERIAL OUT] {out.strip()}")
                logger.debug("simulated serial write: %s", out.strip())
                return False

    def pwm_to_velocity(self, pwm):
        """将PWM值转换为速度值（保持原有逻辑）"""
        try:
            if pwm == self.PWM_NEUTRAL:
                return 0.0
            elif pwm > self.PWM_NEUTRAL:
                normalized = (pwm - self.PWM_NEUTRAL) / (self.PWM_MAX - self.PWM_NEUTRAL)
                return normalized * self.max_linear_speed
            else:
                normalized = (self.PWM_NEUTRAL - pwm) / (self.PWM_NEUTRAL - self.PWM_MIN)
                return -normalized * self.max_linear_speed
        except Exception:
            logger.exception("pwm_to_velocity 出错")
            return 0.0

    def velocity_to_pwm(self, velocity):
        """将速度值转换为PWM值（保持原有逻辑）"""
        try:
            if velocity == 0:
                return self.PWM_NEUTRAL
            elif velocity > 0:
                normalized = velocity / self.max_linear_speed
                pwm = self.PWM_NEUTRAL + normalized * (self.PWM_MAX - self.PWM_NEUTRAL)
                return int(max(self.PWM_NEUTRAL, min(self.PWM_MAX, pwm)))
            else:
                normalized = abs(velocity) / self.max_linear_speed
                pwm = self.PWM_NEUTRAL - normalized * (self.PWM_NEUTRAL - self.PWM_MIN)
                return int(max(self.PWM_MIN, min(self.PWM_NEUTRAL, pwm)))
        except Exception:
            logger.exception("velocity_to_pwm 出错")
            return self.PWM_NEUTRAL

    def send_velocity_command(self, linear_x, angular_z):
        """发送速度命令（改为直接串口输出）"""
        try:
            # 强制数值类型并限制速度范围
            linear_x = float(linear_x)
            angular_z = float(angular_z)
            linear_x = max(-self.max_linear_speed, min(self.max_linear_speed, linear_x))
            angular_z = max(-self.max_angular_speed, min(self.max_angular_speed, angular_z))

            # 创建ROS消息（保持原有接口）
            twist = Twist()
            twist.linear.x = linear_x
            twist.angular.z = angular_z
            try:
                if self.cmd_vel_pub:
                    self.cmd_vel_pub.publish(twist)
            except Exception:
                rospy.logwarn("发布 /cmd_vel 失败（可能非 ROS 环境）:\n" + traceback.format_exc())

            # 更新当前状态
            self.current_speed = linear_x
            self.current_turn = angular_z

            # 计算PWM值
            left_pwm, right_pwm = self.calculate_pwm(linear_x, angular_z)

            # 应用电机反转修正
            if self.REVERSE_LEFT_MOTOR:
                left_pwm = 2 * self.PWM_NEUTRAL - left_pwm
            if self.REVERSE_RIGHT_MOTOR:
                right_pwm = 2 * self.PWM_NEUTRAL - right_pwm

            # 确保在安全范围内
            left_pwm = max(self.PWM_MIN, min(self.PWM_MAX, int(left_pwm)))
            right_pwm = max(self.PWM_MIN, min(self.PWM_MAX, int(right_pwm)))

            # 发送串口命令 - 格式：M,left_pwm,right_pwm\n
            command = f"M,{left_pwm},{right_pwm}\n"
            success = self.send_serial_command(command)

            # 更新当前PWM状态
            self.current_left_pwm = left_pwm
            self.current_right_pwm = right_pwm

            rospy.loginfo(f"速度命令: linear={linear_x:.2f}, angular={angular_z:.2f}, PWM: L={left_pwm}, R={right_pwm}")
            logger.debug("send_velocity_command success=%s", success)

            return success

        except Exception as e:
            rospy.logerr(f"发送速度命令失败: {e}")
            logger.error("Traceback for send_velocity_command:\n%s", traceback.format_exc())
            return False

    def calculate_pwm(self, linear_x, angular_z):
        """根据线速度和角速度计算左右轮PWM值（保持原有逻辑）"""
        try:
            base_pwm = self.velocity_to_pwm(linear_x)

            if angular_z == 0:
                left_pwm = base_pwm
                right_pwm = base_pwm
            elif angular_z > 0:
                turn_adjust = int(abs(angular_z) / self.max_angular_speed * self.pwm_turn_diff)
                left_pwm = base_pwm - turn_adjust
                right_pwm = base_pwm + turn_adjust
            else:
                turn_adjust = int(abs(angular_z) / self.max_angular_speed * self.pwm_turn_diff)
                left_pwm = base_pwm + turn_adjust
                right_pwm = base_pwm - turn_adjust

            left_pwm = max(self.PWM_MIN, min(self.PWM_MAX, left_pwm))
            right_pwm = max(self.PWM_MIN, min(self.PWM_MAX, right_pwm))

            return left_pwm, right_pwm
        except Exception:
            logger.exception("calculate_pwm 出错")
            return self.PWM_NEUTRAL, self.PWM_NEUTRAL

    def control_vacuum(self, state):
        """控制负压吸附（改为直接串口输出）；保证异常安全并记录完整 traceback"""
        try:
            # 强制转为布尔值
            desired = bool(state)

            # 创建ROS消息（保持原有接口）
            if self.vacuum_pub:
                try:
                    msg = Bool()
                    msg.data = desired
                    self.vacuum_pub.publish(msg)
                except Exception:
                    rospy.logwarn("发布 /vacuum_control 失败（可能非 ROS 环境）:\n" + traceback.format_exc())

            # 发送串口命令
            if desired:
                command = '!\n'  # 开启负压
            else:
                command = '%\n'  # 关闭负压

            success = self.send_serial_command(command)

            if success:
                self.vacuum_state = desired
                rospy.loginfo(f"负压吸附状态: {'开启' if desired else '关闭'}")
                logger.debug("vacuum state set to %s", desired)
            else:
                rospy.logwarn(f"尝试设置负压为 {'开启' if desired else '关闭'}，但串口写入失败或在模拟模式。")

            return success

        except Exception as e:
            # 确保绑定了 e 并打印完整 traceback，避免出现 'local variable e referenced before assignment'
            rospy.logerr(f"控制负压吸附失败: {e}")
            logger.error("Traceback for control_vacuum:\n%s", traceback.format_exc())
            return False

    def get_status(self):
        """获取当前状态（增强信息）"""
        try:
            serial_connected = bool(self.ser and getattr(self.ser, "is_open", False))
        except Exception:
            serial_connected = False

        return {
            'speed': self.current_speed,
            'turn': self.current_turn,
            'vacuum': self.vacuum_state,
            'left_pwm': self.current_left_pwm,
            'right_pwm': self.current_right_pwm,
            'max_linear_speed': self.max_linear_speed,
            'max_angular_speed': self.max_angular_speed,
            'serial_connected': serial_connected,
            'serial_port': self.serial_port
        }

    def emergency_stop(self):
        """紧急停止（改为直接串口输出）"""
        try:
            # 发送停止命令到串口
            command = f"M,{self.PWM_NEUTRAL},{self.PWM_NEUTRAL}\n"
            self.send_serial_command(command)

            # 更新状态
            self.current_speed = 0.0
            self.current_turn = 0.0
            self.current_left_pwm = self.PWM_NEUTRAL
            self.current_right_pwm = self.PWM_NEUTRAL

            # 发布ROS消息（保持原有接口）
            try:
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                if self.cmd_vel_pub:
                    self.cmd_vel_pub.publish(twist)
            except Exception:
                rospy.logwarn("发布 /cmd_vel（紧急停止）失败：\n" + traceback.format_exc())

            rospy.loginfo("紧急停止")
        except Exception:
            rospy.logerr("emergency_stop 出现异常:\n" + traceback.format_exc())

    def cleanup(self):
        """清理资源"""
        try:
            if self.ser and getattr(self.ser, "is_open", False):
                self.emergency_stop()

                # 关闭负压吸附
                if self.vacuum_state:
                    try:
                        self.control_vacuum(False)
                    except Exception:
                        rospy.logwarn("关闭负压时出现异常:\n" + traceback.format_exc())

                try:
                    self.ser.close()
                    rospy.loginfo("串口已关闭")
                except Exception:
                    rospy.logwarn("关闭串口时发生异常:\n" + traceback.format_exc())
        except Exception:
            rospy.logwarn("cleanup 出现异常:\n" + traceback.format_exc())


# 全局机器人控制器实例
robot = None

def get_robot_controller():
    """获取机器人控制器实例（单例模式）"""
    global robot
    if robot is None:
        robot = RobotController()
    return robot
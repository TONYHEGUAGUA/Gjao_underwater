#!/usr/bin/env python3
# robot_controller.py - 机器人控制器（直接串口控制）
# 说明：
# - 提供串口控制（send_velocity_command / control_vacuum / emergency_stop）
# - 串口调试输出（serial_verbose 参数）会在每次写入时打印 [SERIAL OUT]，并尝试读取并打印 [SERIAL IN]
# - calculate_pwm 中对 angular_z 做了映射（mapped_ang = -angular_z）以修正左右转向反向问题
# - 增加 pwm_turn_diff，以及提升默认 max_linear_speed，便于转弯更有力/更快

import rospy
import threading
import serial
import time
import traceback
import logging
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

logger = logging.getLogger(__name__)

class RobotController:
    def __init__(self):
        # PWM 参数
        self.PWM_NEUTRAL = 1500
        self.PWM_MIN = 1000
        self.PWM_MAX = 2000

        # 差分参数（可通过 ROS 参数覆盖）
        self.pwm_drive_diff = rospy.get_param('~pwm_drive_diff', 100)
        # 增大转向差分，使转弯更有力
        self.pwm_turn_diff = rospy.get_param('~pwm_turn_diff', 180)

        # 电机方向修正（硬件反向时修改 True/False）
        self.REVERSE_LEFT_MOTOR = rospy.get_param('~reverse_left_motor', True)
        self.REVERSE_RIGHT_MOTOR = rospy.get_param('~reverse_right_motor', True)

        # 串口参数
        self.serial_port = rospy.get_param('~serial_port', '/dev/ttyUSB1')
        self.baud_rate = rospy.get_param('~baud_rate', 115200)
        self._serial_retry_count = rospy.get_param('~serial_retry_count', 1)
        self._serial_retry_delay = rospy.get_param('~serial_retry_delay', 0.5)
        self.serial_verbose = rospy.get_param('~serial_verbose', True)
        self._serial_read_timeout = rospy.get_param('~serial_read_timeout', 0.05)

        self.ser = None
        self.serial_lock = threading.Lock()

        # 运动参数
        # 加大默认最大线速度为 0.8 m/s（根据实际硬件调整）
        self.max_linear_speed = rospy.get_param('~max_linear_speed', 0.8)
        self.max_angular_speed = rospy.get_param('~max_angular_speed', 1.0)

        # 当前状态
        self.current_speed = 0.0
        self.current_turn = 0.0
        self.vacuum_state = False
        self.current_left_pwm = self.PWM_NEUTRAL
        self.current_right_pwm = self.PWM_NEUTRAL

        # ROS publisher（若在非 ROS 环境可为空）
        try:
            self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
            self.vacuum_pub = rospy.Publisher('/vacuum_control', Bool, queue_size=10)
        except Exception:
            logger.warning("初始化 ROS publisher 失败（可能非 ROS 环境）:\n" + traceback.format_exc())
            self.cmd_vel_pub = None
            self.vacuum_pub = None

        # 初始化串口（尝试）
        self._init_serial()

        rospy.loginfo("RobotController 初始化完成")
        logger.debug("RobotController params: serial_port=%s baud=%s verbose=%s max_linear=%s pwm_turn_diff=%s",
                     self.serial_port, self.baud_rate, self.serial_verbose, self.max_linear_speed, self.pwm_turn_diff)

    def _init_serial(self):
        """尝试打开串口，支持重试"""
        for attempt in range(1, max(1, self._serial_retry_count) + 1):
            try:
                self.ser = serial.Serial(port=self.serial_port, baudrate=self.baud_rate, timeout=0.1)
                rospy.loginfo(f"成功连接到串口 {self.serial_port}")
                # 可选：发送初始化命令
                self.send_serial_command("INIT\n")
                time.sleep(0.1)
                return
            except Exception as ex:
                rospy.logwarn(f"打开串口 {self.serial_port} 第 {attempt} 次失败: {ex}")
                logger.debug("Traceback for serial init:\n" + traceback.format_exc())
                self.ser = None
                if attempt < self._serial_retry_count:
                    time.sleep(self._serial_retry_delay)
        rospy.logwarn("串口打开失败，进入模拟模式（ser=None）")

    def send_serial_command(self, command):
        """
        发送串口命令并（可选）打印发送/接收内容。
        返回 True 表示写入操作执行（实际上写入成功或模拟输出），False 表示写入失败。
        """
        if not isinstance(command, str):
            command = str(command)
        out = command

        with self.serial_lock:
            if self.ser and getattr(self.ser, "is_open", False):
                try:
                    self.ser.write(out.encode())
                    try:
                        self.ser.flush()
                    except Exception:
                        pass

                    if self.serial_verbose:
                        msg = f"[SERIAL OUT] {out.strip()}"
                        rospy.loginfo(msg)
                        print(msg)

                    # 尝试短时间读取串口返回
                    if self._serial_read_timeout and self._serial_read_timeout > 0:
                        try:
                            time.sleep(self._serial_read_timeout)
                            available = 0
                            try:
                                available = self.ser.in_waiting
                            except Exception:
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
                                if self.serial_verbose:
                                    rospy.logdebug("[SERIAL IN] <no data>")
                        except Exception:
                            logger.debug("读取串口返回时出错:\n" + traceback.format_exc())

                    return True
                except Exception as e:
                    rospy.logerr(f"发送串口命令失败: {e}")
                    logger.error("Traceback for serial write:\n%s", traceback.format_exc())
                    if self.serial_verbose:
                        print(f"[SERIAL OUT-ERR] {out.strip()} -> {e}")
                    return False
            else:
                # 串口不可用 -> 模拟输出
                rospy.logwarn(f"串口未连接，模拟发送命令: {out.strip()}")
                if self.serial_verbose:
                    print(f"[SIMULATED SERIAL OUT] {out.strip()}")
                return False

    def pwm_to_velocity(self, pwm):
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
        try:
            if velocity == 0:
                return self.PWM_NEUTRAL
            elif velocity > 0:
                normalized = velocity / self.max_linear_speed if self.max_linear_speed != 0 else 0
                pwm = self.PWM_NEUTRAL + normalized * (self.PWM_MAX - self.PWM_NEUTRAL)
                return int(max(self.PWM_NEUTRAL, min(self.PWM_MAX, pwm)))
            else:
                normalized = abs(velocity) / self.max_linear_speed if self.max_linear_speed != 0 else 0
                pwm = self.PWM_NEUTRAL - normalized * (self.PWM_NEUTRAL - self.PWM_MIN)
                return int(max(self.PWM_MIN, min(self.PWM_NEUTRAL, pwm)))
        except Exception:
            logger.exception("velocity_to_pwm 出错")
            return self.PWM_NEUTRAL

    def calculate_pwm(self, linear_x, angular_z):
        """根据线速度和角速度计算左右轮 PWM（注意方向修正）"""
        try:
            # 修正方向：前端的 angular_z 与电机映射需要取反（根据你设备测试结果）
            mapped_ang = -float(angular_z)

            base_pwm = self.velocity_to_pwm(linear_x)

            if mapped_ang == 0:
                left_pwm = base_pwm
                right_pwm = base_pwm
            elif mapped_ang > 0:
                turn_adjust = int(abs(mapped_ang) / self.max_angular_speed * self.pwm_turn_diff)
                left_pwm = base_pwm - turn_adjust
                right_pwm = base_pwm + turn_adjust
            else:
                turn_adjust = int(abs(mapped_ang) / self.max_angular_speed * self.pwm_turn_diff)
                left_pwm = base_pwm + turn_adjust
                right_pwm = base_pwm - turn_adjust

            left_pwm = max(self.PWM_MIN, min(self.PWM_MAX, left_pwm))
            right_pwm = max(self.PWM_MIN, min(self.PWM_MAX, right_pwm))
            return left_pwm, right_pwm
        except Exception:
            logger.exception("calculate_pwm 出错")
            return self.PWM_NEUTRAL, self.PWM_NEUTRAL

    def send_velocity_command(self, linear_x, angular_z):
        """通过串口发送速度命令（并发布 cmd_vel topic）"""
        try:
            linear_x = float(linear_x)
            angular_z = float(angular_z)
            linear_x = max(-self.max_linear_speed, min(self.max_linear_speed, linear_x))
            angular_z = max(-self.max_angular_speed, min(self.max_angular_speed, angular_z))

            # 尝试发布 ROS topic（非必须）
            try:
                if self.cmd_vel_pub:
                    twist = Twist()
                    twist.linear.x = linear_x
                    twist.angular.z = angular_z
                    self.cmd_vel_pub.publish(twist)
            except Exception:
                logger.debug("发布 /cmd_vel 失败（可能非 ROS 环境）:\n" + traceback.format_exc())

            self.current_speed = linear_x
            self.current_turn = angular_z

            left_pwm, right_pwm = self.calculate_pwm(linear_x, angular_z)

            # 电机反转校正（如需要）
            if self.REVERSE_LEFT_MOTOR:
                left_pwm = 2 * self.PWM_NEUTRAL - left_pwm
            if self.REVERSE_RIGHT_MOTOR:
                right_pwm = 2 * self.PWM_NEUTRAL - right_pwm

            left_pwm = max(self.PWM_MIN, min(self.PWM_MAX, int(left_pwm)))
            right_pwm = max(self.PWM_MIN, min(self.PWM_MAX, int(right_pwm)))

            command = f"M,{left_pwm},{right_pwm}\n"
            success = self.send_serial_command(command)

            self.current_left_pwm = left_pwm
            self.current_right_pwm = right_pwm

            rospy.loginfo(f"速度命令: linear={linear_x:.2f}, angular={angular_z:.2f}, PWM: L={left_pwm}, R={right_pwm}")
            return success
        except Exception:
            logger.error("发送速度命令失败:\n" + traceback.format_exc())
            return False

    def control_vacuum(self, state):
        """控制负压吸附（串口 + topic）"""
        try:
            desired = bool(state)
            # topic
            try:
                if self.vacuum_pub:
                    msg = Bool()
                    msg.data = desired
                    self.vacuum_pub.publish(msg)
            except Exception:
                logger.debug("发布 /vacuum_control 失败（可能非 ROS 环境）:\n" + traceback.format_exc())

            # 串口命令
            command = '!\n' if desired else '%\n'
            success = self.send_serial_command(command)

            if success:
                self.vacuum_state = desired
                rospy.loginfo(f"负压吸附状态: {'开启' if desired else '关闭'}")
            else:
                rospy.logwarn("尝试设置负压，但串口写入失败或在模拟模式。")

            return success
        except Exception:
            logger.error("control_vacuum 失败:\n" + traceback.format_exc())
            return False

    def emergency_stop(self):
        """紧急停止：发送中性 PWM"""
        try:
            command = f"M,{self.PWM_NEUTRAL},{self.PWM_NEUTRAL}\n"
            self.send_serial_command(command)
            self.current_speed = 0.0
            self.current_turn = 0.0
            self.current_left_pwm = self.PWM_NEUTRAL
            self.current_right_pwm = self.PWM_NEUTRAL
            try:
                if self.cmd_vel_pub:
                    twist = Twist()
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.cmd_vel_pub.publish(twist)
            except Exception:
                logger.debug("发布 /cmd_vel（紧急停止）失败:\n" + traceback.format_exc())
            rospy.loginfo("紧急停止")
        except Exception:
            logger.error("emergency_stop 出现异常:\n" + traceback.format_exc())

    def get_status(self):
        """返回当前状态信息，用于前端显示"""
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

# 单例工厂
robot = None
def get_robot_controller():
    global robot
    if robot is None:
        robot = RobotController()
    return robot
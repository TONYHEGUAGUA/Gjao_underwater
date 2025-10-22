#!/usr/bin/env python
import rospy
import math
import serial
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

class CarTurnTest:
    def __init__(self):
        rospy.init_node('car_turn_test', anonymous=True)
        
        # 串口初始化
        try:
            self.ser = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
            rospy.loginfo("成功连接到串口 /dev/ttyUSB1")
        except Exception as e:
            rospy.logerr(f"无法打开串口 /dev/ttyUSB1: {e}")
            return
        
        # 初始化负压吸附和速度
        self.initialize_car()
        
        # 状态变量
        self.current_pose = None
        self.initial_pose = None
        self.initial_yaw = None
        self.test_state = 0  # 0:等待开始, 1:右转180度, 2:测试完成
        
        # 控制参数
        self.angle_tolerance = 0.05  # 约3度角度容差
        
        # 订阅slam_out_pose话题
        rospy.Subscriber('/slam_out_pose', PoseStamped, self.pose_callback)
        
        rospy.loginfo("右转180度测试节点初始化完成，等待定位数据...")
        
    def initialize_car(self):
        """初始化小车：开启负压吸附并设置速度"""
        # 减速
        for i in range(3):
            self.ser.write('6'.encode())
            rospy.sleep(0.1)
        
        # 开启负压吸附
        self.ser.write('!'.encode())
        rospy.loginfo("负压吸附已开启，速度已降低")
        rospy.sleep(1)
    
    def pose_callback(self, msg):
        """处理定位数据"""
        self.current_pose = msg.pose
        
        if self.initial_pose is None:
            self.initial_pose = msg.pose
            self.initial_yaw = self.get_yaw_from_pose(self.initial_pose)
            rospy.loginfo(f"收到初始位姿，位置: ({self.initial_pose.position.x:.3f}, {self.initial_pose.position.y:.3f})")
            rospy.loginfo(f"初始偏航角: {math.degrees(self.initial_yaw):.2f}°")
            self.test_state = 1
            rospy.loginfo("开始右转180度测试...")
            return
        
        # 执行测试
        self.turn_test_controller()
    
    def get_yaw_from_pose(self, pose):
        """从四元数中提取偏航角"""
        orientation = pose.orientation
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = euler_from_quaternion(quaternion)
        return yaw
    
    def calculate_angle_error(self, target_yaw):
        """计算角度误差"""
        current_yaw = self.get_yaw_from_pose(self.current_pose)
        error = target_yaw - current_yaw
        
        # 将角度误差规范化到[-pi, pi]
        while error > math.pi:
            error -= 2 * math.pi
        while error < -math.pi:
            error += 2 * math.pi
            
        return error, current_yaw
    
    def send_control_command(self, command):
        """发送控制命令"""
        try:
            self.ser.write(command.encode())
            rospy.logdebug(f"发送命令: {command}")
        except Exception as e:
            rospy.logerr(f"发送命令失败: {e}")
    
    def stop_car(self):
        """停止小车"""
        self.send_control_command('0')
        rospy.sleep(0.1)
    
    def turn_test_controller(self):
        """测试控制器"""
        if self.test_state == 0 or self.current_pose is None:
            return
        
        if self.test_state == 1:
            # 右转180度测试
            self.turn_right_180()
        elif self.test_state == 2:
            # 测试完成
            self.stop_car()
            rospy.loginfo("右转180度测试完成!")
            return
    
    def turn_right_180(self):
        """右转180度测试"""
        target_yaw = self.initial_yaw - math.pi  # 右转180度
        
        angle_error, current_yaw = self.calculate_angle_error(target_yaw)
        
        # 打印详细的偏航角信息
        rospy.loginfo(f"初始偏航角: {math.degrees(self.initial_yaw):.2f}°, 当前偏航角: {math.degrees(current_yaw):.2f}°")
        rospy.loginfo(f"目标偏航角: {math.degrees(target_yaw):.2f}°, 角度误差: {math.degrees(angle_error):.2f}°")
        
        # 检查是否达到目标角度
        if abs(angle_error) < self.angle_tolerance:
            self.stop_car()
            rospy.loginfo("成功完成180度右转!")
            rospy.loginfo(f"初始角度: {math.degrees(self.initial_yaw):.2f}°, 最终角度: {math.degrees(current_yaw):.2f}°")
            rospy.loginfo(f"实际转动角度: {math.degrees(current_yaw - self.initial_yaw):.2f}°")
            self.test_state = 2
            return
        
        # 执行右转
        self.send_control_command('C')
        rospy.loginfo_throttle(1, "右转中...")
    
    def run(self):
        """主循环"""
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            rate.sleep()
        
        # 程序退出时停止小车
        self.stop_car()
        self.ser.close()

if __name__ == '__main__':
    try:
        turn_test = CarTurnTest()
        turn_test.run()
    except rospy.ROSInterruptException:
        pass
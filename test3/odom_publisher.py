#!/usr/bin/env python
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Vector3
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class TFToOdom:
    def __init__(self):
        rospy.init_node('tf_to_odom')
        
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.tf_listener = tf.TransformListener()
        
        # 等待 TF 数据
        rospy.loginfo("Waiting for TF data...")
        self.tf_listener.waitForTransform("odom", "base_link", rospy.Time(), rospy.Duration(10.0))
        rospy.loginfo("TF data available!")
        
        # 用于计算速度的变量
        self.last_time = rospy.Time.now()
        self.last_x = 0.0
        self.last_y = 0.0
        self.last_yaw = 0.0
        
        rospy.loginfo("TF to Odometry node started")

    def find_available_frames(self):
        """查找可用的坐标系"""
        try:
            # 获取所有坐标系
            frames = self.tf_listener.getFrameStrings()
            rospy.loginfo("Available frames: %s", frames)
            return frames
        except:
            rospy.logwarn("Could not get frame list")
            return []

    def run(self):
        # 先查找可用的坐标系
        frames = self.find_available_frames()
        
        # 尝试不同的坐标系组合
        frame_combinations = [
            ("odom", "base_link"),
            ("map", "base_link"), 
            ("world", "base_link"),
            ("base_footprint", "base_link")
        ]
        
        rate = rospy.Rate(50)
        
        while not rospy.is_shutdown():
            for global_frame, base_frame in frame_combinations:
                try:
                    # 获取当前的变换
                    (trans, rot) = self.tf_listener.lookupTransform(
                        global_frame, base_frame, rospy.Time(0))
                    
                    current_time = rospy.Time.now()
                    dt = (current_time - self.last_time).to_sec()
                    
                    if dt > 0:
                        # 提取位置和朝向
                        x = trans[0]
                        y = trans[1]
                        
                        # 四元数转欧拉角
                        (roll, pitch, yaw) = euler_from_quaternion(rot)
                        
                        # 计算速度
                        vx = (x - self.last_x) / dt
                        vy = (y - self.last_y) / dt
                        vyaw = (yaw - self.last_yaw) / dt
                        
                        # 发布 Odometry 消息
                        odom = Odometry()
                        odom.header.stamp = current_time
                        odom.header.frame_id = global_frame
                        odom.child_frame_id = base_frame
                        
                        # 设置位置
                        odom.pose.pose.position.x = x
                        odom.pose.pose.position.y = y
                        odom.pose.pose.position.z = trans[2]
                        
                        # 设置朝向
                        odom.pose.pose.orientation.x = rot[0]
                        odom.pose.pose.orientation.y = rot[1]
                        odom.pose.pose.orientation.z = rot[2]
                        odom.pose.pose.orientation.w = rot[3]
                        
                        # 设置速度
                        odom.twist.twist.linear.x = vx
                        odom.twist.twist.linear.y = vy
                        odom.twist.twist.angular.z = vyaw
                        
                        # 发布
                        self.odom_pub.publish(odom)
                        
                        rospy.loginfo_once("Successfully publishing odometry from %s to %s", global_frame, base_frame)
                        
                        # 更新上一时刻的状态
                        self.last_x = x
                        self.last_y = y
                        self.last_yaw = yaw
                        self.last_time = current_time
                        
                        break  # 找到可用的坐标系组合，跳出循环
                    
                except (tf.LookupException, tf.ConnectivityException, 
                       tf.ExtrapolationException) as e:
                    continue  # 尝试下一个坐标系组合
            
            rate.sleep()

if __name__ == '__main__':
    try:
        node = TFToOdom()
        node.run()
    except rospy.ROSInterruptException:
        pass
#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseArray
import time

class SimpleWaypointsTester:
    def __init__(self):
        rospy.init_node('simple_waypoints_tester')
        
        self.waypoints_sub = rospy.Subscriber(
            '/move_base_simple/waypoints', 
            PoseArray, 
            self.callback
        )
        
        self.count = 0
        print("🚀 简单waypoints测试节点已启动...")
    
    def callback(self, msg):
        self.count += 1
        num_waypoints = len(msg.poses)
        
        print(f"\n✅ 收到第 {self.count} 个waypoints消息")
        print(f"   序列号: {msg.header.seq}")
        print(f"   时间戳: {msg.header.stamp.secs}.{msg.header.stamp.nsecs}")
        print(f"   坐标系: {msg.header.frame_id}")
        print(f"   Waypoints数量: {num_waypoints}")
        
        if num_waypoints > 0:
            for i, pose in enumerate(msg.poses):
                print(f"   点{i+1}: ({pose.position.x:.2f}, {pose.position.y:.2f})")
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        tester = SimpleWaypointsTester()
        tester.run()
    except rospy.ROSInterruptException:
        print(f"\n📊 测试结束，共接收 {tester.count} 次消息")
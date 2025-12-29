#!/usr/bin/env python3
# app.py - Flask主程序

from flask import Flask, render_template, Response, jsonify, request
from flask_socketio import SocketIO, emit
from flask_cors import CORS
import rospy
import threading
import time
import json
import os

# ROS消息类型
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Path
from std_msgs.msg import String, Bool
import tf

# 自定义模块
from camera_stream import CameraStream
from robot_controller import RobotController

app = Flask(__name__)
CORS(app)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

# 全局变量
robot_controller = None
camera_stream = None
ros_thread = None
current_goal = None
goal_history = []

class WebControlNode:
    def __init__(self):
        self.cmd_vel_pub = None
        self.vacuum_pub = None
        self.goal_sub = None
        self.pose_sub = None
        self.current_pose = None
        self.ros_initialized = False
        
    def init_ros(self):
        """初始化ROS节点"""
        try:
            if not rospy.get_node_uri():
                rospy.init_node('web_control_node', anonymous=True, disable_signals=True)
            
            # 发布器
            self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
            self.vacuum_pub = rospy.Publisher('/vacuum_control', Bool, queue_size=10)
            
            # 订阅器 - Foxglove设置的目标点
            self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, 
                                            self.goal_callback)
            
            # 订阅机器人当前位置（如果有）
            self.pose_sub = rospy.Subscriber('/robot_pose', Point, 
                                            self.pose_callback, queue_size=1)
            
            self.ros_initialized = True
            print("ROS节点初始化成功")
            
        except Exception as e:
            print(f"ROS初始化失败: {e}")
            self.ros_initialized = False
    
    def goal_callback(self, msg):
        """处理Foxglove设置的目标点"""
        global current_goal
        try:
            # 提取目标点信息
            goal_point = {
                'x': msg.pose.position.x,
                'y': msg.pose.position.y,
                'z': msg.pose.position.z,
                'orientation': {
                    'x': msg.pose.orientation.x,
                    'y': msg.pose.orientation.y,
                    'z': msg.pose.orientation.z,
                    'w': msg.pose.orientation.w
                },
                'frame_id': msg.header.frame_id,
                'timestamp': rospy.get_time()
            }
            
            current_goal = goal_point
            goal_history.append(goal_point)
            
            # 通过WebSocket发送给前端
            socketio.emit('new_goal', goal_point)
            print(f"收到新目标点: {goal_point}")
            
        except Exception as e:
            print(f"处理目标点错误: {e}")
    
    def pose_callback(self, msg):
        """更新机器人当前位置"""
        self.current_pose = {
            'x': msg.x,
            'y': msg.y,
            'z': msg.z
        }
        
        # 发送给前端
        socketio.emit('robot_pose', self.current_pose)
    
    def send_velocity(self, linear_x, angular_z):
        """发送速度命令"""
        if not self.ros_initialized:
            print("ROS未初始化，无法发送速度命令")
            return False
        
        try:
            twist = Twist()
            twist.linear.x = linear_x
            twist.angular.z = angular_z
            self.cmd_vel_pub.publish(twist)
            return True
        except Exception as e:
            print(f"发送速度命令失败: {e}")
            return False
    
    def control_vacuum(self, state):
        """控制负压吸附"""
        if not self.ros_initialized:
            print("ROS未初始化，无法控制负压吸附")
            return False
        
        try:
            msg = Bool()
            msg.data = state  # True: 开启, False: 关闭
            self.vacuum_pub.publish(msg)
            return True
        except Exception as e:
            print(f"控制负压吸附失败: {e}")
            return False

# 创建ROS控制节点
web_node = WebControlNode()

def init_ros_in_thread():
    """在单独线程中初始化ROS"""
    global web_node
    time.sleep(1)  # 等待Flask启动
    web_node.init_ros()
    
    # 保持ROS运行
    rospy.spin()

@app.route('/')
def index():
    """主页面"""
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    """视频流路由"""
    global camera_stream
    if camera_stream is None:
        camera_stream = CameraStream()
    return Response(camera_stream.generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/api/status')
def get_status():
    """获取系统状态"""
    status = {
        'ros_connected': web_node.ros_initialized,
        'camera_available': camera_stream is not None,
        'current_goal': current_goal,
        'robot_pose': web_node.current_pose,
        'goal_history_count': len(goal_history)
    }
    return jsonify(status)

@app.route('/api/control/vacuum', methods=['POST'])
def control_vacuum():
    """控制负压吸附API"""
    data = request.json
    state = data.get('state', False)
    
    success = web_node.control_vacuum(state)
    return jsonify({'success': success, 'state': state})

@socketio.on('connect')
def handle_connect():
    """客户端连接"""
    print(f'客户端连接: {request.sid}')
    emit('connected', {'message': 'Connected to robot control system'})

@socketio.on('disconnect')
def handle_disconnect():
    """客户端断开连接"""
    print(f'客户端断开: {request.sid}')

@socketio.on('control_command')
def handle_control_command(data):
    """处理控制命令"""
    try:
        command = data.get('command', '')
        linear_x = data.get('linear_x', 0.0)
        angular_z = data.get('angular_z', 0.0)
        
        print(f"收到控制命令: {command}, linear_x: {linear_x}, angular_z: {angular_z}")
        
        # 发送速度命令
        success = web_node.send_velocity(linear_x, angular_z)
        
        # 发送响应
        emit('control_response', {
            'success': success,
            'command': command,
            'timestamp': time.time()
        })
        
    except Exception as e:
        print(f"处理控制命令错误: {e}")
        emit('control_response', {
            'success': False,
            'error': str(e)
        })

@socketio.on('vacuum_control')
def handle_vacuum_control(data):
    """处理负压吸附控制"""
    try:
        state = data.get('state', False)
        success = web_node.control_vacuum(state)
        
        emit('vacuum_response', {
            'success': success,
            'state': state,
            'timestamp': time.time()
        })
        
    except Exception as e:
        print(f"处理负压吸附控制错误: {e}")
        emit('vacuum_response', {
            'success': False,
            'error': str(e)
        })

def cleanup():
    """清理资源"""
    global camera_stream
    if camera_stream:
        camera_stream.stop()
    print("清理完成")

if __name__ == '__main__':
    try:
        # 启动ROS线程
        ros_thread = threading.Thread(target=init_ros_in_thread, daemon=True)
        ros_thread.start()
        
        # 启动Flask服务器
        print("启动Flask服务器...")
        print("访问地址: http://树莓派IP:5000")
        socketio.run(app, host='0.0.0.0', port=5000, debug=False)
        
    except KeyboardInterrupt:
        print("正在关闭服务器...")
        cleanup()
    except Exception as e:
        print(f"服务器错误: {e}")
        cleanup()
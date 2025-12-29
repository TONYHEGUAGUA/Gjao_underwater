#!/usr/bin/env python3
# app.py - Flask 主程序（已增强）
# 说明：
# - 在 ROS 初始化完成后延迟创建 RobotController 单例（串口控制）
# - 优先使用 RobotController 进行直接串口控制，失败时回退到 ROS topic 发布
# - 增加周期性状态广播 (robot_status)，并在每次控制后立即广播一次
# - 更稳健的异常/traceback 打印，便于调试

from flask import Flask, render_template, Response, jsonify, request
from flask_socketio import SocketIO, emit
from flask_cors import CORS
import rospy
import threading
import time
import traceback

# ROS消息类型
from geometry_msgs.msg import Twist, PoseStamped, Point
from std_msgs.msg import Bool

# 自定义模块
from camera_stream import CameraStream
# 注意：不要在模块导入阶段创建 RobotController，延迟在 ROS 初始化后创建

app = Flask(__name__)
CORS(app)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

# 全局变量
robot_controller = None  # RobotController 单例（将在 ROS 初始化后创建）
camera_stream = None
ros_thread = None
status_thread = None
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
        """初始化 ROS 节点"""
        try:
            # 如果没有 node uri，说明还未初始化
            if not rospy.get_node_uri():
                rospy.init_node('web_control_node', anonymous=True, disable_signals=True)
            
            # 发布器
            self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
            self.vacuum_pub = rospy.Publisher('/vacuum_control', Bool, queue_size=10)
            
            # 订阅器 - Foxglove设置的目标点
            self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
            # 订阅机器人当前位置（如果有）
            self.pose_sub = rospy.Subscriber('/robot_pose', Point, self.pose_callback, queue_size=1)
            
            self.ros_initialized = True
            print("ROS节点初始化成功")
        except Exception as e:
            print(f"ROS初始化失败: {e}")
            traceback.print_exc()
            self.ros_initialized = False
    
    def goal_callback(self, msg):
        """处理 Foxglove 设置的目标点"""
        global current_goal
        try:
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
            socketio.emit('new_goal', goal_point)
            print(f"收到新目标点: {goal_point}")
        except Exception as e:
            print(f"处理目标点错误: {e}")
            traceback.print_exc()
    
    def pose_callback(self, msg):
        """更新机器人当前位置并广播"""
        try:
            self.current_pose = {'x': msg.x, 'y': msg.y, 'z': msg.z}
            socketio.emit('robot_pose', self.current_pose)
        except Exception:
            traceback.print_exc()
    
    def send_velocity(self, linear_x, angular_z):
        """通过 ROS topic 发布速度命令（回退路径）"""
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
            traceback.print_exc()
            return False
    
    def control_vacuum(self, state):
        """通过 ROS topic 发布负压控制（回退路径）"""
        if not self.ros_initialized:
            print("ROS未初始化，无法控制负压吸附")
            return False
        try:
            msg = Bool()
            msg.data = bool(state)
            self.vacuum_pub.publish(msg)
            return True
        except Exception as e:
            print(f"控制负压吸附失败: {e}")
            traceback.print_exc()
            return False

# 创建 ROS 控制节点实例（不初始化 rospy）
web_node = WebControlNode()

def init_ros_in_thread():
    """在单独线程中初始化 ROS，并在成功后创建 RobotController 单例"""
    global web_node, robot_controller
    time.sleep(1)  # 等待 Flask 启动完成
    web_node.init_ros()
    
    # 在 ROS 初始化完成后再创建 RobotController 单例（延迟导入）
    try:
        from robot_controller import get_robot_controller
        robot_controller = get_robot_controller()
        print("RobotController 已创建 (将输出串口调试信息，如串口可用)")
    except Exception as e:
        print("创建 RobotController 失败:", e)
        traceback.print_exc()
        robot_controller = None

    # 保持 ROS 运行
    try:
        rospy.spin()
    except Exception:
        pass

def status_broadcaster(interval=0.2):
    """周期性广播机器人状态给前端（事件名: robot_status）"""
    global robot_controller, web_node
    while True:
        try:
            status = {}
            # 优先使用 RobotController 的状态
            if robot_controller is not None:
                try:
                    status = robot_controller.get_status()
                except Exception:
                    status = {}
            # 补充 ROS 层信息
            try:
                status['robot_pose'] = web_node.current_pose
                status['ros_connected'] = web_node.ros_initialized
            except Exception:
                pass
            socketio.emit('robot_status', status)
        except Exception:
            traceback.print_exc()
        time.sleep(interval)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    global camera_stream
    if camera_stream is None:
        camera_stream = CameraStream()
    return Response(camera_stream.generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/api/status')
def get_status():
    status = {
        'ros_connected': web_node.ros_initialized,
        'camera_available': camera_stream is not None,
        'current_goal': current_goal,
        'robot_pose': web_node.current_pose,
        'goal_history_count': len(goal_history),
        'robot_controller_created': robot_controller is not None
    }
    return jsonify(status)

@app.route('/api/control/vacuum', methods=['POST'])
def control_vacuum():
    """HTTP API 控制负压：优先串口，回退 topic"""
    data = request.json or {}
    state = data.get('state', False)
    rc_success = None
    if robot_controller is not None:
        try:
            rc_success = robot_controller.control_vacuum(state)
        except Exception:
            traceback.print_exc()
            rc_success = None
    topic_success = web_node.control_vacuum(state)
    final_success = rc_success if rc_success is not None else topic_success
    # 立即广播状态
    try:
        socketio.emit('robot_status', robot_controller.get_status() if robot_controller else {'robot_pose': web_node.current_pose})
    except Exception:
        pass
    return jsonify({'success': bool(final_success), 'state': bool(state)})

@socketio.on('connect')
def handle_connect():
    print(f'客户端连接: {request.sid}')
    emit('connected', {'message': 'Connected to robot control system'})

@socketio.on('disconnect')
def handle_disconnect():
    print(f'客户端断开: {request.sid}')

@socketio.on('control_command')
def handle_control_command(data):
    """
    处理控制命令
    - 优先对 'manual' / 'stop' / 'emergency_stop' 使用 RobotController 串口接口（如果存在）
    - 否则回退到通过 ROS topic 发布
    """
    try:
        command = data.get('command', '')
        linear_x = data.get('linear_x', 0.0)
        angular_z = data.get('angular_z', 0.0)
        print(f"收到控制命令: {command}, linear_x: {linear_x}, angular_z: {angular_z}")
        
        rc_success = None
        if robot_controller is not None:
            try:
                if command == 'manual':
                    rc_success = robot_controller.send_velocity_command(linear_x, angular_z)
                elif command == 'stop':
                    robot_controller.emergency_stop()
                    rc_success = True
                elif command == 'emergency_stop':
                    robot_controller.emergency_stop()
                    rc_success = True
            except Exception:
                traceback.print_exc()
                rc_success = None

        topic_success = web_node.send_velocity(linear_x, angular_z)
        final_success = rc_success if rc_success is not None else topic_success

        # 立即广播最新状态
        try:
            socketio.emit('robot_status', robot_controller.get_status() if robot_controller else {'robot_pose': web_node.current_pose})
        except Exception:
            pass

        emit('control_response', {
            'success': bool(final_success),
            'command': command,
            'timestamp': time.time()
        })
    except Exception as e:
        print(f"处理控制命令错误: {e}")
        traceback.print_exc()
        emit('control_response', {'success': False, 'error': str(e)})

@socketio.on('vacuum_control')
def handle_vacuum_control(data):
    """处理负压吸附控制（优先串口，回退 topic）"""
    try:
        print(f"收到负压吸附控制: {data}")
        state = data.get('state', False) if isinstance(data, dict) else bool(data)

        rc_success = None
        if robot_controller is not None:
            try:
                rc_success = robot_controller.control_vacuum(state)
            except Exception:
                traceback.print_exc()
                rc_success = None

        topic_success = web_node.control_vacuum(state)
        final_success = rc_success if rc_success is not None else topic_success

        # 立即广播最新状态
        try:
            socketio.emit('robot_status', robot_controller.get_status() if robot_controller else {'robot_pose': web_node.current_pose})
        except Exception:
            pass

        emit('vacuum_response', {'success': bool(final_success), 'state': bool(state), 'timestamp': time.time()})
    except Exception as e:
        print(f"处理负压吸附控制错误: {e}")
        traceback.print_exc()
        emit('vacuum_response', {'success': False, 'error': str(e)})

def cleanup():
    global camera_stream
    if camera_stream:
        camera_stream.stop()
    print("清理完成")

if __name__ == '__main__':
    try:
        # 启动 ROS 初始化线程（会创建 RobotController）
        ros_thread = threading.Thread(target=init_ros_in_thread, daemon=True)
        ros_thread.start()

        # 启动状态广播线程（emit robot_status）
        status_thread = threading.Thread(target=status_broadcaster, kwargs={'interval': 0.2}, daemon=True)
        status_thread.start()
        
        # 启动 Flask/SocketIO 服务器
        print("启动Flask服务器...")
        print("访问地址: http://树莓派IP:5000")
        socketio.run(app, host='0.0.0.0', port=5000, debug=False)
    except KeyboardInterrupt:
        print("正在关闭服务器...")
        cleanup()
    except Exception as e:
        print(f"服务器错误: {e}")
        traceback.print_exc()
        cleanup()
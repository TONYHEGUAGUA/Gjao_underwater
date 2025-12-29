#!/usr/bin/env python3
# app.py - Flask 主程序（增强版）
# 说明：
# - ROS 初始化后延迟创建 RobotController 单例（串口控制）
# - 优先使用 RobotController 进行直接串口控制，失败时回退到通过 ROS topic 发布
# - 增加周期性状态广播 (robot_status)，并在每次控制后立即广播一次
# - 订阅常见位姿话题并使用 tf.TransformListener 查询 map->base_link（若可用），将位姿发送到前端

from flask import Flask, render_template, Response, jsonify, request
from flask_socketio import SocketIO, emit
from flask_cors import CORS
import rospy
import threading
import time
import traceback

# ROS 消息类型
from geometry_msgs.msg import Twist, PoseStamped, Point
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

# tf
import tf
import tf.transformations as tft

# 自定义模块
from camera_stream import CameraStream
# RobotController 延迟导入 / 延迟创建，见 init_ros_in_thread

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
        # topic subscribers (may be None if not present)
        self.pose_sub = None
        self.odom_sub = None
        self.pose_stamped_sub = None

        self.current_pose = None  # dict: {x,y,z,orientation:{x,y,z,w}}
        self.ros_initialized = False

        # tf listener and polling thread
        self.tf_listener = None
        self._tf_thread = None
        self._tf_thread_stop = threading.Event()

    def init_ros(self):
        """初始化 ROS 节点并订阅常见的位置相关 topic，同时启动 tf 轮询线程"""
        try:
            if not rospy.get_node_uri():
                rospy.init_node('web_control_node', anonymous=True, disable_signals=True)

            # Publishers
            self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
            self.vacuum_pub = rospy.Publisher('/vacuum_control', Bool, queue_size=10)

            # Subscriptions
            try:
                self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
            except Exception:
                rospy.logwarn("订阅 /move_base_simple/goal 失败")

            # Try Point topic (legacy)
            try:
                self.pose_sub = rospy.Subscriber('/robot_pose', Point, self.pose_point_callback, queue_size=1)
            except Exception:
                rospy.loginfo("/robot_pose(Point) 不存在或订阅失败")

            # /odom (Odometry)
            try:
                self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=1)
            except Exception:
                rospy.loginfo("/odom(Odometry) 不存在或订阅失败")

            # /amcl_pose or other PoseWithCovarianceStamped
            try:
                self.pose_stamped_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_with_cov_callback, queue_size=1)
            except Exception:
                rospy.loginfo("/amcl_pose(PoseWithCovarianceStamped) 不存在或订阅失败")

            # Initialize tf listener and start polling thread
            try:
                self.tf_listener = tf.TransformListener()
                # give listener a short time to fill buffer
                time.sleep(0.1)
                self._tf_thread_stop.clear()
                self._tf_thread = threading.Thread(target=self._tf_polling_loop, daemon=True)
                self._tf_thread.start()
                rospy.loginfo("tf listener 启动")
            except Exception:
                rospy.logwarn("无法初始化 tf listener:\n" + traceback.format_exc())
                self.tf_listener = None

            self.ros_initialized = True
            print("ROS节点初始化成功")
        except Exception:
            print("ROS 初始化失败")
            traceback.print_exc()
            self.ros_initialized = False

    # ---- topic callbacks ----
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
        except Exception:
            rospy.logwarn("goal_callback 处理失败:\n" + traceback.format_exc())

    def pose_point_callback(self, msg):
        """处理 /robot_pose (geometry_msgs/Point)"""
        try:
            self.current_pose = {'x': msg.x, 'y': msg.y, 'z': msg.z}
            socketio.emit('robot_pose', self.current_pose)
        except Exception:
            rospy.logwarn("pose_point_callback 处理失败:\n" + traceback.format_exc())

    def odom_callback(self, msg):
        """处理 /odom (nav_msgs/Odometry)"""
        try:
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            self.current_pose = {
                'x': p.x, 'y': p.y, 'z': p.z,
                'orientation': {'x': q.x, 'y': q.y, 'z': q.z, 'w': q.w}
            }
            socketio.emit('robot_pose', self.current_pose)
        except Exception:
            rospy.logwarn("odom_callback 处理失败:\n" + traceback.format_exc())

    def pose_with_cov_callback(self, msg):
        """处理 /amcl_pose (geometry_msgs/PoseWithCovarianceStamped)"""
        try:
            p = msg.pose.pose
            self.current_pose = {
                'x': p.position.x,
                'y': p.position.y,
                'z': p.position.z,
                'orientation': {'x': p.orientation.x, 'y': p.orientation.y, 'z': p.orientation.z, 'w': p.orientation.w}
            }
            socketio.emit('robot_pose', self.current_pose)
        except Exception:
            rospy.logwarn("pose_with_cov_callback 处理失败:\n" + traceback.format_exc())

    # ---- tf polling ----
    def _tf_polling_loop(self, rate_hz=10):
        """
        周期性从 tf buffer 查询 transform 并更新 current_pose。
        优先尝试: map -> base_link (或 base_footprint, base)
        若不可用，尝试 map->odom 与 odom->base_link 组合。
        """
        rate = rospy.Rate(rate_hz)
        possible_base_frames = ['base_link', 'base_footprint', 'base']
        try:
            while not rospy.is_shutdown() and not self._tf_thread_stop.is_set():
                try:
                    found = False
                    if not self.tf_listener:
                        break
                    # 优先直接 map -> base
                    for base in possible_base_frames:
                        try:
                            (trans, rot) = self.tf_listener.lookupTransform('map', base, rospy.Time(0))
                            self.current_pose = {
                                'x': float(trans[0]),
                                'y': float(trans[1]),
                                'z': float(trans[2]),
                                'orientation': {'x': float(rot[0]), 'y': float(rot[1]), 'z': float(rot[2]), 'w': float(rot[3])}
                            }
                            socketio.emit('robot_pose', self.current_pose)
                            found = True
                            break
                        except Exception:
                            continue

                    if not found:
                        # 尝试 map->odom 和 odom->base 的组合
                        try:
                            (trans_m_odom, rot_m_odom) = self.tf_listener.lookupTransform('map', 'odom', rospy.Time(0))
                            for base in possible_base_frames:
                                try:
                                    (trans_odom_base, rot_odom_base) = self.tf_listener.lookupTransform('odom', base, rospy.Time(0))
                                    # 组合变换：map->base = map->odom * odom->base
                                    mat_m_odom = tft.quaternion_matrix(rot_m_odom)
                                    mat_m_odom[0:3, 3] = trans_m_odom
                                    mat_odom_base = tft.quaternion_matrix(rot_odom_base)
                                    mat_odom_base[0:3, 3] = trans_odom_base
                                    mat_m_base = mat_m_odom.dot(mat_odom_base)
                                    trans = mat_m_base[0:3, 3]
                                    quat = tft.quaternion_from_matrix(mat_m_base)
                                    self.current_pose = {
                                        'x': float(trans[0]),
                                        'y': float(trans[1]),
                                        'z': float(trans[2]),
                                        'orientation': {'x': float(quat[0]), 'y': float(quat[1]), 'z': float(quat[2]), 'w': float(quat[3])}
                                    }
                                    socketio.emit('robot_pose', self.current_pose)
                                    found = True
                                    break
                                except Exception:
                                    continue
                        except Exception:
                            # map->odom 不可用
                            pass
                    # 下一轮
                except Exception:
                    rospy.logdebug("tf polling 查询异常:\n" + traceback.format_exc())
                rate.sleep()
        except Exception:
            rospy.logdebug("tf polling 主循环退出:\n" + traceback.format_exc())

    def shutdown(self):
        """停止 tf 轮询线程"""
        try:
            self._tf_thread_stop.set()
            if self._tf_thread and self._tf_thread.is_alive():
                self._tf_thread.join(0.5)
        except Exception:
            pass

# 创建 WebControlNode 实例（不在此处初始化 rospy）
web_node = WebControlNode()

def init_ros_in_thread():
    """在单独线程中初始化 ROS，并在成功后创建 RobotController 单例"""
    global web_node, robot_controller
    time.sleep(1)  # 等待 Flask 启动完成
    web_node.init_ros()

    # 延迟导入并创建 RobotController 单例
    try:
        from robot_controller import get_robot_controller
        robot_controller = get_robot_controller()
        print("RobotController 已创建 (将输出串口调试信息，如串口可用)")
    except Exception:
        print("创建 RobotController 失败:")
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
            # 优先使用 RobotController 的状态（包含 PWM、serial_connected 等）
            if robot_controller is not None:
                try:
                    status = robot_controller.get_status()
                except Exception:
                    status = {}
            # 补充 ROS 层信息（位置/ros_connected）
            try:
                status['robot_pose'] = web_node.current_pose
                status['ros_connected'] = web_node.ros_initialized
            except Exception:
                pass
            socketio.emit('robot_status', status)
        except Exception:
            traceback.print_exc()
        time.sleep(interval)

# ---- Flask routes and SocketIO handlers ----
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
    except Exception:
        traceback.print_exc()
        emit('control_response', {'success': False, 'error': 'server_error'})

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
    except Exception:
        traceback.print_exc()
        emit('vacuum_response', {'success': False, 'error': 'server_error'})

def cleanup():
    global camera_stream
    try:
        web_node.shutdown()
    except Exception:
        pass
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
    except Exception:
        traceback.print_exc()
        cleanup()
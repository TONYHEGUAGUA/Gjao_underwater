#!/usr/bin/env python3
# camera_stream.py - 摄像头视频流处理

import cv2
import threading
import time
import numpy as np
from queue import Queue

class CameraStream:
    def __init__(self, device_id=0, width=640, height=480, fps=15):
        """
        初始化摄像头流
        device_id: 摄像头设备ID，默认0 (/dev/video0)
        """
        self.device_id = device_id
        self.width = width
        self.height = height
        self.fps = fps
        
        self.camera = None
        self.frame_queue = Queue(maxsize=2)
        self.running = False
        self.capture_thread = None
        
        # 尝试初始化摄像头
        self.init_camera()
        
    def init_camera(self):
        """初始化摄像头"""
        try:
            self.camera = cv2.VideoCapture(self.device_id)
            
            if not self.camera.isOpened():
                # 尝试使用设备路径
                self.camera = cv2.VideoCapture(f'/dev/video{self.device_id}')
                
            if not self.camera.isOpened():
                print(f"无法打开摄像头 /dev/video{self.device_id}")
                return False
            
            # 设置摄像头参数
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.camera.set(cv2.CAP_PROP_FPS, self.fps)
            
            # 获取实际参数
            actual_width = self.camera.get(cv2.CAP_PROP_FRAME_WIDTH)
            actual_height = self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT)
            actual_fps = self.camera.get(cv2.CAP_PROP_FPS)
            
            print(f"摄像头初始化成功: {actual_width}x{actual_height} @ {actual_fps}FPS")
            return True
            
        except Exception as e:
            print(f"摄像头初始化失败: {e}")
            return False
    
    def start_capture(self):
        """开始捕获视频帧"""
        if self.running:
            return
        
        self.running = True
        self.capture_thread = threading.Thread(target=self._capture_frames, daemon=True)
        self.capture_thread.start()
        print("开始捕获视频帧")
    
    def _capture_frames(self):
        """捕获视频帧（内部线程函数）"""
        while self.running:
            try:
                if self.camera and self.camera.isOpened():
                    ret, frame = self.camera.read()
                    
                    if ret:
                        # 压缩图像以减小网络传输
                        frame = self.process_frame(frame)
                        
                        # 放入队列（如果队列已满，移除最旧的一帧）
                        if self.frame_queue.full():
                            self.frame_queue.get()
                        self.frame_queue.put(frame)
                    else:
                        print("读取摄像头帧失败")
                        time.sleep(0.1)
                
                time.sleep(1.0 / self.fps)
                
            except Exception as e:
                print(f"捕获帧错误: {e}")
                time.sleep(0.1)
    
    def process_frame(self, frame):
        """处理视频帧（压缩、添加信息等）"""
        try:
            # 调整大小
            if frame.shape[1] != self.width or frame.shape[0] != self.height:
                frame = cv2.resize(frame, (self.width, self.height))
        
            # 镜像翻转（修正左右相反问题）
            frame = cv2.flip(frame, 1)  # 1表示水平翻转
        
            # 添加时间戳
            timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
            cv2.putText(frame, timestamp, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
            # 添加分辨率信息
            info = f"{self.width}x{self.height}"
            cv2.putText(frame, info, (10, self.height - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
            return frame
        
        except Exception as e:
            print(f"处理帧错误: {e}")
            return frame
    
    def get_frame(self):
        """获取最新视频帧"""
        try:
            if not self.frame_queue.empty():
                return self.frame_queue.get()
            return None
        except:
            return None
    
    def generate_frames(self):
        """生成视频流（用于HTTP响应）"""
        self.start_capture()
        
        while self.running:
            frame = self.get_frame()
            
            if frame is not None:
                # 编码为JPEG
                ret, buffer = cv2.imencode('.jpg', frame, 
                                          [cv2.IMWRITE_JPEG_QUALITY, 70])
                
                if ret:
                    yield (b'--frame\r\n'
                          b'Content-Type: image/jpeg\r\n\r\n' + 
                          buffer.tobytes() + b'\r\n')
                else:
                    # 发送黑色帧
                    black_frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
                    cv2.putText(black_frame, "No Camera Feed", 
                               (50, self.height//2),
                               cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                    ret, buffer = cv2.imencode('.jpg', black_frame)
                    yield (b'--frame\r\n'
                          b'Content-Type: image/jpeg\r\n\r\n' + 
                          buffer.tobytes() + b'\r\n')
            
            time.sleep(1.0 / self.fps)
    
    def stop(self):
        """停止摄像头"""
        self.running = False
        
        if self.capture_thread:
            self.capture_thread.join(timeout=2.0)
        
        if self.camera:
            self.camera.release()
        
        print("摄像头已停止")

# 全局摄像头实例
camera = None

def get_camera_instance():
    """获取摄像头实例（单例模式）"""
    global camera
    if camera is None:
        camera = CameraStream()
    return camera
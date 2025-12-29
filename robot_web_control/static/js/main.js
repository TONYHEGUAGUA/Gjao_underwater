// main.js - 前端控制逻辑（已修改）
//
// 主要改动：
// - 监听后端周期广播事件 'robot_status'，并用其数据更新页面上的 "当前速度 / 当前转向 / 机器人位置 / 左右 PWM / 串口状态"
// - 将 controlParams.maxLinear 默认值调整为 0.8（与后端默认一致）
// - 在收到 robot_status 时更新地图/位置显示，保持原有 control_response / robot_pose 兼容

// 全局变量
let socket = null;
let isConnected = false;
let controlInterval = null;
let currentKeys = {
    move: null,    // 'forward', 'backward', null
    turn: null     // 'left', 'right', null
};
let mapCanvas = null;
let mapContext = null;
let goals = [];
let robotPose = { x: 0, y: 0, z: 0 };
let controlParams = {
    speed: 0.5,     // 0-1 (slider % -> value)
    turn: 0.5,      // 0-1
    maxLinear: 0.8,   // 与后端默认一致（m/s）
    maxAngular: 1.0
};

// 初始化函数
function init() {
    console.log("初始化水下机器人控制界面");
    
    // 初始化WebSocket连接
    initWebSocket();
    
    // 初始化地图画布
    initMapCanvas();
    
    // 初始化键盘控制
    initKeyboardControls();
    
    // 初始化系统状态检查
    checkSystemStatus();
    
    // 更新界面状态
    updateUI();
    
    console.log("界面初始化完成");
}

// WebSocket连接
function initWebSocket() {
    // 获取服务器地址（自动检测）
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const hostname = window.location.hostname;
    const port = window.location.port || '5000';
    const wsUrl = `${protocol}//${hostname}:${port}`;
    
    console.log(`连接到WebSocket服务器: ${wsUrl}`);
    
    // 使用 socket.io 客户端（页面 head 已加载 socket.io.js）
    // io(wsUrl) 会自动与服务器协商 transport（polling 或 websocket）
    socket = io(wsUrl);
    
    // 连接事件
    socket.on('connect', () => {
        console.log('WebSocket连接成功');
        isConnected = true;
        updateConnectionStatus(true);
        logMessage('系统', 'WebSocket连接成功');
    });
    
    socket.on('disconnect', () => {
        console.log('WebSocket断开连接');
        isConnected = false;
        updateConnectionStatus(false);
        logMessage('系统', 'WebSocket连接断开');
    });
    
    socket.on('connected', (data) => {
        console.log('服务器连接确认:', data);
        logMessage('系统', '服务器连接确认');
    });
    
    socket.on('control_response', (data) => {
        console.log('控制响应:', data);
        if (data && data.success) {
            // control_response 可能包含命令信息（来自按键），用来更新部分 UI（短时）
            updateRobotStatusFromCommand(data);
        }
    });
    
    socket.on('vacuum_response', (data) => {
        console.log('负压吸附响应:', data);
        if (data && data.success) {
            updateVacuumStatus(data.state);
            logMessage('负压吸附', data.state ? '已开启' : '已关闭');
        }
    });
    
    socket.on('new_goal', (goal) => {
        console.log('收到新目标点:', goal);
        addGoalToMap(goal);
        logMessage('目标点', `收到新目标点 (${goal.x.toFixed(2)}, ${goal.y.toFixed(2)})`);
    });
    
    // 后端周期广播状态（robot_status）
    socket.on('robot_status', (status) => {
        // status 例：{speed, turn, robot_pose:{x,y,z}, left_pwm, right_pwm, serial_connected}
        try {
            if (!status) return;
            // 更新速度/转向显示
            if ('speed' in status) {
                const speedEl = document.getElementById('current-speed');
                if (speedEl) speedEl.textContent = (status.speed !== undefined && status.speed !== null) ? status.speed.toFixed(2) + ' m/s' : '0.00 m/s';
            }
            if ('turn' in status) {
                const turnEl = document.getElementById('current-turn');
                if (turnEl) turnEl.textContent = (status.turn !== undefined && status.turn !== null) ? status.turn.toFixed(2) + ' rad/s' : '0.00 rad/s';
            }
            // 更新 PWM
            if ('left_pwm' in status) {
                const l = document.getElementById('left-pwm');
                if (l) l.textContent = status.left_pwm;
            }
            if ('right_pwm' in status) {
                const r = document.getElementById('right-pwm');
                if (r) r.textContent = status.right_pwm;
            }
            // 串口/连接信息
            if ('serial_connected' in status) {
                const ss = document.getElementById('serial-status');
                if (ss) ss.textContent = status.serial_connected ? '已连接' : '未连接/模拟';
            }
            // 位置
            if (status.robot_pose) {
                robotPose = status.robot_pose;
                updateRobotPositionDisplay();
                drawRobotOnMap();
            }
        } catch (e) {
            console.error('处理 robot_status 失败', e);
        }
    });
    
    socket.on('robot_pose', (pose) => {
        // 保持兼容：有些后端也发送单独 robot_pose 事件
        try {
            if (!pose) return;
            robotPose = pose;
            updateRobotPositionDisplay();
            drawRobotOnMap();
        } catch (e) { console.error(e); }
    });
    
    // 错误处理
    socket.on('connect_error', (error) => {
        console.error('WebSocket连接错误:', error);
        logMessage('错误', `连接失败: ${error && error.message ? error.message : error}`);
        updateConnectionStatus(false);
    });
}

// 初始化地图画布
function initMapCanvas() {
    mapCanvas = document.getElementById('map-canvas');
    if (!mapCanvas) {
        console.error('找不到地图画布元素');
        return;
    }
    
    mapContext = mapCanvas.getContext('2d');
    
    // 设置初始地图视图
    drawMapGrid();
    
    // 监听窗口大小变化
    window.addEventListener('resize', resizeMapCanvas);
    
    // 初始调整大小
    resizeMapCanvas();
}

// 绘制地图网格
function drawMapGrid() {
    const width = mapCanvas.width;
    const height = mapCanvas.height;
    
    // 清空画布
    mapContext.clearRect(0, 0, width, height);
    
    // 绘制背景
    mapContext.fillStyle = '#1a1a2e';
    mapContext.fillRect(0, 0, width, height);
    
    // 绘制网格
    mapContext.strokeStyle = '#2d2d44';
    mapContext.lineWidth = 1;
    
    const gridSize = 50; // 网格大小（像素）
    
    // 垂直网格线
    for (let x = 0; x <= width; x += gridSize) {
        mapContext.beginPath();
        mapContext.moveTo(x, 0);
        mapContext.lineTo(x, height);
        mapContext.stroke();
    }
    
    // 水平网格线
    for (let y = 0; y <= height; y += gridSize) {
        mapContext.beginPath();
        mapContext.moveTo(0, y);
        mapContext.lineTo(width, y);
        mapContext.stroke();
    }
    
    // 绘制中心点
    const centerX = width / 2;
    const centerY = height / 2;
    
    mapContext.fillStyle = '#4CAF50';
    mapContext.beginPath();
    mapContext.arc(centerX, centerY, 8, 0, Math.PI * 2);
    mapContext.fill();
    
    mapContext.fillStyle = 'white';
    mapContext.font = '12px Arial';
    mapContext.textAlign = 'center';
    mapContext.fillText('起点', centerX, centerY - 15);
}

// 调整地图画布大小
function resizeMapCanvas() {
    const container = mapCanvas.parentElement;
    if (!container) return;
    
    const width = container.clientWidth;
    const height = Math.min(400, container.clientHeight);
    
    mapCanvas.width = width;
    mapCanvas.height = height;
    
    // 重绘地图
    drawMapGrid();
    drawGoalsOnMap();
    drawRobotOnMap();
}

// 键盘控制初始化
function initKeyboardControls() {
    document.addEventListener('keydown', handleKeyDown);
    document.addEventListener('keyup', handleKeyUp);
    
    console.log('键盘控制已启用');
    logMessage('控制', '键盘控制已启用');
}

// 处理按键按下
function handleKeyDown(event) {
    if (!isConnected) return;
    
    switch(event.key.toLowerCase()) {
        case 'w':
        case 'arrowup':
            startMoving('forward');
            event.preventDefault();
            break;
            
        case 's':
        case 'arrowdown':
            startMoving('backward');
            event.preventDefault();
            break;
            
        case 'a':
        case 'arrowleft':
            startTurning('left');
            event.preventDefault();
            break;
            
        case 'd':
        case 'arrowright':
            startTurning('right');
            event.preventDefault();
            break;
            
        case ' ':
        case 'x':
            emergencyStop();
            event.preventDefault();
            break;
            
        case '+':
        case '=':
            increaseSpeed();
            event.preventDefault();
            break;
            
        case '-':
        case '_':
            decreaseSpeed();
            event.preventDefault();
            break;
    }
}

// 处理按键释放
function handleKeyUp(event) {
    if (!isConnected) return;
    
    switch(event.key.toLowerCase()) {
        case 'w':
        case 'arrowup':
            if (currentKeys.move === 'forward') stopMoving();
            break;
            
        case 's':
        case 'arrowdown':
            if (currentKeys.move === 'backward') stopMoving();
            break;
            
        case 'a':
        case 'arrowleft':
            if (currentKeys.turn === 'left') stopTurning();
            break;
            
        case 'd':
        case 'arrowright':
            if (currentKeys.turn === 'right') stopTurning();
            break;
    }
}

// 开始移动
function startMoving(direction) {
    if (!isConnected) return;
    
    currentKeys.move = direction;
    updateControlLoop();
    logMessage('控制', `${direction === 'forward' ? '前进' : '后退'}`);
}

// 停止移动
function stopMoving() {
    currentKeys.move = null;
    updateControlLoop();
}

// 开始转向
function startTurning(direction) {
    if (!isConnected) return;
    
    currentKeys.turn = direction;
    updateControlLoop();
    logMessage('控制', `${direction === 'left' ? '左转' : '右转'}`);
}

// 停止转向
function stopTurning() {
    currentKeys.turn = null;
    updateControlLoop();
}

// 更新控制循环
function updateControlLoop() {
    // 清除现有的控制循环
    if (controlInterval) {
        clearInterval(controlInterval);
    }
    
    // 如果没有控制命令，直接发送停止命令
    if (!currentKeys.move && !currentKeys.turn) {
        sendControlCommand('stop', 0, 0);
        return;
    }
    
    // 开始新的控制循环
    controlInterval = setInterval(() => {
        sendControlCommand();
    }, 100); // 10Hz控制频率
}

// 发送控制命令
function sendControlCommand(command = 'manual', linearX = 0, angularZ = 0) {
    if (!isConnected || !socket) return;
    
    // 根据当前按键状态计算速度
    if (command === 'manual') {
        // 计算线速度
        if (currentKeys.move === 'forward') {
            linearX = controlParams.speed * controlParams.maxLinear;
        } else if (currentKeys.move === 'backward') {
            linearX = -controlParams.speed * controlParams.maxLinear;
        } else {
            linearX = 0;
        }
        
        // 计算角速度
        if (currentKeys.turn === 'left') {
            angularZ = controlParams.turn * controlParams.maxAngular;
        } else if (currentKeys.turn === 'right') {
            angularZ = -controlParams.turn * controlParams.maxAngular;
        } else {
            angularZ = 0;
        }
    }
    
    // 发送命令
    socket.emit('control_command', {
        command: command,
        linear_x: linearX,
        angular_z: angularZ,
        timestamp: Date.now()
    });
}

// 控制函数
function moveForward() {
    startMoving('forward');
}

function moveBackward() {
    startMoving('backward');
}

function turnLeft() {
    startTurning('left');
}

function turnRight() {
    startTurning('right');
}

function emergencyStop() {
    currentKeys.move = null;
    currentKeys.turn = null;
    
    if (controlInterval) {
        clearInterval(controlInterval);
        controlInterval = null;
    }
    
    sendControlCommand('stop', 0, 0);
    logMessage('控制', '紧急停止');
}

// 速度控制
function updateSpeed(value) {
    controlParams.speed = value / 100;
    document.getElementById('speed-value').textContent = `${value}%`;
    logMessage('参数', `速度设置为: ${value}%`);
}

function updateTurn(value) {
    controlParams.turn = value / 100;
    document.getElementById('turn-value').textContent = `${value}%`;
    logMessage('参数', `转向设置为: ${value}%`);
}

function increaseSpeed() {
    const slider = document.getElementById('speed-slider');
    let value = parseInt(slider.value) + 10;
    if (value > 100) value = 100;
    slider.value = value;
    updateSpeed(value);
}

function decreaseSpeed() {
    const slider = document.getElementById('speed-slider');
    let value = parseInt(slider.value) - 10;
    if (value < 10) value = 10;
    slider.value = value;
    updateSpeed(value);
}

// 负压吸附控制
function controlVacuum(state) {
    if (!isConnected || !socket) return;
    
    socket.emit('vacuum_control', {
        state: state,
        timestamp: Date.now()
    });
}

// 视频控制
function toggleVideo() {
    const videoToggle = document.getElementById('video-toggle');
    const videoFeed = document.getElementById('video-feed');
    
    if (videoToggle.textContent === '暂停视频') {
        // 暂停视频
        const currentSrc = videoFeed.src;
        videoFeed.dataset.lastSrc = currentSrc;
        videoFeed.src = '';
        videoToggle.textContent = '恢复视频';
        logMessage('视频', '视频流已暂停');
    } else {
        // 恢复视频
        if (videoFeed.dataset.lastSrc) {
            videoFeed.src = videoFeed.dataset.lastSrc;
        } else {
            videoFeed.src = '/video_feed';
        }
        videoToggle.textContent = '暂停视频';
        logMessage('视频', '视频流已恢复');
    }
}

function snapshot() {
    const videoFeed = document.getElementById('video-feed');
    const canvas = document.createElement('canvas');
    const context = canvas.getContext('2d');
    
    // 设置画布大小与视频相同
    canvas.width = videoFeed.videoWidth || 640;
    canvas.height = videoFeed.videoHeight || 480;
    
    // 绘制当前视频帧
    context.drawImage(videoFeed, 0, 0, canvas.width, canvas.height);
    
    // 添加时间戳
    const timestamp = new Date().toLocaleString();
    context.fillStyle = 'white';
    context.font = '16px Arial';
    context.fillText(`水下机器人 - ${timestamp}`, 10, 30);
    
    // 转换为数据URL并显示
    const dataUrl = canvas.toDataURL('image/jpeg', 0.9);
    showSnapshotModal(dataUrl);
    
    logMessage('视频', '截图已保存');
}

function showSnapshotModal(dataUrl) {
    const modal = document.getElementById('snapshot-modal');
    const preview = document.getElementById('snapshot-preview');
    
    preview.src = dataUrl;
    modal.style.display = 'block';
}

function closeModal() {
    const modal = document.getElementById('snapshot-modal');
    modal.style.display = 'none';
}

function downloadSnapshot() {
    const preview = document.getElementById('snapshot-preview');
    const link = document.createElement('a');
    
    const timestamp = new Date().toISOString().replace(/[:.]/g, '-');
    link.download = `robot-snapshot-${timestamp}.jpg`;
    link.href = preview.src;
    link.click();
}

function changeResolution() {
    const select = document.getElementById('resolution-select');
    const resolution = select.value;
    
    // 这里可以添加改变视频分辨率的逻辑
    // 可能需要重新连接视频流
    logMessage('视频', `分辨率切换为: ${resolution}`);
}

// 目标点地图功能
function addGoalToMap(goal) {
    goals.push(goal);
    updateGoalList();
    drawGoalsOnMap();
}

function drawGoalsOnMap() {
    if (!mapContext) return;
    
    // 先重绘网格
    drawMapGrid();
    
    // 绘制所有目标点
    goals.forEach((goal, index) => {
        drawGoalPoint(goal, index + 1);
    });
}

function drawGoalPoint(goal, number) {
    const width = mapCanvas.width;
    const height = mapCanvas.height;
    
    // 将世界坐标转换为画布坐标
    // 这里使用简单的缩放和偏移，实际可能需要更复杂的坐标转换
    const scale = 20; // 每米多少像素
    const centerX = width / 2;
    const centerY = height / 2;
    
    const canvasX = centerX + goal.x * scale;
    const canvasY = centerY - goal.y * scale; // Y轴翻转
    
    // 绘制目标点
    mapContext.fillStyle = '#FF5722';
    mapContext.beginPath();
    mapContext.arc(canvasX, canvasY, 10, 0, Math.PI * 2);
    mapContext.fill();
    
    // 绘制目标点编号
    mapContext.fillStyle = 'white';
    mapContext.font = 'bold 14px Arial';
    mapContext.textAlign = 'center';
    mapContext.textBaseline = 'middle';
    mapContext.fillText(number.toString(), canvasX, canvasY);
    
    // 绘制到机器人的连线（如果机器人位置已知）
    if (robotPose.x !== undefined && robotPose.y !== undefined) {
        const robotX = centerX + robotPose.x * scale;
        const robotY = centerY - robotPose.y * scale;
        
        mapContext.strokeStyle = 'rgba(255, 87, 34, 0.5)';
        mapContext.lineWidth = 2;
        mapContext.beginPath();
        mapContext.moveTo(robotX, robotY);
        mapContext.lineTo(canvasX, canvasY);
        mapContext.stroke();
    }
}

function drawRobotOnMap() {
    if (!mapContext || robotPose.x === undefined || robotPose.y === undefined) return;
    
    const width = mapCanvas.width;
    const height = mapCanvas.height;
    const scale = 20;
    const centerX = width / 2;
    const centerY = height / 2;
    
    const robotX = centerX + robotPose.x * scale;
    const robotY = centerY - robotPose.y * scale;
    
    // 绘制机器人位置
    mapContext.fillStyle = '#4CAF50';
    mapContext.beginPath();
    mapContext.arc(robotX, robotY, 8, 0, Math.PI * 2);
    mapContext.fill();
    
    // 绘制机器人方向指示器
    if (robotPose.orientation) {
        const angle = Math.atan2(robotPose.orientation.y, robotPose.orientation.x);
        const arrowLength = 20;
        
        mapContext.strokeStyle = '#4CAF50';
        mapContext.lineWidth = 3;
        mapContext.beginPath();
        mapContext.moveTo(robotX, robotY);
        mapContext.lineTo(
            robotX + Math.cos(angle) * arrowLength,
            robotY - Math.sin(angle) * arrowLength
        );
        mapContext.stroke();
    }
}

function updateGoalList() {
    const listContainer = document.getElementById('goals-list');
    const countElement = document.getElementById('goal-count');
    
    listContainer.innerHTML = '';
    countElement.textContent = goals.length;
    
    goals.forEach((goal, index) => {
        const goalItem = document.createElement('div');
        goalItem.className = 'goal-item';
        
        const time = new Date(goal.timestamp * 1000).toLocaleTimeString();
        
        goalItem.innerHTML = `
            <div class="goal-number">${index + 1}</div>
            <div class="goal-coords">
                X: ${goal.x.toFixed(2)} | Y: ${goal.y.toFixed(2)} | Z: ${goal.z.toFixed(2)}
            </div>
            <div class="goal-time">${time}</div>
            <button onclick="removeGoal(${index})" class="goal-remove">×</button>
        `;
        
        listContainer.appendChild(goalItem);
    });
}

function removeGoal(index) {
    if (index >= 0 && index < goals.length) {
        goals.splice(index, 1);
        updateGoalList();
        drawGoalsOnMap();
        logMessage('目标点', `已删除目标点 ${index + 1}`);
    }
}

function clearGoals() {
    if (goals.length > 0) {
        goals = [];
        updateGoalList();
        drawMapGrid();
        logMessage('目标点', '已清空所有目标点');
    }
}

function exportGoals() {
    const dataStr = JSON.stringify(goals, null, 2);
    const dataBlob = new Blob([dataStr], { type: 'application/json' });
    const url = URL.createObjectURL(dataBlob);
    
    const link = document.createElement('a');
    const timestamp = new Date().toISOString().replace(/[:.]/g, '-');
    link.download = `robot-goals-${timestamp}.json`;
    link.href = url;
    link.click();
    
    logMessage('目标点', `已导出 ${goals.length} 个目标点`);
}

function resetView() {
    drawMapGrid();
    drawGoalsOnMap();
    drawRobotOnMap();
    logMessage('地图', '视图已重置');
}

// 状态更新函数
function updateConnectionStatus(connected) {
    const statusElement = document.getElementById('connection-status');
    const addressElement = document.getElementById('server-address');
    const rosStatus = document.getElementById('ros-status');
    
    if (connected) {
        statusElement.textContent = '已连接';
        statusElement.className = 'connected';
        addressElement.textContent = window.location.host;
        rosStatus.textContent = 'ROS: 连接中...';
        rosStatus.className = 'connecting';
    } else {
        statusElement.textContent = '未连接';
        statusElement.className = 'disconnected';
        addressElement.textContent = '等待连接...';
        rosStatus.textContent = 'ROS: 断开';
        rosStatus.className = 'offline';
    }
}

function updateRobotStatusFromCommand(data) {
    // data 来自 control_response，通常不包含完整状态，只用于短暂显示
    const speedElement = document.getElementById('current-speed');
    const turnElement = document.getElementById('current-turn');
    const robotStatus = document.getElementById('robot-status');
    
    if (data.command === 'stop') {
        if (speedElement) speedElement.textContent = '0.00 m/s';
        if (turnElement) turnElement.textContent = '0.00 rad/s';
    } else {
        if (speedElement) speedElement.textContent = `${(data.linear_x || 0).toFixed(2)} m/s`;
        if (turnElement) turnElement.textContent = `${(data.angular_z || 0).toFixed(2)} rad/s`;
    }
    
    if (robotStatus) {
        robotStatus.textContent = '机器人: 在线';
        robotStatus.className = 'online';
    }
}

function updateRobotPositionDisplay() {
    const positionElement = document.getElementById('robot-position');
    if (!positionElement) return;
    positionElement.textContent = `(${(robotPose.x !== undefined ? robotPose.x.toFixed(2) : '0.00')}, ${(robotPose.y !== undefined ? robotPose.y.toFixed(2) : '0.00')}, ${(robotPose.z !== undefined ? robotPose.z.toFixed(2) : '0.00')})`;
}

function updateVacuumStatus(state) {
    const vacuumStatus = document.getElementById('vacuum-status');
    if (!vacuumStatus) return;
    const statusSpan = vacuumStatus.querySelector('span');
    
    if (statusSpan) {
        if (state) {
            statusSpan.textContent = '开启';
            statusSpan.className = 'on';
        } else {
            statusSpan.textContent = '关闭';
            statusSpan.className = 'off';
        }
    }
}

// 日志功能
function logMessage(source, message) {
    const logContainer = document.getElementById('log-container');
    const timestamp = new Date().toLocaleTimeString();
    
    const logEntry = document.createElement('div');
    logEntry.className = 'log-entry';
    logEntry.innerHTML = `
        <span class="log-time">[${timestamp}]</span>
        <span class="log-source">${source}:</span>
        <span class="log-message">${message}</span>
    `;
    
    logContainer.appendChild(logEntry);
    
    // 自动滚动到底部
    logContainer.scrollTop = logContainer.scrollHeight;
    
    // 更新最后更新时间
    const lastUpd = document.getElementById('last-update');
    if (lastUpd) lastUpd.textContent = timestamp;
}

function clearLog() {
    const logContainer = document.getElementById('log-container');
    logContainer.innerHTML = '';
    logMessage('系统', '日志已清空');
}

// 系统状态检查
async function checkSystemStatus() {
    try {
        const response = await axios.get('/api/status');
        const status = response.data;
        
        // 更新ROS状态
        const rosStatus = document.getElementById('ros-status');
        if (status.ros_connected) {
            rosStatus.textContent = 'ROS: 连接';
            rosStatus.className = 'online';
        } else {
            rosStatus.textContent = 'ROS: 断开';
            rosStatus.className = 'offline';
        }
        
        // 更新摄像头状态
        const cameraStatus = document.getElementById('camera-status');
        if (status.camera_available) {
            cameraStatus.textContent = '摄像头: 在线';
            cameraStatus.className = 'online';
        } else {
            cameraStatus.textContent = '摄像头: 离线';
            cameraStatus.className = 'offline';
        }
        
        // 如果后端表明 robot_controller 已创建，记录日志
        if (status.robot_controller_created) {
            logMessage('系统', 'RobotController 已创建（串口控制可用）');
        }
        
    } catch (error) {
        console.error('检查系统状态失败:', error);
        logMessage('错误', '无法获取系统状态');
    }
}

// 更新UI
function updateUI() {
    // 更新最后更新时间
    const lastUpd = document.getElementById('last-update');
    if (lastUpd) lastUpd.textContent = new Date().toLocaleTimeString();
    
    // 启动定期更新
    setInterval(() => {
        const lastUpd = document.getElementById('last-update');
        if (lastUpd) lastUpd.textContent = new Date().toLocaleTimeString();
    }, 1000);
}

// 页面加载完成后初始化
document.addEventListener('DOMContentLoaded', init);

// 页面关闭前清理
window.addEventListener('beforeunload', () => {
    if (controlInterval) {
        clearInterval(controlInterval);
    }
    
    if (socket && socket.connected) {
        // 发送停止命令
        socket.emit('control_command', {
            command: 'stop',
            linear_x: 0,
            angular_z: 0
        });
        
        // 断开连接
        socket.disconnect();
    }
});
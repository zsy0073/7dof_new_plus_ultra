#ifndef WEBPAGE_H
#define WEBPAGE_H

// 添加加减按钮和相应控制逻辑的网页HTML
const char PAGE_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>7DOF Robot Arm Control</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            margin: 20px;
            background-color: #f0f0f0;
        }
        .container {
            background-color: white;
            padding: 20px;
            border-radius: 10px;
            box-shadow: 0 2px 4px rgba(0,0,0,0.1);
        }
        h2 {
            color: #333;
        }
        .slider-container {
            margin-bottom: 15px;
            display: flex;
            align-items: center;
        }
        .slider-label {
            width: 80px;
            display: inline-block;
        }
        .slider-value {
            width: 40px;
            display: inline-block;
            text-align: center;
        }
        .slider {
            flex-grow: 1;
            margin: 0 10px;
        }
        .btn-container {
            display: flex;
            justify-content: center;
            margin-top: 20px;
        }
        .btn {
            background-color: #4CAF50;
            border: none;
            color: white;
            padding: 10px 20px;
            text-align: center;
            text-decoration: none;
            display: inline-block;
            font-size: 16px;
            margin: 4px 2px;
            cursor: pointer;
            border-radius: 5px;
        }
        .btn:hover {
            background-color: #45a049;
        }
        .temp {
            margin-top: 20px;
            font-style: italic;
        }
        /* 加减按钮的样式 */
        .btn-adjust {
            width: 30px;
            height: 30px;
            background-color: #2196F3;
            border: none;
            color: white;
            text-align: center;
            text-decoration: none;
            display: inline-block;
            font-size: 18px;
            cursor: pointer;
            border-radius: 4px;
            margin: 0 5px;
            line-height: 30px;
            user-select: none;
        }
        .btn-adjust:hover {
            background-color: #0b7dda;
        }
        .btn-adjust:active {
            background-color: #0a5999;
        }
        /* 动作组控制按钮样式 */
        .action-group-controls {
            margin-top: 20px;
            padding: 15px;
            border-radius: 5px;
            background-color: #f5f5f5;
            border: 1px solid #ddd;
        }
        .action-group-title {
            margin-top: 0;
            color: #333;
            font-weight: bold;
            margin-bottom: 10px;
        }
        .action-btn {
            padding: 8px 15px;
            margin: 5px;
            border: none;
            border-radius: 4px;
            cursor: pointer;
            font-weight: bold;
            min-width: 100px;
        }
        .record-btn {
            background-color: #e74c3c;
            color: white;
        }
        .stop-btn {
            background-color: #3498db;
            color: white;
        }
        .play-btn {
            background-color: #2ecc71;
            color: white;
        }
        .action-status {
            margin-top: 10px;
            padding: 5px;
            background-color: #f9f9f9;
            border-radius: 3px;
            min-height: 20px;
            color: #666;
        }
    </style>
</head>
<body>
    <div class="container">
        <h2>7DOF Robot Arm Control</h2>
        
        <div id="jointControls">
            <!-- 关节控制滑块和加减按钮 -->
            <div class="slider-container" id="joint0">
                <span class="slider-label">Joint 0:</span>
                <button class="btn-adjust" onclick="adjustSlider('j0', -10)">-</button>
                <input type="range" min="0" max="1000" value="500" class="slider" id="j0">
                <button class="btn-adjust" onclick="adjustSlider('j0', 10)">+</button>
                <span class="slider-value" id="j0Value">500</span>
            </div>
            
            <div class="slider-container" id="joint1">
                <span class="slider-label">Joint 1:</span>
                <button class="btn-adjust" onclick="adjustSlider('j1', -10)">-</button>
                <input type="range" min="0" max="1000" value="500" class="slider" id="j1">
                <button class="btn-adjust" onclick="adjustSlider('j1', 10)">+</button>
                <span class="slider-value" id="j1Value">500</span>
            </div>
            
            <div class="slider-container" id="joint2">
                <span class="slider-label">Joint 2:</span>
                <button class="btn-adjust" onclick="adjustSlider('j2', -10)">-</button>
                <input type="range" min="0" max="1000" value="500" class="slider" id="j2">
                <button class="btn-adjust" onclick="adjustSlider('j2', 10)">+</button>
                <span class="slider-value" id="j2Value">500</span>
            </div>
            
            <div class="slider-container" id="joint3">
                <span class="slider-label">Joint 3:</span>
                <button class="btn-adjust" onclick="adjustSlider('j3', -10)">-</button>
                <input type="range" min="0" max="1000" value="500" class="slider" id="j3">
                <button class="btn-adjust" onclick="adjustSlider('j3', 10)">+</button>
                <span class="slider-value" id="j3Value">500</span>
            </div>
            
            <div class="slider-container" id="joint4">
                <span class="slider-label">Joint 4:</span>
                <button class="btn-adjust" onclick="adjustSlider('j4', -10)">-</button>
                <input type="range" min="0" max="1000" value="500" class="slider" id="j4">
                <button class="btn-adjust" onclick="adjustSlider('j4', 10)">+</button>
                <span class="slider-value" id="j4Value">500</span>
            </div>
            
            <div class="slider-container" id="joint5">
                <span class="slider-label">Joint 5:</span>
                <button class="btn-adjust" onclick="adjustSlider('j5', -10)">-</button>
                <input type="range" min="0" max="1000" value="500" class="slider" id="j5">
                <button class="btn-adjust" onclick="adjustSlider('j5', 10)">+</button>
                <span class="slider-value" id="j5Value">500</span>
            </div>
            
            <div class="slider-container" id="joint6">
                <span class="slider-label">Joint 6:</span>
                <button class="btn-adjust" onclick="adjustSlider('j6', -10)">-</button>
                <input type="range" min="0" max="1000" value="500" class="slider" id="j6">
                <button class="btn-adjust" onclick="adjustSlider('j6', 10)">+</button>
                <span class="slider-value" id="j6Value">500</span>
            </div>
            
            <div class="slider-container" id="gripper">
                <span class="slider-label">Gripper:</span>
                <button class="btn-adjust" onclick="adjustSlider('g', -10)">-</button>
                <input type="range" min="0" max="1000" value="500" class="slider" id="g">
                <button class="btn-adjust" onclick="adjustSlider('g', 10)">+</button>
                <span class="slider-value" id="gValue">500</span>
            </div>
        </div>
        
        <div class="btn-container">
            <button class="btn" onclick="resetAll()">Reset All</button>
        </div>
        
        <!-- 添加动作组控制部分 (改为英文) -->
        <div class="action-group-controls">
            <h3 class="action-group-title">Action Group Control</h3>
            <button class="action-btn record-btn" onclick="sendActionCommand('action_record_start')">Start Record</button>
            <button class="action-btn stop-btn" onclick="sendActionCommand('action_record_stop')">Stop Record</button>
            <button class="action-btn play-btn" onclick="sendActionCommand('action_play')">Play Actions</button>
            <div id="action-status" class="action-status"></div>
        </div>
        
        <div class="temp" id="tempDisplay">
            Temperature: -- &deg;C
        </div>
    </div>

    <script>
        var gateway = `ws://${window.location.hostname}/ws`;
        var websocket;
        
        // 初始化连接
        window.addEventListener('load', onLoad);
        
        function initWebSocket() {
            console.log('Trying to open a WebSocket connection...');
            websocket = new WebSocket(gateway);
            websocket.onopen = onOpen;
            websocket.onclose = onClose;
            websocket.onmessage = onMessage;
        }
        
        function onOpen(event) {
            console.log('Connection opened');
        }
        
        function onClose(event) {
            console.log('Connection closed');
            setTimeout(initWebSocket, 2000);
        }
        
        function onMessage(event) {
            try {
                var data = JSON.parse(event.data);
                console.log("收到数据:", data);
                
                // 更新温度显示
                document.getElementById('tempDisplay').innerHTML = `<strong>Temperature:</strong> <span style="font-size: 18px; color: #2196F3;">${data.temp} &deg;C</span>`;
                
                // 更新滑块位置
                if (data.joints && Array.isArray(data.joints)) {
                    for (var i = 0; i < Math.min(7, data.joints.length); i++) {
                        var jointSlider = document.getElementById(`j${i}`);
                        var jointValue = document.getElementById(`j${i}Value`);
                        
                        if (jointSlider && jointValue) {
                            console.log(`更新关节 ${i} 到位置: ${data.joints[i]}`);
                            jointSlider.value = data.joints[i];
                            jointValue.innerHTML = data.joints[i];
                        } else {
                            console.error(`未找到关节 ${i} 的控制元素`);
                        }
                    }
                }
                
                if (data.hasOwnProperty('gripper')) {
                    var gripperSlider = document.getElementById('g');
                    var gripperValue = document.getElementById('gValue');
                    
                    if (gripperSlider && gripperValue) {
                        console.log(`更新夹持器到位置: ${data.gripper}`);
                        gripperSlider.value = data.gripper;
                        gripperValue.innerHTML = data.gripper;
                    } else {
                        console.error("未找到夹持器的控制元素");
                    }
                }
                
                // 处理动作组状态更新 (翻译状态消息)
                if (data.hasOwnProperty('actionStatus')) {
                    // 转换中文状态为英文
                    let statusMsg = data.actionStatus;
                    if(statusMsg.includes("正在记录动作组")) {
                        const count = statusMsg.match(/\d+/);
                        statusMsg = `Recording actions - ${count} actions recorded`;
                    } else if(statusMsg.includes("正在播放动作组")) {
                        statusMsg = "Playing action sequence";
                    } else if(statusMsg.includes("动作组已记录")) {
                        const count = statusMsg.match(/\d+/);
                        statusMsg = `${count} actions recorded, ready to play`;
                    }
                    document.getElementById('action-status').innerHTML = statusMsg;
                }
            } catch (error) {
                console.error("处理消息时出错:", error, event.data);
            }
        }
        
        // 添加按钮长按功能相关变量
        var longPressTimer = null;
        var isLongPress = false;
        var longPressDelay = 300; // 长按触发延迟(ms)
        var longPressInterval = 100; // 长按期间重复执行间隔(ms)
        // 添加拖动处理相关变量
        var lastSendTime = 0;
        var throttleDelay = 50; // 命令发送节流延迟(ms)
        
        function onLoad(event) {
            initWebSocket();
            initSliders();
            initLongPressButtons();
        }
        
        // 初始化长按按钮功能
        function initLongPressButtons() {
            // 获取所有加减按钮
            var adjustButtons = document.querySelectorAll('.btn-adjust');
            
            // 为每个按钮添加事件监听器
            adjustButtons.forEach(function(button) {
                // 解析按钮数据
                var actionData = button.getAttribute('onclick');
                var params = actionData.substring(actionData.indexOf('(') + 1, actionData.indexOf(')')).split(',');
                var sliderId = params[0].trim().replace(/'/g, '');
                var amount = parseInt(params[1].trim());
                
                // 移除原有的onclick属性
                button.removeAttribute('onclick');
                
                // 添加触摸和鼠标事件监听器
                // 鼠标事件
                button.addEventListener('mousedown', function(e) {
                    startLongPress(sliderId, amount);
                });
                button.addEventListener('mouseup', stopLongPress);
                button.addEventListener('mouseleave', stopLongPress);
                
                // 触摸事件
                button.addEventListener('touchstart', function(e) {
                    e.preventDefault(); // 防止触摸事件导致滚动
                    startLongPress(sliderId, amount);
                });
                button.addEventListener('touchend', stopLongPress);
                button.addEventListener('touchcancel', stopLongPress);
            });
        }
        
        // 启动长按
        function startLongPress(sliderId, amount) {
            // 先立即执行一次调整
            adjustSlider(sliderId, amount);
            
            // 设置长按标志
            isLongPress = false;
            
            // 清除可能存在的定时器
            clearTimeout(longPressTimer);
            clearInterval(longPressTimer);
            
            // 设置长按延迟触发
            longPressTimer = setTimeout(function() {
                isLongPress = true;
                
                // 设置定期执行的间隔
                longPressTimer = setInterval(function() {
                    adjustSlider(sliderId, amount);
                }, longPressInterval);
                
            }, longPressDelay);
        }
        
        // 停止长按
        function stopLongPress() {
            clearTimeout(longPressTimer);
            clearInterval(longPressTimer);
            longPressTimer = null;
            isLongPress = false;
        }
        
        function initSliders() {
            // 初始化所有滑块的事件监听
            for (var i = 0; i < 7; i++) {
                (function(index) {  // 创建闭包，确保正确保存当前索引值
                    var slider = document.getElementById(`j${index}`);
                    var output = document.getElementById(`j${index}Value`);
                    
                    if (slider && output) {
                        slider.oninput = function() {
                            // 更新显示值
                            output.innerHTML = this.value;
                            
                            // 添加拖动节流，避免发送过多命令
                            const now = Date.now();
                            if (now - lastSendTime > throttleDelay) {
                                console.log(`发送命令: J${index}:${this.value}`);
                                sendCommand(`J${index}:${this.value}`);
                                lastSendTime = now;
                            }
                        }
                        
                        // 保留onchange事件，确保最后一个值被发送
                        slider.onchange = function() {
                            console.log(`最终命令: J${index}:${this.value}`);
                            sendCommand(`J${index}:${this.value}`);
                        }
                    }
                })(i);  // 立即执行函数，传入当前循环索引i
            }
            
            var gripperSlider = document.getElementById('g');
            var gripperOutput = document.getElementById('gValue');
            
            if (gripperSlider && gripperOutput) {
                gripperSlider.oninput = function() {
                    // 更新显示值
                    gripperOutput.innerHTML = this.value;
                    
                    // 添加拖动节流，避免发送过多命令
                    const now = Date.now();
                    if (now - lastSendTime > throttleDelay) {
                        console.log(`Sending G:${this.value}`);
                        sendCommand(`G:${this.value}`);
                        lastSendTime = now;
                    }
                }
                
                // 保留onchange事件，确保最后一个值被发送
                gripperSlider.onchange = function() {
                    console.log(`Final G:${this.value}`);
                    sendCommand(`G:${this.value}`);
                }
            }
        }
        
        // 调整滑块值的函数，用于加减按钮
        function adjustSlider(sliderId, amount) {
            var slider = document.getElementById(sliderId);
            var output = document.getElementById(`${sliderId}Value`);
            if (!slider || !output) return;  // 确保元素存在
            
            var currentValue = parseInt(slider.value);
            var newValue = currentValue + amount;
            
            // 确保值在合法范围内
            newValue = Math.max(0, Math.min(1000, newValue));
            
            // 如果值没有变化，不发送命令
            if (newValue === currentValue) {
                return;
            }
            
            // 更新滑块和显示值
            slider.value = newValue;
            output.innerHTML = newValue;
            
            // 发送命令
            if (sliderId.startsWith('j')) {
                const jointIdx = parseInt(sliderId.substring(1));
                console.log(`按钮调整命令: J${jointIdx}:${newValue}`);
                sendCommand(`J${jointIdx}:${newValue}`);
            } else if (sliderId === 'g') {
                console.log(`按钮调整命令: G:${newValue}`);
                sendCommand(`G:${newValue}`);
            }
        }
        
        function resetAll() {
            sendCommand('R');
        }
        
        function sendCommand(command) {
            if (websocket.readyState === WebSocket.OPEN) {
                websocket.send(command);
            }
        }
        
        // 发送动作组命令
        function sendActionCommand(command) {
            if (websocket.readyState === WebSocket.OPEN) {
                websocket.send(command);
                document.getElementById('action-status').innerHTML = "发送命令: " + command;
            }
        }
    </script>
</body>
</html>
)rawliteral";

#endif // WEBPAGE_H
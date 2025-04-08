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
        
        <div class="temp" id="tempDisplay">
            Temperature: -- °C
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
                console.log("Received data:", data); // 调试输出接收到的数据document.getElementById('tempDisplay').innerHTML = `<strong>Temperature:</strong> <span style="font-size: 18px; color: #2196F3;">${data.temp} &deg;C</span>`;
                
                // 更新温度显示
                document.getElementById('tempDisplay').innerHTML = `<strong>Temperature:</strong> <span style="font-size: 18px; color: #2196F3;">${data.temp} &deg;C</span>`;
                
                // 更新滑块位置getElementById(`j${i}Value`);
                if (data.joints && Array.isArray(data.joints)) {
                    for (var i = 0; i < Math.min(7, data.joints.length); i++) {
                        var joint = document.getElementById(`j${i}`);   jointValue.innerHTML = data.joints[i];
                        var jointValue = document.getElementById(`j${i}Value`);   }
                        if (joint && jointValue) {}
                            joint.value = data.joints[i];
                            jointValue.innerHTML = data.joints[i];
                        }tElementById('gValue');
                    }
                }
                   gripperValue.innerHTML = data.gripper;
                if (data.hasOwnProperty('gripper')) {   }
                    var gripper = document.getElementById('g');}
                    var gripperValue = document.getElementById('gValue');
                    if (gripper && gripperValue) {
                        gripper.value = data.gripper;l;
                        gripperValue.innerHTML = data.gripper;
                    }
                }Interval = 100; // 长按期间重复执行间隔(ms)
            } catch (error) {
                console.error("Error processing message:", error, event.data);
            }var throttleDelay = 50; // 命令发送节流延迟(ms)
        }
        t) {
        // 添加按钮长按功能相关变量);
        var longPressTimer = null;
        var isLongPress = false;   initLongPressButtons();
        var longPressDelay = 300; // 长按触发延迟(ms)}
        var longPressInterval = 100; // 长按期间重复执行间隔(ms)
        // 添加拖动处理相关变量
        var lastSendTime = 0;ngPressButtons() {
        var throttleDelay = 50; // 命令发送节流延迟(ms)
        var adjustButtons = document.querySelectorAll('.btn-adjust');
        function onLoad(event) {
            initWebSocket();
            initSliders();.forEach(function(button) {
            initLongPressButtons();
        }
        Of('(') + 1, actionData.indexOf(')')).split(',');
        // 初始化长按按钮功能/'/g, '');
        function initLongPressButtons() {var amount = parseInt(params[1].trim());
            // 获取所有加减按钮
            var adjustButtons = document.querySelectorAll('.btn-adjust');
            button.removeAttribute('onclick');
            // 为每个按钮添加事件监听器
            adjustButtons.forEach(function(button) {和鼠标事件监听器
                // 解析按钮数据
                var actionData = button.getAttribute('onclick');function(e) {
                var params = actionData.substring(actionData.indexOf('(') + 1, actionData.indexOf(')')).split(','); startLongPress(sliderId, amount);
                var sliderId = params[0].trim().replace(/'/g, '');
                var amount = parseInt(params[1].trim());
                button.addEventListener('mouseleave', stopLongPress);
                // 移除原有的onclick属性
                button.removeAttribute('onclick');
                 function(e) {
                // 添加触摸和鼠标事件监听器
                // 鼠标事件 startLongPress(sliderId, amount);
                button.addEventListener('mousedown', function(e) {
                    startLongPress(sliderId, amount);
                }); button.addEventListener('touchcancel', stopLongPress);
                button.addEventListener('mouseup', stopLongPress);   });
                button.addEventListener('mouseleave', stopLongPress);}
                
                // 触摸事件
                button.addEventListener('touchstart', function(e) {ngPress(sliderId, amount) {
                    e.preventDefault(); // 防止触摸事件导致滚动
                    startLongPress(sliderId, amount);adjustSlider(sliderId, amount);
                });
                button.addEventListener('touchend', stopLongPress);
                button.addEventListener('touchcancel', stopLongPress);isLongPress = false;
            });
        }
        
        // 启动长按clearInterval(longPressTimer);
        function startLongPress(sliderId, amount) {
            // 先立即执行一次调整
            adjustSlider(sliderId, amount);eout(function() {
            isLongPress = true;
            // 设置长按标志
            isLongPress = false;
            on() {
            // 清除可能存在的定时器rId, amount);
            clearTimeout(longPressTimer);}, longPressInterval);
            clearInterval(longPressTimer);
               }, longPressDelay);
            // 设置长按延迟触发}
            longPressTimer = setTimeout(function() {
                isLongPress = true;
                
                // 设置定期执行的间隔
                longPressTimer = setInterval(function() {sTimer);
                    adjustSlider(sliderId, amount);l;
                }, longPressInterval);   isLongPress = false;
                }
            }, longPressDelay);
        }s() {
        
        // 停止长按  // 保存循环变量< 7; i++) {
        function stopLongPress() {
            clearTimeout(longPressTimer);
            clearInterval(longPressTimer);var output = document.getElementById(`j${i}Value`);
            longPressTimer = null;
            isLongPress = false;ninput = function() {ut = function() {
        }
            output.innerHTML = this.value;output.innerHTML = this.value;
        function initSliders() {
            // 初始化所有滑块的事件监听
            for (var i = 0; i < 7; i++) {
                const idx = i;y) {
                var slider = document.getElementById(`j${i}`);nding J${idx}:${this.value}`); // 调试输出}:${this.value}`);
                var output = document.getElementById(`j${i}Value`);       sendCommand(`J${idx}:${this.value}`);   lastSendTime = now;
                           lastSendTime = now;   }
                slider.oninput = function() {        }}
                    // 更新显示值
                    output.innerHTML = this.value;
                    
                    // 添加拖动节流，避免发送过多命令   slider.onchange = function() {   sendCommand(`J${idx}:${this.value}`);
                    const now = Date.now();           console.log(`Final J${idx}:${this.value}`); // 调试输出   }
                    if (now - lastSendTime > throttleDelay) {            sendCommand(`J${idx}:${this.value}`);}
                        sendCommand(`J${idx}:${this.value}`);
                        lastSendTime = now;
                    }}var gripperOutput = document.getElementById('gValue');
                }
                lider = document.getElementById('g');r.oninput = function() {
                // 保留onchange事件，确保最后一个值被发送Id('gValue');
                slider.onchange = function() {perOutput.innerHTML = this.value;
                    sendCommand(`J${idx}:${this.value}`);ripperOutput) {
                } function() {
            }
            value;lay) {
            var gripperSlider = document.getElementById('g');
            var gripperOutput = document.getElementById('gValue');   // 添加拖动节流，避免发送过多命令   lastSendTime = now;
                   const now = Date.now();   }
            gripperSlider.oninput = function() {        if (now - lastSendTime > throttleDelay) {}
                // 更新显示值ending G:${this.value}`); // 调试输出
                gripperOutput.innerHTML = this.value;ue}`);
                
                // 添加拖动节流，避免发送过多命令       }   sendCommand(`G:${this.value}`);
                const now = Date.now();       }   }
                if (now - lastSendTime > throttleDelay) {        }
                    sendCommand(`G:${this.value}`);nge事件，确保最后一个值被发送
                    lastSendTime = now;() {
                }/ 调试输出
            }
            
            // 保留onchange事件，确保最后一个值被发送
            gripperSlider.onchange = function() { newValue = currentValue + amount;
                sendCommand(`G:${this.value}`);
            }
        }tion adjustSlider(sliderId, amount) {newValue = Math.max(0, Math.min(1000, newValue));
        ument.getElementById(sliderId);
        // 调整滑块值的函数，用于加减按钮ById(`${sliderId}Value`);
        function adjustSlider(sliderId, amount) {Value = parseInt(slider.value);e === currentValue) {
            var slider = document.getElementById(sliderId);ar newValue = currentValue + amount;   return;
            var output = document.getElementById(`${sliderId}Value`);}
            var currentValue = parseInt(slider.value);内
            var newValue = currentValue + amount;ath.min(1000, newValue));
            
            // 确保值在合法范围内// 如果值没有变化，不发送命令output.innerHTML = newValue;
            newValue = Math.max(0, Math.min(1000, newValue));Value === currentValue) {
            
            // 如果值没有变化，不发送命令
            if (newValue === currentValue) {
                return;
            }
            utput.innerHTML = newValue;   sendCommand(`G:${newValue}`);
            // 更新滑块和显示值      }
            slider.value = newValue;    // 发送命令}
            output.innerHTML = newValue;tsWith('j')) {
            x = parseInt(sliderId.substring(1));
            // 发送命令       sendCommand(`J${jointIdx}:${newValue}`);   sendCommand('R');
            if (sliderId.startsWith('j')) {    } else if (sliderId === 'g') {}
                const jointIdx = parseInt(sliderId.substring(1));ue}`);
                sendCommand(`J${jointIdx}:${newValue}`);
            } else if (sliderId === 'g') {
                sendCommand(`G:${newValue}`);bsocket.send(command);
            }unction resetAll() {   }
        }endCommand('R');
         }cript>
        function resetAll() { 
            sendCommand('R');ion sendCommand(command) {
        }            if (websocket.readyState === WebSocket.OPEN) {)rawliteral";
        socket.send(command);
        function sendCommand(command) {            }#endif // WEBPAGE_H











#endif // WEBPAGE_H)rawliteral";</html></body>    </script>        }            }                websocket.send(command);            if (websocket.readyState === WebSocket.OPEN) {







#endif // WEBPAGE_H)rawliteral";</html></body>    </script>        }
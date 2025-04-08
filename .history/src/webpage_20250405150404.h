#ifndef WEBPAGE_H
#define WEBPAGE_H

const char PAGE_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>机械臂控制</title>
    <style>
        body { font-family: Arial, sans-serif; max-width: 800px; margin: 0 auto; padding: 20px; }
        .slider-container { margin: 15px 0; padding: 10px; background: #f8f8f8; border-radius: 8px; }
        .slider-wrapper { display: flex; align-items: center; gap: 15px; }
        .slider { flex: 1; }
        .value-display { min-width: 60px; text-align: center; }
    </style>
</head>
<body>
    <div class="slider-container">
        <h3>关节控制</h3>
        <div id="joints"></div>
    </div>
    
    <div class="slider-container">
        <h3>夹爪控制</h3>
        <div class="slider-wrapper">
            <input type="range" min="0" max="1000" value="500" id="gripper">
            <span class="value-display" id="gripperValue">500</span>
        </div>
    </div>

    <button onclick="resetAll()">一键复位</button>

    <script>
        const ws = new WebSocket(`ws://${location.host}/ws`);
        let debounceTimer;

        function createJointSlider(id) {
            const container = document.createElement('div');
            container.className = 'slider-wrapper';
            container.innerHTML = `
                <span>关节 ${id + 1}</span>
                <input type="range" min="0" max="1000" value="500" 
                       class="joint-slider" data-id="${id}">
                <span class="value-display" id="joint${id}Value">500</span>
            `;
            return container;
        }

        // 初始化关节滑块
        for (let i = 0; i < 7; i++) {
            document.getElementById('joints').appendChild(createJointSlider(i));
        }

        // 添加事件监听器
        document.querySelectorAll('.joint-slider, #gripper').forEach(slider => {
            const display = slider.nextElementSibling;
            slider.addEventListener('input', () => {
                display.textContent = slider.value;
                clearTimeout(debounceTimer);
                debounceTimer = setTimeout(() => {
                    const cmd = slider.classList.contains('joint-slider')
                        ? `J${slider.dataset.id}:${slider.value}`
                        : `G:${slider.value}`;
                    ws.send(cmd);
                }, 100);
            });

            // 确保滑块的值在 0 到 1000 范围内
            slider.min = 0;
            slider.max = 1000;
            slider.value = 500; // 设置初始值为 500
        });

        function resetAll() {
            ws.send('R');
        }

        ws.onmessage = e => {
            const data = JSON.parse(e.data);
            document.querySelectorAll('.joint-slider').forEach((slider, i) => {
                if (slider.value != data.joints[i]) {
                    slider.value = data.joints[i];
                    slider.nextElementSibling.textContent = data.joints[i];
                }
            });
            const gripperSlider = document.getElementById('gripper');
            if (gripperSlider.value != data.gripper) {
                gripperSlider.value = data.gripper;
                document.getElementById('gripperValue').textContent = data.gripper;
            }
        };
    </script>
</body>
</html>
)rawliteral";

#endif

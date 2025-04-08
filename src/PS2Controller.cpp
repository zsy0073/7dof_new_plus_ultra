#include "PS2Controller.h"
#include "ServoControl.h" // 这个是必要的，因为使用了舵机控制函数

// 实例化全局对象
PS2Controller ps2Controller;

PS2Controller::PS2Controller() {
    isConnected = false;
    lastUpdateTime = 0;
    lastDebounceTime = 0;
    
    // 初始化关节位置和速度
    for (int i = 0; i < 7; i++) {
        joints[i].currentPos = 500;
        joints[i].targetSpeed = 0;
    }
    
    gripperPos = 500; // 夹爪初始位置
}

void PS2Controller::init() {
    // 增加延时确保PS2控制器上电稳定
    delay(300);
    
    // 设置PS2控制器引脚模式
    pinMode(PS2_CLK_PIN, OUTPUT);
    pinMode(PS2_CMD_PIN, OUTPUT);
    pinMode(PS2_SEL_PIN, OUTPUT);
    pinMode(PS2_DAT_PIN, INPUT_PULLUP); // 明确使用上拉输入模式
    
    Serial.println("开始PS2手柄初始化...");
    Serial.printf("CLK: %d, CMD: %d, SEL: %d, DAT: %d\n", 
                  PS2_CLK_PIN, PS2_CMD_PIN, PS2_SEL_PIN, PS2_DAT_PIN);
    
    // 尝试多次初始化，提高成功率
    int error = -1;
    for (int i = 0; i < 3 && error != 0; i++) {
        // 第一个参数是CLK（时钟）
        error = ps2x.config_gamepad(PS2_CLK_PIN, PS2_CMD_PIN, PS2_SEL_PIN, PS2_DAT_PIN, false, false);
        if (error == 0) {
            isConnected = true;
            Serial.println("PS2手柄已连接成功");
        } else {
            Serial.printf("PS2手柄连接失败，错误代码: %d，尝试: %d\n", error, i+1);
            delay(100); // 短暂延时后重试
        }
    }
    
    // 如果始终连接失败，增加额外诊断信息
    if (!isConnected) {
        Serial.println("PS2手柄连接失败，请检查:");
        Serial.println("1. 硬件连接是否正确");
        Serial.println("2. PS2手柄是否通电");
        Serial.println("3. 是否有其他任务干扰");
    } else {
        // 成功连接后，尝试读取一次手柄数据确认通信正常
        if (ps2x.read_gamepad(false, 0)) {
            Serial.println("PS2手柄通信测试成功");
        } else {
            Serial.println("PS2手柄初始化成功但通信测试失败");
        }
    }
}

void PS2Controller::update() {
    // 如果未连接，尝试重新初始化（每30秒一次）
    static unsigned long lastReconnectTime = 0;
    if (!isConnected && millis() - lastReconnectTime > 30000) {
        Serial.println("尝试重新连接PS2手柄...");
        init();
        lastReconnectTime = millis();
        return;
    }
    
    if (!isConnected) return;
    
    // 限制更新频率
    unsigned long currentTime = millis();
    if (currentTime - lastUpdateTime < 20) return; // 降低频率到50Hz，提高稳定性
    lastUpdateTime = currentTime;
    
    // 读取手柄状态，增加错误处理
    bool success = ps2x.read_gamepad(false, 0);
    if (!success) {
        static int errorCount = 0;
        errorCount++;
        if (errorCount > 10) { // 连续多次读取失败
            Serial.println("PS2手柄连续读取失败，标记为断连");
            isConnected = false;
            errorCount = 0;
            return;
        }
    } else {
        // 处理控制逻辑
        handleJointControl();
        handleGripper();
    }
}

void PS2Controller::handleJointControl() {
    unsigned long currentTime = millis();
    if (currentTime - lastDebounceTime > 50) { // 消抖延时
        lastDebounceTime = currentTime;
        
        // 检测复位按钮 (X按钮)
        if (ps2x.Button(PSB_CROSS)) {
            resetAll();
            return; // 复位后直接返回
        }
        
        // 左摇杆控制1、2舵机
        int lx = ps2x.Analog(PSS_LX);
        int ly = ps2x.Analog(PSS_LY);
        
        // 左摇杆横轴控制1舵机
        if (abs(lx - 128) > DEAD_ZONE) {
            joints[0].targetSpeed = map(lx, 0, 255, MAX_SPEED, -MAX_SPEED);
        } else {
            joints[0].targetSpeed = 0;
        }
        
        // 左摇杆纵轴控制2舵机
        if (abs(ly - 128) > DEAD_ZONE) {
            joints[1].targetSpeed = map(ly, 0, 255, MAX_SPEED, -MAX_SPEED);
        } else {
            joints[1].targetSpeed = 0;
        }
        
        // 右摇杆控制3、4舵机
        int rx = ps2x.Analog(PSS_RX);
        int ry = ps2x.Analog(PSS_RY);
        
        if (abs(rx - 128) > DEAD_ZONE) {
            joints[2].targetSpeed = map(rx, 0, 255, MAX_SPEED, -MAX_SPEED);
        } else {
            joints[2].targetSpeed = 0;
        }
        
        if (abs(ry - 128) > DEAD_ZONE) {
            joints[3].targetSpeed = map(ry, 0, 255, MAX_SPEED, -MAX_SPEED);
        } else {
            joints[3].targetSpeed = 0;
        }
        
        // 方向键控制5、6舵机
        bool up = ps2x.Button(PSB_PAD_UP);
        bool down = ps2x.Button(PSB_PAD_DOWN);
        bool left = ps2x.Button(PSB_PAD_LEFT);
        bool right = ps2x.Button(PSB_PAD_RIGHT);
        
        static unsigned long lastDirTime = 0;
        
        // 控制第5个舵机 (上下方向)
        if (up) {
            joints[4].targetSpeed = BUTTON_SPEED;
            lastDirTime = millis();
        } 
        else if (down) {
            joints[4].targetSpeed = -BUTTON_SPEED;
            lastDirTime = millis();
        }
        else if (millis() - lastDirTime > BRAKE_DELAY) {
            joints[4].targetSpeed = 0;
        }
        
        // 控制第6个舵机 (左右方向)
        if (right) {
            joints[5].targetSpeed = BUTTON_SPEED;
            lastDirTime = millis();
        } 
        else if (left) {
            joints[5].targetSpeed = -BUTTON_SPEED;
            lastDirTime = millis();
        }
        else if (millis() - lastDirTime > BRAKE_DELAY) {
            joints[5].targetSpeed = 0;
        }
        
        // 左边的两个扳机控制7舵机
        if (ps2x.Button(PSB_L2)) {
            joints[6].targetSpeed = -BUTTON_SPEED;
        } else if (ps2x.Button(PSB_L1)) {
            joints[6].targetSpeed = BUTTON_SPEED;
        } else {
            joints[6].targetSpeed = 0;
        }
    }
    
    // 更新所有关节位置并发送命令
    for (int i = 0; i < 7; i++) {
        if (joints[i].targetSpeed != 0) {
            // 更新位置
            joints[i].currentPos = constrain(joints[i].currentPos + joints[i].targetSpeed, 
                                    GRIPPER_MIN, GRIPPER_MAX);
            
            // 发送舵机命令 - 修改部分 
            // 无论是否在记录或播放模式，都发送命令（但在播放模式下不应更改关节位置）
            if (!isPlaying) { 
                handleServoControl(jointServos[i], joints[i].currentPos);
            }
        }
    }
}

void PS2Controller::handleGripper() {
    static int lastGrip = 500;
    static bool isMoving = false;
    
    // 右边的扳机控制夹爪
    if (ps2x.Button(PSB_R2)) {
        gripperPos = constrain(gripperPos - BUTTON_SPEED, GRIPPER_MIN, GRIPPER_MAX);
        isMoving = true;
    }
    if (ps2x.Button(PSB_R1)) {
        gripperPos = constrain(gripperPos + BUTTON_SPEED, GRIPPER_MIN, GRIPPER_MAX);
        isMoving = true;
    }
    
    // 修改部分 - 无论是否在记录模式，都发送命令（但在播放模式下不应更改夹爪位置）
    if ((gripperPos != lastGrip || isMoving) && !isPlaying) {
        handleServoControl(gripperServo, gripperPos);
        lastGrip = gripperPos;
        isMoving = false;
    }
}

void PS2Controller::resetAll() {
    if (!isPlaying && !isRecording) {
        Serial.println("PS2手柄: 复位所有舵机");
        
        // 创建复位命令
        ServoCommand cmd;
        cmd.isReset = true;
        xQueueSend(servoCommandQueue, &cmd, portMAX_DELAY);
        
        // 更新本地状态
        for (int i = 0; i < 7; i++) {
            joints[i].currentPos = 500;
            joints[i].targetSpeed = 0;
        }
        gripperPos = 500;
    }
}

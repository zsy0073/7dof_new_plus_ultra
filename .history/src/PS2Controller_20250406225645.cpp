#include "PS2Controller.h"
#include "ServoControl.h"

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
    // 使用Config.h中定义的引脚
    int error = ps2x.config_gamepad(PS2_CLK_PIN, PS2_CMD_PIN, PS2_SEL_PIN, PS2_DAT_PIN);
    if (error == 0) {
        isConnected = true;
        Serial.println("PS2手柄已连接");
    } else {
        Serial.print("PS2手柄连接失败，错误代码: ");
        Serial.println(error);
    }
}

void PS2Controller::update() {
    if (!isConnected) return;
    
    // 限制更新频率
    unsigned long currentTime = millis();
    if (currentTime - lastUpdateTime < 10) return; // 约100Hz
    lastUpdateTime = currentTime;
    
    // 读取手柄状态
    ps2x.read_gamepad(false, 0);
    
    // 处理控制逻辑
    handleJointControl();
    handleGripper();
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
            
            // 发送舵机命令
            if (!isPlaying && !isRecording) { // 只在非动作组模式下执行
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
    
    if ((gripperPos != lastGrip || isMoving) && !isPlaying && !isRecording) {
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

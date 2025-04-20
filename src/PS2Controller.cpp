#include "PS2Controller.h"
#include "ServoControl.h" // 这个是必要的，因为使用了舵机控制函数
#include "Tasks.h" // 添加Tasks头文件，用于访问运动学命令函数

// 实例化全局对象
PS2Controller ps2Controller;

// 运动学控制模式
bool kinematicsMode = false;
// 当前末端位置和姿态
float currentPos[3] = {0.2, 0.0, 0.4}; // 默认位置(米)
float currentRot[3] = {0.0, 0.0, 0.0}; // 默认姿态(弧度)
// 位置和姿态增量步长
const float POS_STEP = 0.01f; // 位置步长(米)
const float ROT_STEP = 0.05f; // 旋转步长(弧度)

// 示例程序相关变量
bool isRunningDemo = false;
int demoStep = 0;
unsigned long demoStartTime = 0;
unsigned long demoStepStartTime = 0;

// 示例程序的各个点位
const int DEMO_STEPS = 5;
const float demoPositions[DEMO_STEPS][3] = {
    {0.25, 0.0, 0.30},     // 初始位置
    {0.22, 0.10, 0.28},    // 右上角 - 更靠近中心点
    {0.22, 0.10, 0.22},    // 右下角 - 减小Z轴高度差
    {0.22, -0.10, 0.22},   // 左下角 - 减小Y轴范围
    {0.22, -0.10, 0.28}    // 左上角，返回初始位置
};

const float demoRotations[DEMO_STEPS][3] = {
    {0.0, 0.0, 0.0},       // 初始姿态
    {0.0, 0.0, 0.0},       // 保持姿态
    {0.0, M_PI/8, 0.0},    // 轻微倾斜 - 减小倾斜角度
    {0.0, M_PI/8, 0.0},    // 保持倾斜
    {0.0, 0.0, 0.0}        // 返回初始姿态
};

// 各个点位间的停留时间(毫秒) - 增加停留时间
const unsigned long demoDelays[DEMO_STEPS] = {
    1500,  // 初始位置停留1.5秒
    2000,  // 右上角停留2秒
    2000,  // 右下角停留2秒
    2000,  // 左下角停留2秒
    1500   // 左上角停留1.5秒
};

// 默认无奇异的初始姿态(弧度)
const float safeInitialJoints[ARM_DOF] = {
    0.0f,        // 关节1: 0度
    M_PI/6.0f,   // 关节2: 30度
    0.0f,        // 关节3: 0度
    M_PI/4.0f,   // 关节4: 45度
    0.0f,        // 关节5: 0度
    M_PI/4.0f,   // 关节6: 45度
    0.0f         // 关节7: 0度
};

// 获取当前末端执行器位姿
void getCurrentEndEffectorPose() {
    // 获取当前关节角度(假设当前处于关节模式,使用PS2手柄中的值)
    float currentJoints[ARM_DOF];
    for (int i = 0; i < ARM_DOF; i++) {
        // 将舵机值(500-2500)转换为弧度(-PI/2到PI/2)
        currentJoints[i] = (ps2Controller.getJointPosition(i) - 1500) * M_PI / 1000.0f;
    }
    
    // 计算正向运动学
    Matrix4x4 currentPose;
    if (armKinematics.forwardKinematics(currentJoints, &currentPose)) {
        // 提取位置
        float x = currentPose.data[0][3];
        float y = currentPose.data[1][3];
        float z = currentPose.data[2][3];
        
        // 计算姿态角度(从旋转矩阵提取ZYX欧拉角)
        float rx, ry, rz;
        
        // 从旋转矩阵中提取欧拉角
        ry = asin(-currentPose.data[2][0]);
        
        if (cos(ry) > 1e-6) {
            rx = atan2(currentPose.data[2][1], currentPose.data[2][2]);
            rz = atan2(currentPose.data[1][0], currentPose.data[0][0]);
        } else {
            // 奇异点处理
            rx = 0.0f;
            rz = atan2(-currentPose.data[0][1], currentPose.data[1][1]);
        }
        
        // 输出当前末端位姿
        Serial.println("当前末端执行器位姿:");
        Serial.printf("位置: X=%.3f Y=%.3f Z=%.3f (米)\n", x, y, z);
        Serial.printf("姿态: RX=%.3f RY=%.3f RZ=%.3f (弧度)\n", rx, ry, rz);
        Serial.printf("姿态: RX=%.1f RY=%.1f RZ=%.1f (度)\n", 
                     rx * 180.0f / M_PI, ry * 180.0f / M_PI, rz * 180.0f / M_PI);
        
        // 更新目标位置为当前位置
        currentPos[0] = x;
        currentPos[1] = y;
        currentPos[2] = z;
        currentRot[0] = rx;
        currentRot[1] = ry;
        currentRot[2] = rz;
    } else {
        Serial.println("无法计算当前末端位姿");
    }
}

// 设置机械臂到安全的初始姿态
void moveToSafeInitialPose() {
    Serial.println("设置机械臂到安全的初始姿态...");
    
    // 创建舵机数组，用于批量发送命令
    LobotServo servoArray[ARM_DOF];
    int moveTime = 1000; // 统一移动时间(毫秒)
    
    // 设置每个舵机的目标位置
    for (int i = 0; i < ARM_DOF; i++) {
        // 将弧度转换为角度(度)
        float angleDegrees = safeInitialJoints[i] * 180.0f / M_PI;
        
        // 将角度映射到舵机值(0-1000)，按照Config.h中的映射关系
        // 0度对应0，120度对应500，240度对应1000
        float normalizedAngle = angleDegrees + 120.0f; // 将0度偏移到120度
        normalizedAngle = constrain(normalizedAngle, 0.0f, 240.0f); // 确保在有效范围内
        
        // 线性映射到舵机控制值
        int servoPosition = (int)(normalizedAngle * 1000.0f / 240.0f);
        servoPosition = constrain(servoPosition, 0, 1000); // A限制在有效范围
        
        // 设置舵机ID和目标位置
        servoArray[i].ID = i + 1; // 舵机ID从1开始
        servoArray[i].Position = servoPosition;
        
        // 更新PS2控制器内部状态
        ps2Controller.updateJointPosition(i, servoPosition);
        
        Serial.printf("设置舵机ID=%d到位置=%d (角度=%.1f度)\n", 
                     i+1, servoPosition, angleDegrees);
    }
    
    // 一次性发送所有舵机命令
    moveMultipleServos(servoArray, ARM_DOF, moveTime);
    Serial.println("已发送批量舵机命令，等待执行完成...");
    
    // 更新最后移动结束时间
    servoLastMoveEndTime = millis() + moveTime + 100; // 额外增加100ms的缓冲时间
    
    // 等待所有舵机移动完成
    delay(moveTime + 200); // 等待稍微长一点确保动作完成
    
    // 计算初始姿态的正向运动学，获取末端位姿
    Matrix4x4 initialPose;
    if (armKinematics.forwardKinematics(safeInitialJoints, &initialPose)) {
        // 提取位置
        currentPos[0] = initialPose.data[0][3];
        currentPos[1] = initialPose.data[1][3];
        currentPos[2] = initialPose.data[2][3];
        
        // 计算欧拉角
        float ry = asin(-initialPose.data[2][0]);
        
        if (cos(ry) > 1e-6) {
            currentRot[0] = atan2(initialPose.data[2][1], initialPose.data[2][2]);
            currentRot[2] = atan2(initialPose.data[1][0], initialPose.data[0][0]);
        } else {
            currentRot[0] = 0.0f;
            currentRot[2] = atan2(-initialPose.data[0][1], initialPose.data[1][1]);
        }
        currentRot[1] = ry;
        
        Serial.println("已更新当前末端位姿为安全初始姿态");
    }
}

// 运行示例程序
void runDemonstration() {
    unsigned long currentTime = millis();
    
    // 如果示例程序刚开始运行
    if (!isRunningDemo) {
        Serial.println("开始运行运动学示例程序...");
        isRunningDemo = true;
        demoStep = 0;
        demoStartTime = currentTime;
        demoStepStartTime = currentTime;
        
        // 移动到初始位置
        moveToPosition(
            demoPositions[0][0], demoPositions[0][1], demoPositions[0][2],
            demoRotations[0][0], demoRotations[0][1], demoRotations[0][2]
        );
        
        // 更新当前位置和姿态
        for (int i = 0; i < 3; i++) {
            currentPos[i] = demoPositions[0][i];
            currentRot[i] = demoRotations[0][i];
        }
        
        Serial.printf("示例程序: 移动到位置 %d - X:%.2f Y:%.2f Z:%.2f\n", 
                     demoStep + 1, currentPos[0], currentPos[1], currentPos[2]);
        return;
    }
    
    // 检查是否到达下一个步骤的时间
    if (currentTime - demoStepStartTime >= demoDelays[demoStep]) {
        // 移动到下一个位置
        demoStep = (demoStep + 1) % DEMO_STEPS;
        demoStepStartTime = currentTime;
        
        // 移动到新位置
        moveToPosition(
            demoPositions[demoStep][0], demoPositions[demoStep][1], demoPositions[demoStep][2],
            demoRotations[demoStep][0], demoRotations[demoStep][1], demoRotations[demoStep][2]
        );
        
        // 更新当前位置和姿态
        for (int i = 0; i < 3; i++) {
            currentPos[i] = demoPositions[demoStep][i];
            currentRot[i] = demoRotations[demoStep][i];
        }
        
        Serial.printf("示例程序: 移动到位置 %d - X:%.2f Y:%.2f Z:%.2f\n", 
                     demoStep + 1, currentPos[0], currentPos[1], currentPos[2]);
        
        // 如果完成了完整循环，结束示例程序
        if (demoStep == 0) {
            Serial.println("示例程序运行完成!");
            isRunningDemo = false;
        }
    }
}

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
        // 检查是否按下SELECT键切换控制模式
        if (ps2x.ButtonPressed(PSB_SELECT)) {
            kinematicsMode = !kinematicsMode;
            Serial.printf("切换控制模式：%s\n", kinematicsMode ? "运动学模式" : "关节模式");
            
            // 如果切换到运动学模式，先设置安全初始姿态，然后显示当前位姿
            if (kinematicsMode) {
                moveToSafeInitialPose();
                getCurrentEndEffectorPose();
            }
        }
        
        // 根据不同模式处理不同控制逻辑
        if (kinematicsMode) {
            // 在运动学模式下，如果按下三角键，开始或停止示例程序
            if (ps2x.ButtonPressed(PSB_TRIANGLE)) {
                if (isRunningDemo) {
                    isRunningDemo = false;
                    Serial.println("停止运动学示例程序");
                } else {
                    Serial.println("开始运动学示例程序");
                    isRunningDemo = true;
                    demoStep = 0;
                    demoStartTime = millis();
                    demoStepStartTime = millis();
                }
            }
            
            // 如果正在运行示例程序，处理示例程序逻辑
            if (isRunningDemo) {
                runDemonstration();
            } else {
                // 正常的运动学控制逻辑
                handleKinematicsControl();
            }
        } else {
            handleJointControl();
        }
        
        // 夹爪控制在两种模式下都有效
        handleGripper();
    }
}

// 添加获取关节位置的公共方法
int PS2Controller::getJointPosition(int jointIndex) {
    if (jointIndex >= 0 && jointIndex < 7) {
        return joints[jointIndex].currentPos;
    }
    return 500; // 默认中心位置
}

// 更新关节位置
void PS2Controller::updateJointPosition(int jointIndex, int position) {
    if (jointIndex >= 0 && jointIndex < 7) {
        joints[jointIndex].currentPos = position;
        joints[jointIndex].targetSpeed = 0;
    }
}

void PS2Controller::handleKinematicsControl() {
    // 如果正在运行示例程序，跳过手动控制
    if (isRunningDemo) return;
    
    unsigned long currentTime = millis();
    if (currentTime - lastDebounceTime < 50) return; // 消抖延时
    lastDebounceTime = currentTime;
    
    // 检测复位按钮 (X按钮)
    if (ps2x.Button(PSB_CROSS)) {
        resetAll();
        return; // 复位后直接返回
    }
    
    bool positionChanged = false;
    
    // 左摇杆控制XY平面位置
    int lx = ps2x.Analog(PSS_LX);
    int ly = ps2x.Analog(PSS_LY);
    
    // 左摇杆横轴控制X轴位置
    if (abs(lx - 128) > DEAD_ZONE) {
        float delta = map(lx, 0, 255, POS_STEP, -POS_STEP);
        currentPos[0] += delta;
        positionChanged = true;
    }
    
    // 左摇杆纵轴控制Y轴位置
    if (abs(ly - 128) > DEAD_ZONE) {
        float delta = map(ly, 0, 255, POS_STEP, -POS_STEP);
        currentPos[1] += delta;
        positionChanged = true;
    }
    
    // 方向键上下控制Z轴位置
    if (ps2x.Button(PSB_PAD_UP)) {
        currentPos[2] += POS_STEP;
        positionChanged = true;
    } else if (ps2x.Button(PSB_PAD_DOWN)) {
        currentPos[2] -= POS_STEP;
        positionChanged = true;
    }
    
    // 右摇杆控制姿态RX和RY
    int rx = ps2x.Analog(PSS_RX);
    int ry = ps2x.Analog(PSS_RY);
    
    if (abs(rx - 128) > DEAD_ZONE) {
        float delta = map(rx, 0, 255, ROT_STEP, -ROT_STEP);
        currentRot[0] += delta;
        positionChanged = true;
    }
    
    if (abs(ry - 128) > DEAD_ZONE) {
        float delta = map(ry, 0, 255, ROT_STEP, -ROT_STEP);
        currentRot[1] += delta;
        positionChanged = true;
    }
    
    // 方向键左右控制RZ轴旋转
    if (ps2x.Button(PSB_PAD_RIGHT)) {
        currentRot[2] += ROT_STEP;
        positionChanged = true;
    } else if (ps2x.Button(PSB_PAD_LEFT)) {
        currentRot[2] -= ROT_STEP;
        positionChanged = true;
    }
    
    // 如果位置或姿态改变，发送运动学命令
    if (positionChanged && !isPlaying && !isRecording) {
        // 限制位置范围，防止超出工作空间
        for (int i = 0; i < 3; i++) {
            // 限制位置在合理范围内
            currentPos[i] = constrain(currentPos[i], -0.5f, 0.5f);
            // 限制旋转角度
            currentRot[i] = fmod(currentRot[i], 2 * M_PI);
        }
        
        // 打印当前目标位置和姿态
        Serial.println("目标末端执行器位姿:");
        Serial.printf("目标位置: X=%.2f Y=%.2f Z=%.2f (米)\n", 
                     currentPos[0], currentPos[1], currentPos[2]);
        Serial.printf("目标姿态: RX=%.2f RY=%.2f RZ=%.2f (弧度)\n", 
                     currentRot[0], currentRot[1], currentRot[2]);
        
        // 发送运动学命令
        moveToPosition(
            currentPos[0], currentPos[1], currentPos[2], 
            currentRot[0], currentRot[1], currentRot[2]
        );
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
        
        // 重置运动学模式位置和姿态
        currentPos[0] = 0.2f;
        currentPos[1] = 0.0f;
        currentPos[2] = 0.4f;
        currentRot[0] = 0.0f;
        currentRot[1] = 0.0f;
        currentRot[2] = 0.0f;
    }
}

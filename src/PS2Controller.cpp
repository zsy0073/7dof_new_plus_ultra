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
    
    // 初始化轨迹相关变量
    isTrajectoryRunning = false;
    trajectoryExecutor = nullptr;
    
    // 初始化轨迹记录变量
    trajectoryPointCount = 0;
    for (int i = 0; i < MAX_TRAJECTORY_POINTS; i++) {
        for (int j = 0; j < 7; j++) {
            trajectoryJointAngles[i][j] = 0;
        }
    }
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
        // 检测方框键按下，输出轨迹记录矩阵
        if (ps2x.ButtonPressed(PSB_SQUARE)) {
            Serial.println("方框键按下，输出关节角度矩阵");
            outputJointAnglesMatrix();
        }
        
        // 检测三角键按下，启动搬运演示
        if (ps2x.ButtonPressed(PSB_TRIANGLE)) {
            Serial.println("三角键按下，启动搬运演示");
            handleDemoPickPlace();
        }
        
        // 处理控制逻辑
        handleJointControl();
        handleGripper();
        
        // 如果轨迹执行器正在运行，记录当前关节角度
        if (trajectoryExecutor != nullptr && !trajectoryExecutor->isTrajectoryFinished()) {
            recordJointAngles();
            
            // 检查轨迹是否已执行完成
            if (trajectoryExecutor->isTrajectoryFinished()) {
                isTrajectoryRunning = false;
                Serial.println("轨迹执行完成，共记录角度点数: " + String(trajectoryPointCount));
            }
        }
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

// 处理三角键的搬运演示功能
void PS2Controller::handleDemoPickPlace() {
    // 检查是否可以执行轨迹（当前没有其他轨迹在执行）
    if (isTrajectoryRunning || isPlaying || isRecording) {
        Serial.println("无法启动搬运演示：当前有任务正在执行");
        return;
    }
    
    // 参考MATLAB脚本中的搬运位置定义
    // 创建Pick&Place轨迹命令
    TrajectoryCommand cmd;
    cmd.type = TrajectoryCommandType::PICK_PLACE;
    
    // A点位置 - 工作台右侧
    Matrix4d pick_pos = Matrix4d::Identity();
    // 设置末端执行器朝下（绕Y轴旋转180度）
    Matrix3d Ry180;
    Ry180 << -1, 0, 0,
              0, 1, 0,
              0, 0, -1;
    pick_pos.block<3,3>(0,0) = Ry180;
    // 设置位置 x前方25cm，y右侧20cm，z高度2cm
    pick_pos(0,3) = 0.25;  // x
    pick_pos(1,3) = 0.2;   // y
    pick_pos(2,3) = 0.02;  // z
    
    // B点位置 - 工作台左侧
    Matrix4d place_pos = Matrix4d::Identity();
    place_pos.block<3,3>(0,0) = Ry180;
    // 设置位置 x前方25cm，y左侧20cm，z高度2cm
    place_pos(0,3) = 0.25;  // x
    place_pos(1,3) = -0.2;  // y
    place_pos(2,3) = 0.02;  // z
    
    // 提取位置和朝向信息给命令
    cmd.position = Vector3d(pick_pos(0,3), pick_pos(1,3), pick_pos(2,3));
    
    // 从旋转矩阵提取RPY角度
    Matrix3d R = pick_pos.block<3,3>(0,0);
    double roll = atan2(R(2,1), R(2,2));
    double pitch = atan2(-R(2,0), sqrt(R(2,1)*R(2,1) + R(2,2)*R(2,2)));
    double yaw = atan2(R(1,0), R(0,0));
    cmd.orientation = Vector3d(roll, pitch, yaw);
    
    // 设置放置位置
    cmd.viaPoint = Vector3d(place_pos(0,3), place_pos(1,3), place_pos(2,3));
    
    // 设置提升高度和执行时间
    cmd.liftHeight = 0.1;  // 提升高度10cm
    cmd.duration = 10.0;   // 总执行时间10秒
    
    // 如果轨迹执行器可用，执行轨迹
    if (trajectoryExecutor != nullptr) {
        Serial.println("启动搬运演示：从右侧工作台到左侧工作台");
        
        // 在执行新轨迹前重置关节角度记录
        resetJointAngleRecords();
        
        if (trajectoryExecutor->executeCommand(cmd)) {
            isTrajectoryRunning = true;
            Serial.println("搬运演示轨迹开始执行");
        } else {
            Serial.println("搬运演示轨迹执行失败");
        }
    } else {
        Serial.println("轨迹执行器未初始化，无法执行搬运演示");
    }
}

// 记录当前关节角度
void PS2Controller::recordJointAngles() {
    // 确保数组不会越界
    if (trajectoryPointCount < MAX_TRAJECTORY_POINTS) {
        // 记录所有7个关节的当前角度
        for (int i = 0; i < 7; i++) {
            trajectoryJointAngles[trajectoryPointCount][i] = joints[i].currentPos;
        }
        trajectoryPointCount++;
        
        // 打印记录信息
        Serial.printf("记录点 #%d: ", trajectoryPointCount);
        for (int i = 0; i < 7; i++) {
            Serial.printf("%d ", joints[i].currentPos);
        }
        Serial.println();
    } else {
        Serial.println("警告: 轨迹点数已达上限，无法继续记录");
    }
}

// 输出记录的关节角度矩阵到串口
void PS2Controller::outputJointAnglesMatrix() {
    if (trajectoryPointCount == 0) {
        Serial.println("没有记录的轨迹点数据");
        return;
    }
    
    Serial.println("\n=== 关节角度矩阵 ===");
    // 列标题
    Serial.println("点序号, 关节1, 关节2, 关节3, 关节4, 关节5, 关节6, 关节7");
    
    // 输出每一行数据
    for (int i = 0; i < trajectoryPointCount; i++) {
        Serial.printf("%d, ", i+1);
        for (int j = 0; j < 7; j++) {
            // 将脉冲值(0-1000)转换为角度值(0-240度)
            float angleDeg = pulseToAngle(trajectoryJointAngles[i][j]);
            // 计算相对于中值(120度)的偏差，范围为-120到120度
            float relativeAngle = angleDeg - 120.0f;
            // 输出相对角度值，保留两位小数
            Serial.printf("%.2f", relativeAngle);
            if (j < 6) Serial.print(", ");
        }
        Serial.println();
    }
    
    Serial.println("=== 矩阵结束 ===\n");
}

// 重置关节角度记录
void PS2Controller::resetJointAngleRecords() {
    trajectoryPointCount = 0;
    for (int i = 0; i < MAX_TRAJECTORY_POINTS; i++) {
        for (int j = 0; j < 7; j++) {
            trajectoryJointAngles[i][j] = 0;
        }
    }
    Serial.println("重置关节角度记录点");
}

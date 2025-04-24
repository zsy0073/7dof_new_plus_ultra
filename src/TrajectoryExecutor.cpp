#include "TrajectoryExecutor.h"
#include <Arduino.h>

TrajectoryExecutor::TrajectoryExecutor(TrajectoryPlanner& planner, RobotKinematics& kinematics)
    : planner_(planner), kinematics_(kinematics) {
    // 初始化当前关节角度为零
    current_angles_ = VectorXd::Zero(ARM_DOF);
}

bool TrajectoryExecutor::parseCommand(const String& cmdStr, TrajectoryCommand& cmd) {
    // 分割命令字符串
    int spaceIndex = cmdStr.indexOf(' ');
    if(spaceIndex == -1) return false;
    
    String type = cmdStr.substring(0, spaceIndex);
    String params = cmdStr.substring(spaceIndex + 1);
    
    // 解析命令类型
    if(type == "JOINT") {
        cmd.type = TrajectoryCommandType::JOINT_SPACE;
        // 格式: JOINT angle1 angle2 ... angle7 duration
        cmd.jointAngles = VectorXd(7);
        
        for(int i = 0; i < 7; i++) {
            spaceIndex = params.indexOf(' ');
            if(spaceIndex == -1) return false;
            cmd.jointAngles(i) = params.substring(0, spaceIndex).toFloat() * M_PI / 180.0;
            params = params.substring(spaceIndex + 1);
        }
        cmd.duration = params.toFloat();
    }
    else if(type == "LINE") {
        cmd.type = TrajectoryCommandType::LINE;
        // 格式: LINE x y z roll pitch yaw duration
        cmd.position = Vector3d::Zero();
        cmd.orientation = Vector3d::Zero();
        
        // 解析位置
        for(int i = 0; i < 3; i++) {
            spaceIndex = params.indexOf(' ');
            if(spaceIndex == -1) return false;
            cmd.position(i) = params.substring(0, spaceIndex).toFloat();
            params = params.substring(spaceIndex + 1);
        }
        
        // 解析姿态(RPY角)
        for(int i = 0; i < 3; i++) {
            spaceIndex = params.indexOf(' ');
            if(spaceIndex == -1) return false;
            cmd.orientation(i) = params.substring(0, spaceIndex).toFloat() * M_PI / 180.0;
            params = params.substring(spaceIndex + 1);
        }
        
        cmd.duration = params.toFloat();
    }
    else if(type == "ARC") {
        cmd.type = TrajectoryCommandType::ARC;
        // 格式: ARC x y z vx vy vz roll pitch yaw duration
        cmd.position = Vector3d::Zero();
        cmd.viaPoint = Vector3d::Zero();
        cmd.orientation = Vector3d::Zero();
        
        // 解析目标位置
        for(int i = 0; i < 3; i++) {
            spaceIndex = params.indexOf(' ');
            if(spaceIndex == -1) return false;
            cmd.position(i) = params.substring(0, spaceIndex).toFloat();
            params = params.substring(spaceIndex + 1);
        }
        
        // 解析经过点
        for(int i = 0; i < 3; i++) {
            spaceIndex = params.indexOf(' ');
            if(spaceIndex == -1) return false;
            cmd.viaPoint(i) = params.substring(0, spaceIndex).toFloat();
            params = params.substring(spaceIndex + 1);
        }
        
        // 解析姿态
        for(int i = 0; i < 3; i++) {
            spaceIndex = params.indexOf(' ');
            if(spaceIndex == -1) return false;
            cmd.orientation(i) = params.substring(0, spaceIndex).toFloat() * M_PI / 180.0;
            params = params.substring(spaceIndex + 1);
        }
        
        cmd.duration = params.toFloat();
    }
    else if(type == "PICK_PLACE") {
        cmd.type = TrajectoryCommandType::PICK_PLACE;
        // 格式: PICK_PLACE px py pz pr pp py dx dy dz dr dp dy height duration
        Vector3d pickPos = Vector3d::Zero();
        Vector3d pickRPY = Vector3d::Zero();
        Vector3d placePos = Vector3d::Zero();
        Vector3d placeRPY = Vector3d::Zero();
        
        // 解析抓取位置和姿态
        for(int i = 0; i < 3; i++) {
            spaceIndex = params.indexOf(' ');
            if(spaceIndex == -1) return false;
            pickPos(i) = params.substring(0, spaceIndex).toFloat();
            params = params.substring(spaceIndex + 1);
        }
        for(int i = 0; i < 3; i++) {
            spaceIndex = params.indexOf(' ');
            if(spaceIndex == -1) return false;
            pickRPY(i) = params.substring(0, spaceIndex).toFloat() * M_PI / 180.0;
            params = params.substring(spaceIndex + 1);
        }
        
        // 解析放置位置和姿态
        for(int i = 0; i < 3; i++) {
            spaceIndex = params.indexOf(' ');
            if(spaceIndex == -1) return false;
            placePos(i) = params.substring(0, spaceIndex).toFloat();
            params = params.substring(spaceIndex + 1);
        }
        for(int i = 0; i < 3; i++) {
            spaceIndex = params.indexOf(' ');
            if(spaceIndex == -1) return false;
            placeRPY(i) = params.substring(0, spaceIndex).toFloat() * M_PI / 180.0;
            params = params.substring(spaceIndex + 1);
        }
        
        // 解析提升高度和持续时间
        spaceIndex = params.indexOf(' ');
        if(spaceIndex == -1) return false;
        cmd.liftHeight = params.substring(0, spaceIndex).toFloat();
        params = params.substring(spaceIndex + 1);
        cmd.duration = params.toFloat();
        
        // 构建位姿矩阵 - 直接使用欧拉角计算旋转矩阵
        // 欧拉角 (RPY) 转换为旋转矩阵
        Matrix3d R_pick = Matrix3d::Identity();
        double sr = sin(pickRPY(0)), cr = cos(pickRPY(0));
        double sp = sin(pickRPY(1)), cp = cos(pickRPY(1));
        double sy = sin(pickRPY(2)), cy = cos(pickRPY(2));
        
        R_pick(0,0) = cy*cp;    R_pick(0,1) = cy*sp*sr-sy*cr;    R_pick(0,2) = cy*sp*cr+sy*sr;
        R_pick(1,0) = sy*cp;    R_pick(1,1) = sy*sp*sr+cy*cr;    R_pick(1,2) = sy*sp*cr-cy*sr;
        R_pick(2,0) = -sp;      R_pick(2,1) = cp*sr;             R_pick(2,2) = cp*cr;
        
        Matrix3d R_place = Matrix3d::Identity();
        sr = sin(placeRPY(0)), cr = cos(placeRPY(0));
        sp = sin(placeRPY(1)), cp = cos(placeRPY(1));
        sy = sin(placeRPY(2)), cy = cos(placeRPY(2));
        
        R_place(0,0) = cy*cp;    R_place(0,1) = cy*sp*sr-sy*cr;    R_place(0,2) = cy*sp*cr+sy*sr;
        R_place(1,0) = sy*cp;    R_place(1,1) = sy*sp*sr+cy*cr;    R_place(1,2) = sy*sp*cr-cy*sr;
        R_place(2,0) = -sp;      R_place(2,1) = cp*sr;             R_place(2,2) = cp*cr;
        
        Matrix4d T_pick = Matrix4d::Identity();
        T_pick.block<3,3>(0,0) = R_pick;
        T_pick.block<3,1>(0,3) = pickPos;
        
        Matrix4d T_place = Matrix4d::Identity();
        T_place.block<3,3>(0,0) = R_place;
        T_place.block<3,1>(0,3) = placePos;
        
        cmd.position = pickPos;  // 存储抓取位置
        cmd.orientation = pickRPY;  // 存储抓取姿态
        cmd.viaPoint = placePos;  // 使用viaPoint存储放置位置
    }
    else {
        return false;
    }
    
    return true;
}

bool TrajectoryExecutor::executeCommand(const TrajectoryCommand& cmd) {
    if(isExecuting_) return false;
    
    // 确保计算队列已设置
    if(calcCommandQueue_ == nullptr || calcResultQueue_ == nullptr) {
        Serial.println("错误: 轨迹计算队列未设置");
        return false;
    }
    
    // 发送命令到计算任务
    if(xQueueSend(calcCommandQueue_, &cmd, pdMS_TO_TICKS(500)) != pdTRUE) {
        Serial.println("错误: 发送轨迹计算命令失败");
        return false;
    }
    
    Serial.println("已发送轨迹计算命令，等待计算完成...");
    
    // 等待结果 (最多5秒)
    bool success = false;
    if(xQueueReceive(calcResultQueue_, &success, pdMS_TO_TICKS(5000)) != pdTRUE) {
        Serial.println("错误: 等待轨迹计算结果超时");
        return false;
    }
    
    if(success) {
        // 开始执行轨迹
        currentPoint_ = 0;
        startTime_ = millis();
        isExecuting_ = true;
        Serial.println("轨迹计算成功，开始执行");
        return true;
    } else {
        Serial.println("轨迹计算失败");
        return false;
    }
}

void TrajectoryExecutor::sendToServos(const VectorXd& angles) {
    // 创建舵机批量控制数组
    LobotServo servoArray[7];
    
    // 将弧度转换为控制值(0-1000)
    for(int i = 0; i < 7; i++) {
        float angle_deg = angles(i) * 180.0f / M_PI;
        int pulse = angleToPulse(angle_deg);
        servoArray[i].ID = jointServos[i];
        servoArray[i].Position = pulse;
    }
    
    // 设置一个合理的移动时间(100ms)
    const int moveTime = 100;
    
    // 批量控制所有舵机
    moveMultipleServos(servoArray, 7, moveTime);
}

void TrajectoryExecutor::update() {
    if(!isExecuting_) return;
    
    unsigned long currentTime = millis();
    float elapsedTime = (currentTime - startTime_) / 1000.0f;  // 转换为秒
    
    // 找到当前时间对应的轨迹点
    while(currentPoint_ < trajectory_.rows() && 
          timePoints_(currentPoint_) <= elapsedTime) {
        // 发送关节角度给舵机
        VectorXd angles = trajectory_.row(currentPoint_);
        sendToServos(angles);
        
        // 更新当前关节角度
        current_angles_ = angles;
        
        currentPoint_++;
    }
    
    // 检查轨迹是否完成
    if(currentPoint_ >= trajectory_.rows()) {
        isExecuting_ = false;
    }
}

bool TrajectoryExecutor::isTrajectoryFinished() const {
    return !isExecuting_;
}

float TrajectoryExecutor::getProgress() const {
    if(!isExecuting_) return 1.0f;
    if(trajectory_.rows() == 0) return 0.0f;
    
    return static_cast<float>(currentPoint_) / trajectory_.rows();
}



void TrajectoryExecutor::setCalculationQueues(QueueHandle_t commandQueue, QueueHandle_t resultQueue) {
    calcCommandQueue_ = commandQueue;
    calcResultQueue_ = resultQueue;
    
    // 创建互斥锁保护轨迹数据
    if(trajectoryMutex_ == nullptr) {
        trajectoryMutex_ = xSemaphoreCreateMutex();
    }
    
    Serial.println("轨迹计算队列已设置");
}

bool TrajectoryExecutor::setTrajectory(const MatrixXd& trajectory, const VectorXd& timePoints) {
    if(trajectoryMutex_ == nullptr) {
        return false;
    }
    
    if(xSemaphoreTake(trajectoryMutex_, pdMS_TO_TICKS(1000)) != pdTRUE) {
        Serial.println("无法获取轨迹互斥锁");
        return false;
    }
    
    try {
        // 安全地复制轨迹数据
        trajectory_ = trajectory;
        timePoints_ = timePoints;
        
        xSemaphoreGive(trajectoryMutex_);
        return true;
    }
    catch(...) {
        xSemaphoreGive(trajectoryMutex_);
        Serial.println("设置轨迹数据时发生异常");
        return false;
    }
}

VectorXd TrajectoryExecutor::getCurrentJointAngles() const {
    VectorXd current_q = VectorXd::Zero(ARM_DOF);
    
    // 从舵机读取当前关节角度
    for(int i = 0; i < ARM_DOF; i++) {
        // 从全局状态获取当前位置
        int pulse = armStatus.joints[i];
        float angle_deg = pulseToAngle(pulse);
        current_q(i) = angle_deg * M_PI / 180.0;
    }
    
    // 注意：这里不能直接修改，因为方法是const修饰的
    // 需要在非const方法中更新current_angles_
    
    return current_q;
}

// 添加一个安全的无奇异初始位置方法
bool TrajectoryExecutor::moveToSafePose() {
    // 创建一个安全的初始姿态，避开奇异点
    VectorXd safe_q = VectorXd::Zero(ARM_DOF);
    // 这个姿态应该避开机械臂的奇异点
    // 例如：轻度弯曲每个关节，避免关节完全伸直或重合
    safe_q << 0.0, M_PI/6, 0.0, M_PI/3, 0.0, M_PI/4, 0.0;
    
    // 检查该位置是否有效
    Matrix4d test_pose;
    if (!kinematics_.forwardKinematics(safe_q, test_pose)) {
        Serial.println("警告: 无法计算安全位置的正向运动学");
        return false;
    }
    
    // 使用现有方法移动到安全位置
    Serial.println("正在移动到安全的无奇异初始位置...");
    
    // 直接使用现有的sendToServos方法发送舵机命令
    sendToServos(safe_q);
    
    // 等待移动完成
    delay(2000); // 给予充足时间完成移动
    
    // 更新当前关节角度
    current_angles_ = safe_q;
    
    Serial.println("已移动到安全的无奇异初始位置");
    return true;
}

bool TrajectoryExecutor::executeExampleTrajectory() {
    if(isExecuting_) {
        Serial.println("错误: 当前正在执行轨迹，无法启动示例轨迹");
        return false;
    }
    
    // 创建拾放轨迹命令
    TrajectoryCommand cmd;
    cmd.type = TrajectoryCommandType::PICK_PLACE;
    
    // 设置拾取位置: 前方25cm，右侧20cm，高度2cm
    cmd.position = Vector3d(0.25, 0.2, 0.02);
    
    // 设置放置位置: 前方25cm，左侧20cm，高度2cm
    cmd.viaPoint = Vector3d(0.25, -0.2, 0.02);
    
    // 设置姿态: 垂直向下抓取 (0, 180, 0)度 - 转换为弧度
    cmd.orientation = Vector3d(0, M_PI, 0);
    
    // 设置抬升高度和持续时间
    cmd.liftHeight = 0.1;    // 抬升10cm
    cmd.duration = 30.0;     // 30秒完成整个轨迹
    
    // 输出示例轨迹信息
    Serial.println("执行示例拾放轨迹:");
    Serial.printf("  拾取位置: [%.2f, %.2f, %.2f]\n", 
                 cmd.position(0), cmd.position(1), cmd.position(2));
    Serial.printf("  放置位置: [%.2f, %.2f, %.2f]\n", 
                 cmd.viaPoint(0), cmd.viaPoint(1), cmd.viaPoint(2));
    Serial.printf("  抬升高度: %.2f, 持续时间: %.1f秒\n", 
                 cmd.liftHeight, cmd.duration);
    
    // 执行命令
    return executeCommand(cmd);
}
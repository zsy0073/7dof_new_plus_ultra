#include "TrajectoryExecutor.h"
#include <Arduino.h>

TrajectoryExecutor::TrajectoryExecutor(TrajectoryPlanner& planner, RobotKinematics& kinematics)
    : planner_(planner), kinematics_(kinematics) {
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
    
    try {
        // 获取当前机械臂状态
        VectorXd current_q = VectorXd::Zero(7);
        
        // 从舵机读取当前关节角度
        for(int i = 0; i < 7; i++) {
            // 从全局状态获取当前位置
            int pulse = armStatus.joints[i];
            float angle_deg = pulseToAngle(pulse);
            current_q(i) = angle_deg * M_PI / 180.0;
        }
        
        // 创建一个位姿矩阵
        Matrix4d current_pose = Matrix4d::Identity();
        // 调用前向运动学计算当前位姿
        kinematics_.forwardKinematics(current_q, current_pose);
        
        // 根据命令类型生成轨迹
        switch(cmd.type) {
            case TrajectoryCommandType::JOINT_SPACE: {
                planner_.planJointTrajectory(current_q, cmd.jointAngles, 
                                          cmd.duration, trajectory_, timePoints_);
                break;
            }
            case TrajectoryCommandType::LINE: {
                Matrix4d target_pose = Matrix4d::Identity();
                
                // 直接使用 RPY 计算旋转矩阵
                Matrix3d R_target = Matrix3d::Identity();
                double sr = sin(cmd.orientation(0)), cr = cos(cmd.orientation(0));
                double sp = sin(cmd.orientation(1)), cp = cos(cmd.orientation(1));
                double sy = sin(cmd.orientation(2)), cy = cos(cmd.orientation(2));
                
                R_target(0,0) = cy*cp;    R_target(0,1) = cy*sp*sr-sy*cr;    R_target(0,2) = cy*sp*cr+sy*sr;
                R_target(1,0) = sy*cp;    R_target(1,1) = sy*sp*sr+cy*cr;    R_target(1,2) = sy*sp*cr-cy*sr;
                R_target(2,0) = -sp;      R_target(2,1) = cp*sr;             R_target(2,2) = cp*cr;
                
                target_pose.block<3,3>(0,0) = R_target;
                target_pose.block<3,1>(0,3) = cmd.position;
                
                planner_.planCartesianLine(current_pose, target_pose,
                                         cmd.duration, kinematics_,
                                         trajectory_, timePoints_);
                break;
            }
            case TrajectoryCommandType::ARC: {
                Matrix4d target_pose = Matrix4d::Identity();
                
                // 直接使用 RPY 计算旋转矩阵
                Matrix3d R_target = Matrix3d::Identity();
                double sr = sin(cmd.orientation(0)), cr = cos(cmd.orientation(0));
                double sp = sin(cmd.orientation(1)), cp = cos(cmd.orientation(1));
                double sy = sin(cmd.orientation(2)), cy = cos(cmd.orientation(2));
                
                R_target(0,0) = cy*cp;    R_target(0,1) = cy*sp*sr-sy*cr;    R_target(0,2) = cy*sp*cr+sy*sr;
                R_target(1,0) = sy*cp;    R_target(1,1) = sy*sp*sr+cy*cr;    R_target(1,2) = sy*sp*cr-cy*sr;
                R_target(2,0) = -sp;      R_target(2,1) = cp*sr;             R_target(2,2) = cp*cr;
                
                target_pose.block<3,3>(0,0) = R_target;
                target_pose.block<3,1>(0,3) = cmd.position;
                
                planner_.planCartesianArc(current_pose, target_pose,
                                        cmd.viaPoint, cmd.duration,
                                        kinematics_, trajectory_, timePoints_);
                break;
            }
            case TrajectoryCommandType::PICK_PLACE: {
                Matrix4d pick_pose = Matrix4d::Identity();
                
                // 直接使用 RPY 计算旋转矩阵
                Matrix3d R_pick = Matrix3d::Identity();
                double sr = sin(cmd.orientation(0)), cr = cos(cmd.orientation(0));
                double sp = sin(cmd.orientation(1)), cp = cos(cmd.orientation(1));
                double sy = sin(cmd.orientation(2)), cy = cos(cmd.orientation(2));
                
                R_pick(0,0) = cy*cp;    R_pick(0,1) = cy*sp*sr-sy*cr;    R_pick(0,2) = cy*sp*cr+sy*sr;
                R_pick(1,0) = sy*cp;    R_pick(1,1) = sy*sp*sr+cy*cr;    R_pick(1,2) = sy*sp*cr-cy*sr;
                R_pick(2,0) = -sp;      R_pick(2,1) = cp*sr;             R_pick(2,2) = cp*cr;
                
                pick_pose.block<3,3>(0,0) = R_pick;
                pick_pose.block<3,1>(0,3) = cmd.position;
                
                Matrix4d place_pose = Matrix4d::Identity();
                place_pose.block<3,3>(0,0) = R_pick;  // 保持相同姿态
                place_pose.block<3,1>(0,3) = cmd.viaPoint;  // 放置位置存储在viaPoint中
                
                planner_.planPickAndPlace(pick_pose, place_pose,
                                        cmd.liftHeight, cmd.duration,
                                        kinematics_, trajectory_, timePoints_);
                break;
            }
        }
        
        // 开始执行轨迹
        currentPoint_ = 0;
        startTime_ = millis();
        isExecuting_ = true;
        return true;
    }
    catch(const std::exception& e) {
        Serial.print("轨迹规划错误: ");
        Serial.println(e.what());
        return false;
    }
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
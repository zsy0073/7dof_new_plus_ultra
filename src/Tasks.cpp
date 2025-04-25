#include "Tasks.h"
#include "ActionGroup.h"  // 添加动作组头文件
#include "esp_task_wdt.h"  // 添加ESP32任务看门狗头文件

// 实例化机械臂状态
ArmStatus armStatus;

// 全局队列和状态变量
QueueHandle_t servoCommandQueue;
QueueHandle_t trajectoryCommandQueue;
// 移除不需要的队列，因为我们将合并任务
// QueueHandle_t calcResultQueue = nullptr;  // 添加轨迹计算结果队列
// QueueHandle_t calcCommandQueue = nullptr; // 添加轨迹计算命令队列
const int queueSize = 20;  // 队列大小

// 轨迹计算起始时间
unsigned long calcStartTime = 0;

// 状态标志
volatile bool isCommandExecuting = false;
volatile bool isTrajectoryRunning = false;

// 全局轨迹执行器
TrajectoryExecutor* trajectoryExecutor = nullptr;

// 看门狗宏定义
#define FEED_WDT() esp_task_wdt_reset()

// 网络服务任务
void networkTask(void *parameter) {
  // 连接WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(pdMS_TO_TICKS(500));
  }

  // 设置并启动Web服务器
  setupWebServer();

  // 状态更新循环
  while (1) {
    sendStatusUpdate(); // 发送状态更新
    ws.cleanupClients();
    vTaskDelay(pdMS_TO_TICKS(50)); // 更新间隔
  }
}

// 处理所有舵机命令的任务 (网络和PS2手柄共用)
void servoControlTask(void *parameter) {
  ServoCommand cmd;
  
  while (1) {
    // 只有在没有命令执行且没有播放动作组时才获取新命令
    if (!isCommandExecuting && !isPlaying && xQueueReceive(servoCommandQueue, &cmd, 0) == pdTRUE) {
      // 设置命令正在执行标志
      isCommandExecuting = true;
      
      if (cmd.isReset) {
        Serial.println("执行命令: [复位所有舵机]");
        // 处理复位命令
        resetServos();
      } else {
        Serial.print("执行命令: [舵机 ID=");
        Serial.print(cmd.servoId);
        Serial.print(" 位置=");
        Serial.print(cmd.position);
        Serial.println("]");
        
        // 处理普通舵机控制命令
        handleServoControl(cmd.servoId, cmd.position);
      }
    }
    
    // 检查当前命令是否执行完毕
    if (isCommandExecuting && millis() >= servoLastMoveEndTime) {
      Serial.println("命令执行完毕，可以执行下一个命令");
      // 重置命令执行标志
      isCommandExecuting = false;
    }
    
    // 检查动作组播放状态
    checkActionGroupStatus();
    
    vTaskDelay(10); // 短暂延时
  }
}

// 显示和温度监测任务
void displayAndTemperatureTask(void *parameter) {
  while (1) {
    float ambientTemp = mlx.readAmbientTempC();
    armStatus.temperature = ambientTemp; // 更新温度数据
    displayMessage();
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

// 添加PS2控制器任务
void ps2ControllerTask(void *parameter) {
  // 初始化PS2控制器
  ps2Controller.init();
  
  // PS2控制循环
  while (1) {
    // 更新PS2控制器状态和处理控制命令
    ps2Controller.update();
    
    // 短暂延时以提高系统稳定性
    vTaskDelay(1); // 最小延迟
  }
}

// 合并轨迹计算和执行为一个统一任务
void unifiedTrajectoryTask(void *parameter) {
    String cmdBuffer = "";
    
    Serial.println("[UNIFIED_TRAJ] 统一轨迹任务已启动");
    
    while(1) {
        // 处理串口命令
        while(Serial.available()) {
            char c = Serial.read();
            if(c == '\n') {
                // 处理示例轨迹命令
                if(cmdBuffer.equalsIgnoreCase("EXAMPLE") || 
                   cmdBuffer.equalsIgnoreCase("示例轨迹")) {
                    
                    if(!isTrajectoryRunning && trajectoryExecutor != nullptr) {
                        Serial.println("执行示例拾放轨迹...");
                        
                        // 在执行前清空轨迹点记录
                        TrajectoryExecutor::resetRecordedPoints();
                        
                        // 获取轨迹规划器和运动学引用
                        TrajectoryPlanner& planner = trajectoryExecutor->getPlanner();
                        RobotKinematics& kinematics = trajectoryExecutor->getKinematics();
                        
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
                        Serial.println("搬运演示轨迹开始执行");
                        Serial.printf("  拾取位置: [%.2f, %.2f, %.2f]\n", 
                                     cmd.position(0), cmd.position(1), cmd.position(2));
                        Serial.printf("  放置位置: [%.2f, %.2f, %.2f]\n", 
                                     cmd.viaPoint(0), cmd.viaPoint(1), cmd.viaPoint(2));
                        Serial.printf("  抬升高度: %.2f, 持续时间: %.1f秒\n", 
                                     cmd.liftHeight, cmd.duration);
                        
                        // 执行轨迹计算和执行 (直接调用内部方法，不使用队列)
                        if(processTrajectoryCommand(cmd)) {
                            Serial.println("搬运演示轨迹执行完毕！");
                        } else {
                            Serial.println("示例轨迹执行失败");
                            displayErrorMessage("示例轨迹失败");
                        }
                    } else {
                        Serial.println("上一个轨迹正在执行中");
                    }
                }
                // 解析并执行轨迹命令
                else if(cmdBuffer.startsWith("JOINT") || 
                        cmdBuffer.startsWith("LINE") || 
                        cmdBuffer.startsWith("ARC") || 
                        cmdBuffer.startsWith("PICK_PLACE")) {
                    
                    if(!isTrajectoryRunning && trajectoryExecutor != nullptr) {
                        // 在执行前清空轨迹点记录
                        TrajectoryExecutor::resetRecordedPoints();
                        
                        // 解析命令
                        TrajectoryCommand cmd;
                        if(trajectoryExecutor->parseCommand(cmdBuffer, cmd)) {
                            // 直接处理命令，不使用队列
                            if(processTrajectoryCommand(cmd)) {
                                Serial.println("所有轨迹点执行完毕！");
                            } else {
                                Serial.println("轨迹执行失败");
                            }
                        } else {
                            Serial.println("命令格式错误");
                        }
                    } else {
                        Serial.println("上一个轨迹正在执行中");
                    }
                }
                cmdBuffer = "";
            } else {
                cmdBuffer += c;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));  // 10ms更新频率
    }
}

// 内部方法：处理轨迹命令 (计算 + 执行)
bool processTrajectoryCommand(const TrajectoryCommand& cmd) {
    if(isTrajectoryRunning || trajectoryExecutor == nullptr) {
        Serial.println("错误: 当前有轨迹在执行或轨迹执行器未初始化");
        return false;
    }
    
    // 设置任务状态
    isTrajectoryRunning = true;
    calcStartTime = millis();
    
    Serial.println("开始计算轨迹...");
    Serial.println("进度: 准备运动 0%");
    displayProgressWithPoints("轨迹计算", 0, 0, 100); // 初始显示0%进度
    FEED_WDT(); // 喂狗
    
    // 获取轨迹规划器和运动学引用
    TrajectoryPlanner& planner = trajectoryExecutor->getPlanner();
    RobotKinematics& kinematics = trajectoryExecutor->getKinematics();
    
    // 在轨迹规划前，先将机械臂移动到无奇异初始位置
    Serial.println("在轨迹规划前，先将机械臂移动到无奇异初始位置");
    trajectoryExecutor->moveToSafePose();
    Serial.println("机械臂已到达安全初始位置，开始轨迹计算");
    FEED_WDT(); // 喂狗
    
    MatrixXd trajectory;
    VectorXd timePoints;
    bool success = false;
    
    // 输出进度
    Serial.println("进度: 开始轨迹计算");
    
    // 获取当前关节角度
    VectorXd current_q = trajectoryExecutor->getCurrentJointAngles();
    
    // 输出调试信息
    Serial.println("当前关节角度 (弧度):");
    for(int i = 0; i < ARM_DOF; i++) {
        Serial.printf("  关节%d: %.4f\n", i+1, current_q(i));
    }
    
    // 输出进度
    Serial.println("进度: 获取当前姿态 20%");
    FEED_WDT(); // 喂狗
    
    // 计算当前位置的正向运动学
    Matrix4d current_pose;
    if(!kinematics.forwardKinematics(current_q, current_pose)) {
        Serial.println("错误: 无法计算当前位置的正向运动学");
        isTrajectoryRunning = false;
        return false;
    }
    
    // 输出调试信息
    Vector3d current_pos = current_pose.block<3,1>(0,3);
    Serial.printf("当前末端位置: [%.3f, %.3f, %.3f]\n", 
                 current_pos(0), current_pos(1), current_pos(2));
    
    // 输出进度
    Serial.println("进度: 运动学分析 30%");
    FEED_WDT(); // 喂狗
    
    // 输出内存状态
    Serial.printf("内存状态: 空闲堆: %d 字节\n", xPortGetFreeHeapSize());
    
    // 输出进度
    Serial.println("进度: 准备计算轨迹 40%");
    
    // 根据命令类型计算轨迹
    success = false;
    int lastDisplayedProgress = 0; // 记录上次显示的进度百分比
    
    if(cmd.type == TrajectoryCommandType::JOINT_SPACE) {
        // 计算关节空间轨迹
        FEED_WDT(); // 喂狗
        Serial.println("开始计算关节空间轨迹 (初始点数: 0)");
        planner.planJointTrajectory(current_q, cmd.jointAngles, 
                                  cmd.duration, trajectory, timePoints);
        Serial.printf("完成关节空间轨迹计算, 共生成 %d 个点\n", trajectory.rows());
        success = (trajectory.rows() > 0);
    }
    else if(cmd.type == TrajectoryCommandType::LINE) {
        // 创建目标位姿矩阵
        Matrix4d target_pose = Matrix4d::Identity();
        
        // 根据RPY角计算旋转矩阵
        double roll = cmd.orientation(0);
        double pitch = cmd.orientation(1);
        double yaw = cmd.orientation(2);
        
        FEED_WDT(); // 在计算三角函数前喂狗
        
        Matrix3d rotation = Matrix3d::Identity();
        double cr = cos(roll), sr = sin(roll);
        double cp = cos(pitch), sp = sin(pitch);
        double cy = cos(yaw), sy = sin(yaw);
        
        rotation(0,0) = cy*cp;    rotation(0,1) = cy*sp*sr-sy*cr;    rotation(0,2) = cy*sp*cr+sy*sr;
        rotation(1,0) = sy*cp;    rotation(1,1) = sy*sp*sr+cy*cr;    rotation(1,2) = sy*sp*cr-cy*sr;
        rotation(2,0) = -sp;      rotation(2,1) = cp*sr;             rotation(2,2) = cp*cr;
        
        // 设置目标位置和姿态
        target_pose.block<3,3>(0,0) = rotation;
        target_pose.block<3,1>(0,3) = cmd.position;
        
        FEED_WDT(); // 在计算直线轨迹前喂狗
        
        // 计算直线轨迹
        Serial.println("开始计算直线轨迹 (初始点数: 0)");
        planner.planCartesianLine(current_pose, target_pose, 
                                 cmd.duration, kinematics,
                                 trajectory, timePoints);
        Serial.printf("完成直线轨迹计算, 共生成 %d 个点\n", trajectory.rows());
        FEED_WDT(); // 喂狗
        success = (trajectory.rows() > 0);
    }
    else if(cmd.type == TrajectoryCommandType::ARC) {
        // 创建目标位姿矩阵
        Matrix4d target_pose = Matrix4d::Identity();
        
        // 根据RPY角计算旋转矩阵
        double roll = cmd.orientation(0);
        double pitch = cmd.orientation(1);
        double yaw = cmd.orientation(2);
        
        FEED_WDT(); // 在计算三角函数前喂狗
        
        Matrix3d rotation = Matrix3d::Identity();
        double cr = cos(roll), sr = sin(roll);
        double cp = cos(pitch), sp = sin(pitch);
        double cy = cos(yaw), sy = sin(yaw);
        
        rotation(0,0) = cy*cp;    rotation(0,1) = cy*sp*sr-sy*cr;    rotation(0,2) = cy*sp*cr+sy*sr;
        rotation(1,0) = sy*cp;    rotation(1,1) = sy*sp*sr+cy*cr;    rotation(1,2) = sy*sp*cr-cy*sr;
        rotation(2,0) = -sp;      rotation(2,1) = cp*sr;             rotation(2,2) = cp*cr;
        
        // 设置目标位置和姿态
        target_pose.block<3,3>(0,0) = rotation;
        target_pose.block<3,1>(0,3) = cmd.position;
        
        FEED_WDT(); // 在计算圆弧轨迹前喂狗
        
        // 计算圆弧轨迹
        Serial.println("开始计算圆弧轨迹 (初始点数: 0)");
        planner.planCartesianArc(current_pose, target_pose, 
                                cmd.viaPoint, cmd.duration, kinematics,
                                trajectory, timePoints);
        Serial.printf("完成圆弧轨迹计算, 共生成 %d 个点\n", trajectory.rows());
        FEED_WDT(); // 喂狗
        success = (trajectory.rows() > 0);
    }
    else if(cmd.type == TrajectoryCommandType::PICK_PLACE) {
        // 计算拾放轨迹
        
        // 输出轨迹参数信息
        Serial.println("拾放轨迹规划:");
        Serial.printf("当前位置: [%.3f, %.3f, %.3f]\n", 
                     current_pos(0), current_pos(1), current_pos(2));
        Serial.printf("拾取位置: [%.3f, %.3f, %.3f]\n", 
                     cmd.position(0), cmd.position(1), cmd.position(2));
        Serial.printf("放置位置: [%.3f, %.3f, %.3f]\n", 
                     cmd.viaPoint(0), cmd.viaPoint(1), cmd.viaPoint(2));
        Serial.printf("抬升高度: %.3f 米\n", cmd.liftHeight);
        
        FEED_WDT(); // 在计算旋转矩阵前喂狗
        
        // 创建拾取位姿矩阵
        Matrix4d pick_pose = Matrix4d::Identity();
        
        // 根据RPY角计算拾取姿态的旋转矩阵
        double roll = cmd.orientation(0);
        double pitch = cmd.orientation(1);
        double yaw = cmd.orientation(2);
        
        Matrix3d rotation = Matrix3d::Identity();
        double cr = cos(roll), sr = sin(roll);
        double cp = cos(pitch), sp = sin(pitch);
        double cy = cos(yaw), sy = sin(yaw);
        
        rotation(0,0) = cy*cp;    rotation(0,1) = cy*sp*sr-sy*cr;    rotation(0,2) = cy*sp*cr+sy*sr;
        rotation(1,0) = sy*cp;    rotation(1,1) = sy*sp*sr+cy*cr;    rotation(1,2) = sy*sp*cr-cy*sr;
        rotation(2,0) = -sp;      rotation(2,1) = cp*sr;             rotation(2,2) = cp*cr;
        
        // 设置拾取位置和姿态
        pick_pose.block<3,3>(0,0) = rotation;
        pick_pose.block<3,1>(0,3) = cmd.position;
        
        // 创建放置位姿矩阵 (使用相同的姿态但不同的位置)
        Matrix4d place_pose = pick_pose;
        place_pose.block<3,1>(0,3) = cmd.viaPoint;
        
        // 输出进度
        Serial.println("进度: 创建拾取矩阵 40% (初始点数: 0)");
        
        // 输出进度
        Serial.println("进度: 开始计算拾放轨迹 50% (预计点数: 约50点)");
        displayProgressWithPoints("拾放轨迹", 50, 0, 100); // 更新进度到50%
        
        FEED_WDT(); // 在计算拾放轨迹前喂狗
        
        // 计算拾放轨迹
        planner.planPickAndPlace(current_pose, pick_pose, place_pose,
                                cmd.liftHeight, cmd.duration, kinematics,
                                trajectory, timePoints);
        Serial.printf("完成拾放轨迹计算, 共生成 %d 个点\n", trajectory.rows());
        int totalPoints = trajectory.rows();
        displayProgressWithPoints("拾放轨迹", 70, totalPoints, totalPoints); // 更新进度到70%
        FEED_WDT(); // 喂狗
        success = (trajectory.rows() > 0);
    }
    
    // 更新进度并显示点数
    if (success) {
        Serial.printf("进度: 轨迹完成 70%% (已生成点数: %d)\n", trajectory.rows());
        displayProgressWithPoints("轨迹验证", 70, trajectory.rows(), trajectory.rows());
    } else {
        Serial.println("轨迹计算失败，未生成有效点");
        displayErrorMessage("轨迹计算失败");
        isTrajectoryRunning = false;
        return false;
    }
    
    // 验证轨迹
    if(success) {
        // 验证轨迹中的逆运动学解是否都有效
        bool isValid = true;
        int validPoints = 0;
        int totalPoints = trajectory.rows();
        
        Serial.println("开始验证轨迹点的有效性...");
        displayProgressWithPoints("轨迹验证", 75, 0, totalPoints); // 开始验证轨迹
        
        for(int i = 0; i < trajectory.rows(); i++) {
            VectorXd q = trajectory.row(i);
            Matrix4d pose;
            
            // 计算当前进度百分比
            int currentProgress = 30 + (i * 40) / totalPoints; // 验证阶段从30%到70%
            
            // 只有当进度变化超过5%时才更新显示
            if(currentProgress - lastDisplayedProgress >= 5 || i == trajectory.rows() - 1) {
                displayProgressWithPoints("轨迹验证", currentProgress, i+1, totalPoints);
                lastDisplayedProgress = currentProgress;
            }
            
            if(!kinematics.forwardKinematics(q, pose)) {
                Serial.printf("警告: 轨迹点 #%d 无效，无法计算正向运动学\n", i+1);
                isValid = false;
                break;
            }
            
            validPoints++;
            
            // 每验证10个点输出一次进度
            if(i % 10 == 9 || i == trajectory.rows() - 1) {
                Serial.printf("已验证 %d/%d 个轨迹点\n", validPoints, trajectory.rows());
            }
            
            // 每检查10个点喂一次狗
            if(i % 10 == 0) {
                FEED_WDT(); // 喂狗
            }
        }
        success = isValid;
        
        // 更新进度
        if (success) {
            Serial.printf("进度: 验证轨迹完成 80%% (有效点数: %d/%d)\n", 
                        validPoints, trajectory.rows());
        } else {
            Serial.printf("轨迹验证失败，只有 %d/%d 个点有效\n",
                        validPoints, trajectory.rows());
            isTrajectoryRunning = false;
            return false;
        }
    }
    
    // 输出计算结果
    if(success) {
        Serial.println("轨迹计算成功，开始执行");
        Serial.println("计算耗时: " + String(millis() - calcStartTime) + " 毫秒");
        
        // 重要: 计算完成后立即记录所有轨迹点
        Serial.printf("[TRAJ_CALC] 开始记录所有计算的轨迹点 (共 %d 个)...\n", trajectory.rows());
        int totalPoints = trajectory.rows();
        lastDisplayedProgress = 70; // 重置上次显示的进度为70%
        
        for(int i = 0; i < trajectory.rows(); i++) {
            VectorXd angles = trajectory.row(i);
            
            // 计算当前进度百分比
            int currentProgress = 70 + (i * 30) / totalPoints; // 记录阶段从70%到100%
            
            // 只有当进度变化超过5%时或最后一个点才更新显示
            if(currentProgress - lastDisplayedProgress >= 5 || i == trajectory.rows() - 1) {
                displayProgressWithPoints("记录轨迹", currentProgress, i+1, totalPoints);
                lastDisplayedProgress = currentProgress;
                Serial.printf("[TRAJ_CALC] 记录轨迹点 %d/%d, 时间=%.2f秒\n", 
                            i+1, trajectory.rows(), timePoints(i));
            }
            
            trajectoryExecutor->recordTrajectoryPoint(angles);
            FEED_WDT(); // 定期喂狗
        }
        
        // 一旦所有点记录完毕，显示100%进度
        displayProgressWithPoints("轨迹完成", 100, totalPoints, totalPoints);
        
        // 检查轨迹计算总时间并输出
        unsigned long calcEndTime = millis();
        float calcTime = (calcEndTime - calcStartTime) / 1000.0;
        Serial.printf("[TRAJ_CALC] 轨迹计算和验证总时间: %.2f 秒\n", calcTime);
        
        if(!trajectoryExecutor->setTrajectory(trajectory, timePoints)) {
            Serial.println("错误: 无法设置轨迹数据");
            displayErrorMessage("无法设置轨迹数据");
            isTrajectoryRunning = false;
            return false;
        }
        
        // 直接执行轨迹
        Serial.println("开始执行轨迹点...");
        bool execResult = trajectoryExecutor->executeAllTrajectoryPoints();
        
        // 执行完成，输出轨迹记录
        Serial.println("输出轨迹记录:");
        TrajectoryExecutor::outputTrajectoryMatrix();
        
        // 重置状态标志
        isTrajectoryRunning = false;
        
        // 重置显示进度标志，允许恢复温度显示
        armStatus.isShowingProgress = false;
        
        return execResult;
    } else {
        Serial.println("轨迹计算失败");
        isTrajectoryRunning = false;
        
        // 轨迹计算失败时也需要重置显示进度标志
        armStatus.isShowingProgress = false;
        
        return false;
    }
}

// 创建并启动所有任务
void setupTasks() {
  // 创建命令队列
  servoCommandQueue = xQueueCreate(queueSize, sizeof(ServoCommand));
  trajectoryCommandQueue = xQueueCreate(queueSize, sizeof(TrajectoryCommand));
  
  // 不再需要轨迹计算队列
  // calcCommandQueue = xQueueCreate(queueSize, sizeof(TrajectoryCommand));
  // calcResultQueue = xQueueCreate(queueSize, sizeof(bool));
  
  // 创建轨迹执行器实例
  static TrajectoryPlanner planner;
  static RobotKinematics kinematics;
  trajectoryExecutor = new TrajectoryExecutor(planner, kinematics);
  
  // 不再需要设置计算队列
  // trajectoryExecutor->setCalculationQueues(calcCommandQueue, calcResultQueue);
  
  // 设置PS2Controller的轨迹执行器引用
  ps2Controller.setTrajectoryExecutor(trajectoryExecutor);
  
  // 创建网络服务任务在核心1上运行
  xTaskCreatePinnedToCore(
    networkTask,     // 任务函数
    "NetworkTask",   // 任务名称
    8192,           // 堆栈大小
    NULL,           // 任务参数
    1,              // 任务优先级
    NULL,           // 任务句柄
    1               // 运行的核心（Core 1）
  );

  // 创建PS2控制器任务在核心0上运行，优先级高
  xTaskCreatePinnedToCore(
    ps2ControllerTask, // 任务函数
    "PS2ControlTask",  // 任务名称
    24576,             // 增加堆栈大小到24K
    NULL,             // 任务参数
    2,                // 优先级比计算任务低
    NULL,             // 任务句柄
    0                 // 运行的核心（Core 0）
  );

  // 舵机控制任务
  xTaskCreatePinnedToCore(
    servoControlTask,  // 任务函数
    "ServoControlTask", // 任务名称
    2048,              // 堆栈大小
    NULL,              // 任务参数
    1,                 // 优先级比PS2控制器任务低
    NULL,              // 任务句柄
    0                  // 运行的核心（Core 0）
  );

  // 创建屏幕显示与温度采集任务在核心1上运行
  xTaskCreatePinnedToCore(
    displayAndTemperatureTask, // 任务函数
    "DisplayAndTempTask",      // 任务名称
    2048,                      // 堆栈大小
    NULL,                      // 任务参数
    1,                         // 任务优先级
    NULL,                      // 任务句柄
    1                          // 修改到核心1，不干扰控制任务
  );

  // 创建统一的轨迹任务在核心0上运行
  xTaskCreatePinnedToCore(
    unifiedTrajectoryTask,    // 替换为统一任务函数
    "UnifiedTrajTask",        // 任务名称
    32768,                    // 使用大堆栈大小
    NULL,                     // 任务参数
    configMAX_PRIORITIES-1,   // 最高优先级
    NULL,                     // 任务句柄
    0                         // 运行的核心（Core 0）
  );
}

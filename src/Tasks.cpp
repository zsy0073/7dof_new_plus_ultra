#include "Tasks.h"
#include "ActionGroup.h"  // 添加动作组头文件
#include "esp_task_wdt.h"  // 添加ESP32任务看门狗头文件

// 实例化机械臂状态
ArmStatus armStatus;

// 全局队列和状态变量
QueueHandle_t servoCommandQueue;
QueueHandle_t trajectoryCommandQueue;
QueueHandle_t calcResultQueue = nullptr;  // 添加轨迹计算结果队列
QueueHandle_t calcCommandQueue = nullptr; // 添加轨迹计算命令队列
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

// 轨迹执行任务
void trajectoryTask(void *parameter) {
    String cmdBuffer = "";
    
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
                        if(trajectoryExecutor->executeExampleTrajectory()) {
                            isTrajectoryRunning = true;
                            Serial.println("示例轨迹执行开始");
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
                        TrajectoryCommand cmd;
                        if(trajectoryExecutor->parseCommand(cmdBuffer, cmd)) {
                            if(trajectoryExecutor->executeCommand(cmd)) {
                                isTrajectoryRunning = true;
                                Serial.println("轨迹执行开始");
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
        
        // 更新轨迹执行
        if(isTrajectoryRunning && trajectoryExecutor != nullptr) {
            // 添加明确的调试输出
            static unsigned long lastDebugTime = 0;
            unsigned long currentTime = millis();
            if(currentTime - lastDebugTime > 1000) {  // 每秒输出一次调试信息
                Serial.printf("[TRAJ_TASK] 正在执行轨迹，currentPoint=%d, isExecuting=%d\n", 
                             trajectoryExecutor->getCurrentPoint(),
                             trajectoryExecutor->isExecuting() ? 1 : 0);
                lastDebugTime = currentTime;
            }
            
            // 执行更新
            trajectoryExecutor->update();
            
            // 检查轨迹是否完成
            if(trajectoryExecutor->isTrajectoryFinished()) {
                isTrajectoryRunning = false;
                Serial.println("轨迹执行完成");
                
                // 输出轨迹点数据
                Serial.println("输出轨迹记录:");
                TrajectoryExecutor::outputTrajectoryMatrix();
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));  // 10ms更新频率
    }
}

// 修改轨迹计算任务
void trajectoryCalculationTask(void *parameter) {
    // 获取队列句柄
    QueueHandle_t commandQueue = (QueueHandle_t)parameter;
    
    // 轨迹和结果队列
    if(calcResultQueue == nullptr) {
        Serial.println("错误: 结果队列未创建");
        vTaskDelete(nullptr);
        return;
    }
    
    // 初始化工作标志
    bool isWorking = false;
    
    // 注释掉手动初始化看门狗，因为在main.cpp中已经禁用系统看门狗
    // esp_task_wdt_init(10, true);  // 10秒超时，允许重置
    // esp_task_wdt_add(NULL);       // 将当前任务添加到看门狗
    Serial.println("轨迹计算任务已启动");
    
    while(1) {
        // 重置看门狗
        FEED_WDT();
        
        // 等待计算命令
        TrajectoryCommand cmd;
        
        if(xQueueReceive(commandQueue, &cmd, portMAX_DELAY) == pdTRUE) {
            isWorking = true;
            // 记录开始时间
            calcStartTime = millis();
            Serial.println("开始计算轨迹...");
            
            // 输出进度
            Serial.println("进度: 准备运动 0%");
            FEED_WDT(); // 喂狗
            
            // 获取轨迹执行器
            if(trajectoryExecutor == nullptr) {
                Serial.println("错误: 轨迹执行器未初始化");
                bool result = false;
                xQueueSend(calcResultQueue, &result, 0);
                isWorking = false;
                continue;
            }
            
            // 先清空之前记录的轨迹点
            TrajectoryExecutor::resetRecordedPoints();
            
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
            Serial.println("进度: 轨迹计算 10%");
            
            // 获取当前关节角度
            VectorXd current_q = trajectoryExecutor->getCurrentJointAngles();
            
            // 输出调试信息
            Serial.println("当前关节角度 (弧度):");
            for(int i = 0; i < ARM_DOF; i++) {
                Serial.printf("  关节%d: %.4f\n", i, current_q(i));
            }
            
            // 输出进度
            Serial.println("进度: 获取当前姿态 20%");
            FEED_WDT(); // 喂狗
            
            // 计算当前位置的正向运动学
            Matrix4d current_pose;
            if(!kinematics.forwardKinematics(current_q, current_pose)) {
                Serial.println("错误: 无法计算当前位置的正向运动学");
                bool result = false;
                xQueueSend(calcResultQueue, &result, 0);
                isWorking = false;
                continue;
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
            
            if(cmd.type == TrajectoryCommandType::JOINT_SPACE) {
                // 计算关节空间轨迹
                FEED_WDT(); // 喂狗
                planner.planJointTrajectory(current_q, cmd.jointAngles, 
                                          cmd.duration, trajectory, timePoints);
                FEED_WDT(); // 喂狗
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
                planner.planCartesianLine(current_pose, target_pose, 
                                         cmd.duration, kinematics,
                                         trajectory, timePoints);
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
                planner.planCartesianArc(current_pose, target_pose, 
                                        cmd.viaPoint, cmd.duration, kinematics,
                                        trajectory, timePoints);
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
                
                // 先输出进度
                Serial.println("进度: 创建拾取矩阵 40% (点数: 0/50)");
                
                // 再输出进度
                Serial.println("进度: 计算拾放轨迹 50% (点数: 0/50)");
                
                FEED_WDT(); // 在计算拾放轨迹前喂狗
                
                // 计算拾放轨迹
                planner.planPickAndPlace(current_pose, pick_pose, place_pose,
                                        cmd.liftHeight, cmd.duration, kinematics,
                                        trajectory, timePoints);
                FEED_WDT(); // 喂狗
                success = (trajectory.rows() > 0);
            }
            
            // 更新进度
            Serial.println("进度: 轨迹完成 70% (点数: " + 
                         String(trajectory.rows()) + "/" + String(trajectory.rows()) + ")");
            
            // 验证轨迹
            if(success) {
                // 验证轨迹中的逆运动学解是否都有效
                bool isValid = true;
                for(int i = 0; i < trajectory.rows(); i++) {
                    VectorXd q = trajectory.row(i);
                    Matrix4d pose;
                    if(!kinematics.forwardKinematics(q, pose)) {
                        isValid = false;
                        break;
                    }
                    
                    // 每检查10个点喂一次狗
                    if(i % 10 == 0) {
                        FEED_WDT(); // 喂狗
                    }
                }
                success = isValid;
                
                // 更新进度
                Serial.println("进度: 验证轨迹 80% (点数: " + 
                             String(trajectory.rows()) + "/" + String(trajectory.rows()) + ")");
            }
            
            // 输出计算结果
            if(success) {
                Serial.println("轨迹计算完成: " + String(trajectory.rows()) + " 个点");
                Serial.println("计算耗时: " + String(millis() - calcStartTime) + " 毫秒");
                
                // 更新进度
                Serial.println("进度: 设置轨迹 90% (点数: " + 
                             String(trajectory.rows()) + "/" + String(trajectory.rows()) + ")");
                
                FEED_WDT(); // 喂狗
                
                // 重要修改：强制输出轨迹调试信息
                Serial.println("轨迹点信息:");
                for(int i = 0; i < min(5, (int)trajectory.rows()); i++) {
                    Serial.printf("点 %d (%.2f秒): [", i+1, timePoints(i));
                    for(int j = 0; j < ARM_DOF; j++) {
                        Serial.printf("%.2f", trajectory(i, j) * 180.0 / M_PI);
                        if(j < ARM_DOF-1) Serial.print(", ");
                    }
                    Serial.println("]°");
                }
                
                // 重要: 计算完成后立即记录所有轨迹点
                Serial.println("[TRAJ_CALC] 开始记录所有计算的轨迹点...");
                for(int i = 0; i < trajectory.rows(); i++) {
                    VectorXd angles = trajectory.row(i);
                    // 调用记录函数
                    Serial.printf("[TRAJ_CALC] 记录轨迹点 %d/%d, 时间=%.2f秒\n", 
                                 i+1, trajectory.rows(), timePoints(i));
                    trajectoryExecutor->recordTrajectoryPoint(angles);
                    
                    // 短暂延时确保记录完成
                    if(i % 10 == 0) { // 每10个点延时一次，避免频繁延时
                        vTaskDelay(pdMS_TO_TICKS(1));
                        FEED_WDT(); // 喂狗
                    }
                }
                Serial.printf("[TRAJ_CALC] 已记录 %d 个轨迹点\n", trajectory.rows());
                
                FEED_WDT(); // 喂狗
                
                // 将轨迹设置到执行器
                if(!trajectoryExecutor->setTrajectory(trajectory, timePoints)) {
                    Serial.println("错误: 无法设置轨迹数据");
                    success = false;
                } else {
                    Serial.println("设置轨迹成功，发送成功状态到队列");
                }
                
                // 发送结果
                xQueueSend(calcResultQueue, &success, 0);
                Serial.println("成功状态已发送到结果队列");
                
                // 更新进度
                Serial.println("进度: 计算完成 100% (点数: " + 
                             String(trajectory.rows()) + "/" + String(trajectory.rows()) + ")");
            } else {
                Serial.println("轨迹计算失败");
                // 发送结果
                xQueueSend(calcResultQueue, &success, 0);
            }
            
            isWorking = false;
        }
        
        // 短暂延时以避免占用过多CPU
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// 创建并启动所有任务
void setupTasks() {
  // 创建命令队列
  servoCommandQueue = xQueueCreate(queueSize, sizeof(ServoCommand));
  trajectoryCommandQueue = xQueueCreate(queueSize, sizeof(TrajectoryCommand));
  
  // 创建轨迹计算队列
  calcCommandQueue = xQueueCreate(queueSize, sizeof(TrajectoryCommand));
  calcResultQueue = xQueueCreate(queueSize, sizeof(bool));
  
  // 创建轨迹执行器实例
  static TrajectoryPlanner planner;
  static RobotKinematics kinematics;
  trajectoryExecutor = new TrajectoryExecutor(planner, kinematics);
  
  // 重要：设置轨迹执行器的计算队列
  trajectoryExecutor->setCalculationQueues(calcCommandQueue, calcResultQueue);
  
  // 设置PS2Controller的轨迹执行器引用
  ps2Controller.setTrajectoryExecutor(trajectoryExecutor);
  
  // 创建专门的轨迹计算任务在核心0上运行，最高优先级和栈大小
  xTaskCreatePinnedToCore(
    trajectoryCalculationTask,  // 计算轨迹的专门任务
    "TrajCalcTask",            // 任务名称
    32768,                     // 超大堆栈大小(32K)
    calcCommandQueue,          // 任务参数，传递计算命令队列
    configMAX_PRIORITIES-1,    // 最高优先级
    NULL,                      // 任务句柄
    0                          // 运行的核心（Core 0）
  );
  
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

  // 创建轨迹执行任务在核心1上运行
  xTaskCreatePinnedToCore(
    trajectoryTask,      // 任务函数
    "TrajectoryTask",    // 任务名称
    8192,               // 堆栈大小
    NULL,               // 任务参数
    1,                  // 任务优先级
    NULL,               // 任务句柄
    1                   // 运行的核心（Core 1）
  );
}

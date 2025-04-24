#include "Tasks.h"
#include "ActionGroup.h"  // 添加动作组头文件
#include "esp_task_wdt.h"  // 添加ESP32任务看门狗头文件

// 实例化机械臂状态
ArmStatus armStatus;

// 全局队列和状态变量
QueueHandle_t servoCommandQueue;
QueueHandle_t trajectoryCommandQueue;
const int queueSize = 20;  // 队列大小

// 状态标志
volatile bool isCommandExecuting = false;
volatile bool isTrajectoryRunning = false;

// 全局轨迹执行器
TrajectoryExecutor* trajectoryExecutor = nullptr;

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
                        if(trajectoryExecutor->executeExampleTrajectory()) {
                            isTrajectoryRunning = true;
                            Serial.println("示例轨迹执行开始");
                            // 显示当前任务状态
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
            trajectoryExecutor->update();
            
            // 检查轨迹是否完成
            if(trajectoryExecutor->isTrajectoryFinished()) {
                isTrajectoryRunning = false;
                Serial.println("轨迹执行完成");
                // 恢复默认显示
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));  // 10ms更新频率
    }
}

// 轨迹计算任务 - 运行在单独的核心上处理复杂计算
void trajectoryCalculationTask(void *parameter) {
  // 初始化任务
  Serial.println("轨迹计算任务已启动");
  
  // 禁用当前任务的看门狗以避免复杂计算导致超时
  esp_task_wdt_delete(NULL);
  
  // 创建命令通信通道
  static QueueHandle_t calcCommandQueue = xQueueCreate(1, sizeof(TrajectoryCommand));
  static QueueHandle_t calcResultQueue = xQueueCreate(1, sizeof(bool));
  
  // 等待TrajectoryExecutor初始化完成
  while(trajectoryExecutor == nullptr) {
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  
  // 给外部代码设置队列引用
  trajectoryExecutor->setCalculationQueues(calcCommandQueue, calcResultQueue);
  
  // 用于安全保护的额外变量
  int retryCount = 0;
  const int maxRetries = 3;
  
  TrajectoryCommand cmd;
  while(true) {
    // 等待计算命令
    if(xQueueReceive(calcCommandQueue, &cmd, portMAX_DELAY) == pdTRUE) {
      bool success = false;
      
      try {
        // 检查可用内存
        size_t freeHeap = ESP.getFreeHeap();
        if (freeHeap < 30000) {  // 确保至少有30KB可用内存
          Serial.printf("警告: 内存不足 (%d 字节)，可能导致计算失败\n", freeHeap);
          displayErrorMessage("内存不足，无法计算");
          throw std::runtime_error("内存不足");
        }
        
        Serial.println("开始计算轨迹...");
        
        // 首先，将机械臂移动到无奇异初始位置
        displayProgressMessage("准备运动", 0);
        
        Serial.println("在轨迹规划前，先将机械臂移动到无奇异初始位置");
        if (!trajectoryExecutor->moveToSafePose()) {
          Serial.println("错误: 无法移动到安全初始位置");
          displayErrorMessage("无法移动到安全位置");
          throw std::runtime_error("无法移动到安全初始位置");
        }
        
        // 移动完成后等待一小段时间确保机械臂稳定
        vTaskDelay(pdMS_TO_TICKS(2000)); // 增加等待时间到2秒，确保舵机稳定
        Serial.println("机械臂已到达安全初始位置，开始轨迹计算");
        
        // 每个复杂计算步骤后让出CPU时间
        ESP.getFreeHeap(); // 触发垃圾回收
        vTaskDelay(1); // 让出CPU时间

        // 显示进度初始化
        displayProgressMessage("轨迹计算", 10);
        unsigned long startTime = millis();
        
        // 使用try-catch保护所有Eigen操作
        Matrix4d current_pose = Matrix4d::Identity();
        VectorXd current_q;
        
        // 获取当前关节角和显示进度 - 从机械臂状态直接读取，而不是从默认值
        try {
          current_q = trajectoryExecutor->getCurrentJointAngles();
          
          // 确保关节角有效，而不是全零
          bool hasNonZero = false;
          for(int i = 0; i < current_q.size(); i++) {
            if(abs(current_q(i)) > 0.001) {
              hasNonZero = true;
              break;
            }
          }
          
          if(!hasNonZero) {
            Serial.println("警告: 当前关节角度全部为零或接近零，可能未正确获取");
            // 使用 moveToSafePose 中设置的关节角度作为备选
            current_q << 0.0, M_PI/6, 0.0, M_PI/3, 0.0, M_PI/4, 0.0;
          }
          
          Serial.println("当前关节角度 (弧度):");
          for(int i = 0; i < current_q.size(); i++) {
            Serial.printf("  关节%d: %.4f\n", i, current_q(i));
          }
          
          displayProgressMessage("获取当前姿态", 20);
          vTaskDelay(1); // 让出CPU时间
        } catch (const std::exception& e) {
          Serial.printf("获取关节角度异常: %s\n", e.what());
          throw;
        }
        
        // 检查向量维度以避免断言错误
        if(current_q.size() != ARM_DOF) {
          Serial.printf("错误: 关节角向量大小错误 (%d, 应为 %d)\n", current_q.size(), ARM_DOF);
          throw std::runtime_error("关节角向量大小错误");
        }
        
        // 计算当前位姿 - 安全包装
        try {
          if(!trajectoryExecutor->getKinematics().forwardKinematics(current_q, current_pose)) {
            throw std::runtime_error("正向运动学计算失败");
          }
          
          // 打印当前位姿信息，用于调试
          Vector3d position = current_pose.block<3,1>(0,3);
          Serial.printf("当前末端位置: [%.3f, %.3f, %.3f]\n", 
                        position(0), position(1), position(2));
          vTaskDelay(1); // 让出CPU时间
        } catch (const std::exception& e) {
          Serial.printf("正向运动学异常: %s\n", e.what());
          throw;
        }
        
        // 进度显示更新
        displayProgressMessage("运动学分析", 30);
        
        // 获取轨迹规划器引用
        TrajectoryPlanner& planner = trajectoryExecutor->getPlanner();
        
        // 内存分配前检查
        freeHeap = ESP.getFreeHeap();
        Serial.printf("内存状态: 空闲堆: %d 字节\n", freeHeap);
        
        // 创建轨迹存储容器
        MatrixXd trajectory;
        VectorXd timePoints;
        
        // 进度显示更新
        displayProgressMessage("准备计算轨迹", 40);
        vTaskDelay(1); // 让出CPU时间
        
        // 轨迹点计数变量
        int calculatedPoints = 0;
        int estimatedTotalPoints = 0;
        
        // 获取最大点数限制
        int maxPoints = planner.getMaxPoints();
        
        // 根据命令类型规划轨迹 - 每个部分都被try-catch保护
        try {
          // 正确估算总点数 (基于持续时间和最大点数限制)
          switch(cmd.type) {
            case TrajectoryCommandType::JOINT_SPACE: {
              // 检查目标关节角向量尺寸
              if(cmd.jointAngles.size() != ARM_DOF) {
                throw std::runtime_error("目标关节角向量大小错误");
              }
              
              // 估算实际会生成的点数
              estimatedTotalPoints = std::min(maxPoints, (int)ceil(cmd.duration / 0.01) + 1);
              
              // 进度显示更新
              displayProgressWithPoints("计算关节轨迹", 40, 0, estimatedTotalPoints);
              
              // 在耗时计算之前添加短暂延迟让出CPU
              vTaskDelay(1);
              
              planner.planJointTrajectory(current_q, cmd.jointAngles,
                                        cmd.duration, trajectory, timePoints);
              
              // 获取计算出的实际点数
              calculatedPoints = trajectory.rows();
              
              // 进度显示更新
              displayProgressWithPoints("关节轨迹完成", 70, calculatedPoints, calculatedPoints);
              vTaskDelay(1); // 让出CPU时间
              break;
            }
            
            case TrajectoryCommandType::LINE: {
              // 创建目标位姿矩阵
              Matrix4d target_pose = Matrix4d::Identity();
              Matrix3d R_target = trajectoryExecutor->getKinematics().eulerToRotation(cmd.orientation);
              target_pose.block<3,3>(0,0) = R_target;
              target_pose.block<3,1>(0,3) = cmd.position;
              
              // 打印起点终点，用于调试
              Vector3d start_pos = current_pose.block<3,1>(0,3);
              Vector3d end_pos = cmd.position;
              Serial.println("直线轨迹规划:");
              Serial.printf("起点位置: [%.3f, %.3f, %.3f]\n", 
                          start_pos(0), start_pos(1), start_pos(2));
              Serial.printf("终点位置: [%.3f, %.3f, %.3f]\n", 
                          end_pos(0), end_pos(1), end_pos(2));
              
              // 计算路径长度
              double path_length = (end_pos - start_pos).norm();
              Serial.printf("路径长度: %.3f 米\n", path_length);
              
              // 估算实际会生成的点数
              estimatedTotalPoints = std::min(maxPoints, (int)ceil(cmd.duration / 0.01) + 1);
              
              // 进度显示更新
              displayProgressWithPoints("计算直线轨迹", 40, 0, estimatedTotalPoints);
              
              // 在耗时计算之前添加短暂延迟让出CPU
              vTaskDelay(1);
              
              planner.planCartesianLine(current_pose, target_pose,
                                      cmd.duration, trajectoryExecutor->getKinematics(),
                                      trajectory, timePoints);
              
              // 获取计算出的实际点数
              calculatedPoints = trajectory.rows();
              
              // 进度显示更新
              displayProgressWithPoints("直线轨迹完成", 70, calculatedPoints, calculatedPoints);
              vTaskDelay(1); // 让出CPU时间
              break;
            }
            
            case TrajectoryCommandType::ARC: {
              // 创建目标位姿矩阵
              Matrix4d target_pose = Matrix4d::Identity();
              Matrix3d R_target = trajectoryExecutor->getKinematics().eulerToRotation(cmd.orientation);
              target_pose.block<3,3>(0,0) = R_target;
              target_pose.block<3,1>(0,3) = cmd.position;
              
              // 打印信息用于调试
              Vector3d start_pos = current_pose.block<3,1>(0,3);
              Vector3d end_pos = cmd.position;
              Vector3d via_pos = cmd.viaPoint;
              Serial.println("弧线轨迹规划:");
              Serial.printf("起点位置: [%.3f, %.3f, %.3f]\n", 
                          start_pos(0), start_pos(1), start_pos(2));
              Serial.printf("终点位置: [%.3f, %.3f, %.3f]\n", 
                          end_pos(0), end_pos(1), end_pos(2));
              Serial.printf("经过点位置: [%.3f, %.3f, %.3f]\n", 
                          via_pos(0), via_pos(1), via_pos(2));
              
              // 估算实际会生成的点数
              estimatedTotalPoints = std::min(maxPoints, (int)ceil(cmd.duration / 0.01) + 1);
              
              // 进度显示更新
              displayProgressWithPoints("计算弧线轨迹", 40, 0, estimatedTotalPoints);
              
              // 在耗时计算之前添加短暂延迟让出CPU
              vTaskDelay(1);
              
              planner.planCartesianArc(current_pose, target_pose,
                                     cmd.viaPoint, cmd.duration,
                                     trajectoryExecutor->getKinematics(),
                                     trajectory, timePoints);
              
              // 获取计算出的实际点数
              calculatedPoints = trajectory.rows();
              
              // 进度显示更新
              displayProgressWithPoints("弧线轨迹完成", 70, calculatedPoints, calculatedPoints);
              vTaskDelay(1); // 让出CPU时间
              break;
            }
            
            case TrajectoryCommandType::PICK_PLACE: {
              // 为Pick&Place特殊处理
              // 由于它是由6个子轨迹段组成的复合轨迹，我们需要更准确地估算点数
              
              // 安全地创建抓取位姿矩阵
              Matrix4d pick_pose = Matrix4d::Identity();
              Matrix3d R_pick;
              
              try {
                R_pick = trajectoryExecutor->getKinematics().eulerToRotation(cmd.orientation);
                pick_pose.block<3,3>(0,0) = R_pick;
                pick_pose.block<3,1>(0,3) = cmd.position;
              } catch (const std::exception& e) {
                Serial.printf("创建抓取矩阵异常: %s\n", e.what());
                throw;
              }
              
              // 打印拾取放置信息
              Vector3d current_pos = current_pose.block<3,1>(0,3);
              Vector3d pick_pos = cmd.position;
              Vector3d place_pos = cmd.viaPoint;
              Serial.println("拾放轨迹规划:");
              Serial.printf("当前位置: [%.3f, %.3f, %.3f]\n", 
                          current_pos(0), current_pos(1), current_pos(2));
              Serial.printf("拾取位置: [%.3f, %.3f, %.3f]\n", 
                          pick_pos(0), pick_pos(1), pick_pos(2));
              Serial.printf("放置位置: [%.3f, %.3f, %.3f]\n", 
                          place_pos(0), place_pos(1), place_pos(2));
              Serial.printf("抬升高度: %.3f 米\n", cmd.liftHeight);
              
              // 估算总点数 - 复合轨迹通常限制在max_points内
              estimatedTotalPoints = maxPoints;
              
              // 进度显示更新
              displayProgressWithPoints("创建拾取矩阵", 40, 0, estimatedTotalPoints);
              
              // 安全地创建放置位姿矩阵
              Matrix4d place_pose = Matrix4d::Identity();
              try {
                place_pose.block<3,3>(0,0) = R_pick;  // 保持相同姿态
                place_pose.block<3,1>(0,3) = cmd.viaPoint;
              } catch (const std::exception& e) {
                Serial.printf("创建放置矩阵异常: %s\n", e.what());
                throw;
              }
              
              // 进度显示更新
              displayProgressWithPoints("计算拾放轨迹", 50, 0, estimatedTotalPoints);
              
              // 轨迹计算前再次检查内存
              freeHeap = ESP.getFreeHeap();
              if (freeHeap < 15000) {  // 确保至少有15KB可用内存进行计算
                throw std::runtime_error("内存不足，无法计算拾放轨迹");
              }
              
              // 在耗时计算之前添加短暂延迟让出CPU
              vTaskDelay(1);
              
              planner.planPickAndPlace(current_pose, pick_pose, place_pose,
                                     cmd.liftHeight, cmd.duration,
                                     trajectoryExecutor->getKinematics(),
                                     trajectory, timePoints);
              
              // 获取计算出的实际点数
              calculatedPoints = trajectory.rows();
              
              // 进度显示更新
              displayProgressWithPoints("拾放轨迹完成", 70, calculatedPoints, calculatedPoints);
              vTaskDelay(1); // 让出CPU时间
              break;
            }
          }
        } catch (const std::exception& e) {
          Serial.printf("轨迹计算步骤异常: %s\n", e.what());
          throw;
        }
        
        // 检查计算结果的有效性
        if (trajectory.rows() <= 0 || timePoints.size() <= 0) {
          throw std::runtime_error("计算结果无效: 轨迹点数为零");
        }
        
        // 进度显示更新
        displayProgressWithPoints("验证轨迹", 80, calculatedPoints, calculatedPoints);
        
        Serial.printf("轨迹计算完成: %d 个点\n", trajectory.rows());
        unsigned long calcTime = millis() - startTime;
        Serial.printf("计算耗时: %lu 毫秒\n", calcTime);
        
        // 进度显示更新
        displayProgressWithPoints("设置轨迹", 90, calculatedPoints, calculatedPoints);
        vTaskDelay(1); // 让出CPU时间
        
        // 将结果发送回轨迹执行器 - 安全包装
        try {
          success = trajectoryExecutor->setTrajectory(trajectory, timePoints);
          if (!success) {
            throw std::runtime_error("设置轨迹失败");
          }
        } catch (const std::exception& e) {
          Serial.printf("设置轨迹异常: %s\n", e.what());
          throw;
        }
        
        // 进度显示更新 - 100%完成
        displayProgressWithPoints("计算完成", 100, calculatedPoints, calculatedPoints);
        
        // 重置重试计数
        retryCount = 0;
      }
      catch(const std::exception& e) {
        Serial.print("轨迹计算异常: ");
        Serial.println(e.what());
        // 显示错误信息
        displayErrorMessage(String("计算错误: ") + e.what());
        success = false;
        
        // 异常发生时，等待更长时间恢复
        vTaskDelay(pdMS_TO_TICKS(500));
        
        // 墨加重试计数
        retryCount++;
        if (retryCount >= maxRetries) {
          // 如果多次重试失败，尝试释放内存
          Serial.println("多次计算失败，正在尝试释放内存...");
          // 强制进行一次GC (如果有的话)
          ESP.getFreeHeap();
          vTaskDelay(pdMS_TO_TICKS(1000)); // 等待系统稳定
          retryCount = 0;
        }
      }
      catch(...) {
        Serial.println("轨迹计算发生未知异常");
        // 显示错误信息
        displayErrorMessage("未知计算错误");
        success = false;
        
        // 异常发生时，等待更长时间恢复
        vTaskDelay(pdMS_TO_TICKS(1000));
      }
      
      // 返回计算结果
      xQueueSend(calcResultQueue, &success, 0);
      vTaskDelay(pdMS_TO_TICKS(100)); // 留出时间处理结果
    } else {
      // 即使没有任务也要让出CPU时间
      vTaskDelay(1);
    }
  }
}

// 创建并启动所有任务
void setupTasks() {
  // 创建命令队列
  servoCommandQueue = xQueueCreate(queueSize, sizeof(ServoCommand));
  trajectoryCommandQueue = xQueueCreate(queueSize, sizeof(TrajectoryCommand));
  
  // 创建轨迹执行器实例
  static TrajectoryPlanner planner;
  static RobotKinematics kinematics;
  trajectoryExecutor = new TrajectoryExecutor(planner, kinematics);
  
  // 设置PS2Controller的轨迹执行器引用
  ps2Controller.setTrajectoryExecutor(trajectoryExecutor);
  
  // 创建专门的轨迹计算任务在核心0上运行，最高优先级和栈大小
  xTaskCreatePinnedToCore(
    trajectoryCalculationTask,  // 计算轨迹的专门任务
    "TrajCalcTask",            // 任务名称
    32768,                     // 超大堆栈大小(32K)
    NULL,                      // 任务参数
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

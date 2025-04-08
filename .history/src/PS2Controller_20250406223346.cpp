#include "PS2Controller.h"
#include "ActionGroup.h"
#include "Tasks.h" // 添加Tasks.h以获取servoCommandQueue的外部声明

// 实例化PS2手柄对象
PS2X ps2x;

// 状态变量
static ControlMode currentMode = MODE_JOINT;  // 默认为关节控制模式
static int currentJointIndex = 0;             // 当前选中的关节索引(0-6)
static unsigned long lastButtonTime = 0;      // 上次按键时间
static const unsigned long buttonDelay = 200; // 按键延迟，防止快速连按

// 初始化PS2手柄控制器
bool initPS2Controller() {
  Serial.println("初始化PS2手柄控制器...");
  
  // 配置PS2手柄，参数：数据引脚，命令引脚，选通引脚，时钟引脚
  int error = ps2x.config_gamepad(PS2_CLK_PIN, PS2_CMD_PIN, PS2_ATT_PIN, PS2_DAT_PIN, false, false);
  
  if (error == 0) {
    Serial.println("PS2手柄初始化成功");
    return true;
  } else {
    Serial.print("PS2手柄初始化失败，错误码：");
    Serial.println(error);
    return false;
  }
}

// 切换控制模式
void switchControlMode() {
  currentMode = static_cast<ControlMode>((currentMode + 1) % MODE_MAX);
  Serial.print("切换到模式：");
  Serial.println(currentMode == MODE_JOINT ? "关节控制" : "夹持器控制");
}

// 获取当前控制模式
ControlMode getCurrentControlMode() {
  return currentMode;
}

// 获取当前选择的关节索引
int getCurrentJointIndex() {
  return currentJointIndex;
}

// 根据摇杆值调整舵机位置
void adjustServoByJoystick(int jointIndex, int stickValue) {
  // 确保有效的关节索引
  if (jointIndex < 0 || jointIndex >= 7) return;
  
  // 忽略死区内的摇杆值
  if (abs(stickValue - 128) < JOYSTICK_DEADZONE) return;
  
  // 计算新位置
  int currentPosition = armStatus.joints[jointIndex];
  int increment = 0;
  
  // 将摇杆值映射为位置增量
  if (stickValue < 128 - JOYSTICK_DEADZONE) {
    // 向左/上移动
    increment = -map(128 - stickValue, JOYSTICK_DEADZONE, 128, 1, PS2_INCREMENT_STEP);
  } else if (stickValue > 128 + JOYSTICK_DEADZONE) {
    // 向右/下移动
    increment = map(stickValue - 128, JOYSTICK_DEADZONE, 128, 1, PS2_INCREMENT_STEP);
  }
  
  // 计算新位置并约束在有效范围内
  int newPosition = constrain(currentPosition + increment, 0, 1000);
  
  // 仅当位置有变化时才发送命令
  if (newPosition != currentPosition) {
    // 使用预定义的ServoCommand结构体(在Tasks.h中已定义)
    struct ServoCommand cmd;
    
    cmd.servoId = jointServos[jointIndex];
    cmd.position = newPosition;
    cmd.isReset = false;
    
    // 发送命令到队列
    if (xQueueSend(servoCommandQueue, &cmd, 0) == pdTRUE) {
      // 更新状态
      armStatus.joints[jointIndex] = newPosition;
      armStatus.updated = true;
    }
  }
}

// 处理PS2手柄输入
void handlePS2Input() {
  // 读取PS2手柄状态
  if (!ps2x.read_gamepad()) {
    // 读取失败，可能是连接问题
    return;
  }
  
  unsigned long currentTime = millis();
  
  // 使用SELECT键切换控制模式
  if (ps2x.ButtonPressed(PSB_SELECT) && currentTime - lastButtonTime > buttonDelay) {
    switchControlMode();
    lastButtonTime = currentTime;
  }
  
  // 根据当前模式处理输入
  if (currentMode == MODE_JOINT) {
    // 关节控制模式
    
    // 使用L1和R1切换当前控制的关节
    if (ps2x.ButtonPressed(PSB_L1) && currentTime - lastButtonTime > buttonDelay) {
      // 上一个关节
      currentJointIndex = (currentJointIndex + 6) % 7;
      Serial.print("当前选中关节：");
      Serial.println(currentJointIndex);
      lastButtonTime = currentTime;
    }
    
    if (ps2x.ButtonPressed(PSB_R1) && currentTime - lastButtonTime > buttonDelay) {
      // 下一个关节
      currentJointIndex = (currentJointIndex + 1) % 7;
      Serial.print("当前选中关节：");
      Serial.println(currentJointIndex);
      lastButtonTime = currentTime;
    }
    
    // 使用左摇杆Y轴控制当前选中的舵机
    int leftY = ps2x.Analog(PSS_LY);
    adjustServoByJoystick(currentJointIndex, leftY);
    
    // 使用按钮直接控制舵机
    struct ServoCommand cmd;
    cmd.isReset = false;
    
    // 三角形按钮 - 增加当前选中关节位置
    if (ps2x.ButtonPressed(PSB_TRIANGLE) && currentTime - lastButtonTime > buttonDelay) {
      int newPos = constrain(armStatus.joints[currentJointIndex] + PS2_INCREMENT_STEP, 0, 1000);
      cmd.servoId = jointServos[currentJointIndex];
      cmd.position = newPos;
      
      if (xQueueSend(servoCommandQueue, &cmd, 0) == pdTRUE) {
        armStatus.joints[currentJointIndex] = newPos;
        armStatus.updated = true;
      }
      lastButtonTime = currentTime;
    }
    
    // 叉号按钮 - 减少当前选中关节位置
    if (ps2x.ButtonPressed(PSB_CROSS) && currentTime - lastButtonTime > buttonDelay) {
      int newPos = constrain(armStatus.joints[currentJointIndex] - PS2_INCREMENT_STEP, 0, 1000);
      cmd.servoId = jointServos[currentJointIndex];
      cmd.position = newPos;
      
      if (xQueueSend(servoCommandQueue, &cmd, 0) == pdTRUE) {
        armStatus.joints[currentJointIndex] = newPos;
        armStatus.updated = true;
      }
      lastButtonTime = currentTime;
    }
    
  } else if (currentMode == MODE_GRIPPER) {
    // 夹持器控制模式
    
    // 使用左摇杆Y轴控制夹持器
    int leftY = ps2x.Analog(PSS_LY);
    
    // 忽略死区内的摇杆值
    if (abs(leftY - 128) >= JOYSTICK_DEADZONE) {
      int currentPosition = armStatus.gripper;
      int increment = 0;
      
      // 计算增量
      if (leftY < 128 - JOYSTICK_DEADZONE) {
        increment = -map(128 - leftY, JOYSTICK_DEADZONE, 128, 1, PS2_INCREMENT_STEP);
      } else if (leftY > 128 + JOYSTICK_DEADZONE) {
        increment = map(leftY - 128, JOYSTICK_DEADZONE, 128, 1, PS2_INCREMENT_STEP);
      }
      
      int newPosition = constrain(currentPosition + increment, 0, 1000);
      
      // 仅当位置有变化时才发送命令
      if (newPosition != currentPosition) {
        struct ServoCommand cmd;
        
        cmd.servoId = gripperServo;
        cmd.position = newPosition;
        cmd.isReset = false;
        
        if (xQueueSend(servoCommandQueue, &cmd, 0) == pdTRUE) {
          armStatus.gripper = newPosition;
          armStatus.updated = true;
        }
      }
    }
    
    // 三角形按钮 - 增加夹持器位置
    if (ps2x.ButtonPressed(PSB_TRIANGLE) && currentTime - lastButtonTime > buttonDelay) {
      int newPos = constrain(armStatus.gripper + PS2_INCREMENT_STEP, 0, 1000);
      struct ServoCommand cmd;
      cmd.servoId = gripperServo;
      cmd.position = newPos;
      cmd.isReset = false;
      
      if (xQueueSend(servoCommandQueue, &cmd, 0) == pdTRUE) {
        armStatus.gripper = newPos;
        armStatus.updated = true;
      }
      lastButtonTime = currentTime;
    }
    
    // 叉号按钮 - 减少夹持器位置
    if (ps2x.ButtonPressed(PSB_CROSS) && currentTime - lastButtonTime > buttonDelay) {
      int newPos = constrain(armStatus.gripper - PS2_INCREMENT_STEP, 0, 1000);
      struct ServoCommand cmd;
      cmd.servoId = gripperServo;
      cmd.position = newPos;
      cmd.isReset = false;
      
      if (xQueueSend(servoCommandQueue, &cmd, 0) == pdTRUE) {
        armStatus.gripper = newPos;
        armStatus.updated = true;
      }
      lastButtonTime = currentTime;
    }
  }
  
  // 通用控制
  // START按钮 - 复位所有舵机
  if (ps2x.ButtonPressed(PSB_START) && currentTime - lastButtonTime > buttonDelay) {
    struct ServoCommand cmd;
    cmd.isReset = true;
    
    if (xQueueSend(servoCommandQueue, &cmd, 0) == pdTRUE) {
      Serial.println("PS2控制: 复位所有舵机");
      lastButtonTime = currentTime;
    }
  }
  
  // 圆形按钮 - 开始/停止录制动作组
  if (ps2x.ButtonPressed(PSB_CIRCLE) && currentTime - lastButtonTime > buttonDelay) {
    if (isRecording) {
      stopRecordActionGroup();
    } else {
      startRecordActionGroup();
    }
    lastButtonTime = currentTime;
  }
  
  // 方形按钮 - 播放动作组
  if (ps2x.ButtonPressed(PSB_SQUARE) && currentTime - lastButtonTime > buttonDelay) {
    playActionGroup();
    lastButtonTime = currentTime;
  }
  
  // 短暂延迟，降低CPU占用
  vTaskDelay(pdMS_TO_TICKS(20)); // 使用FreeRTOS延时代替delay
}

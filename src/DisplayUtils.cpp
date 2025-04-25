#include "DisplayUtils.h"

// OLED 显示屏和温度传感器对象
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
Adafruit_SSD1306 display(128, 64, &Wire, -1);

// WiFi配置和学生信息
const char* ssid = "404notfound";
const char* password = "404notfound";
const char* studentID = "20250001";
const char* studentName = "ZSY";

// 修改初始化I2C函数，使用 GPIO13 (SDA) 和 GPIO14 (SCL)
void initI2C() {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN); // 使用 GPIO13 和 GPIO14 作为 I2C 引脚
  delay(100); // 等待I2C初始化完成
}

//显示函数
void displayMessage() {
  // 如果正在显示进度，不要切换回温度显示界面
  if (armStatus.isShowingProgress) {
    return;
  }
  
  // 读取环境温度
  float ambientTemp = mlx.readAmbientTempC();
  
  // 在OLED上显示温度
  display.clearDisplay();
  display.setCursor(0, 0);
  
  if (isnan(ambientTemp)) {
    display.println("Sensor Error!");
  } else {
    display.print("Ambient Temp: ");
    display.print(ambientTemp);
    display.print(" C");
  }

  // 显示学号和姓名
  display.setCursor(0, 15);
  display.print("ID: ");
  display.print(studentID);
  display.setCursor(0, 25);
  display.print("Name: ");
  display.print(studentName);

  // 显示舵机状态
  display.setCursor(0, 35);
  display.print("J0-J2: ");
  for(int i=0; i<3; i++){
    display.print(armStatus.joints[i]);
    display.print(" ");
  }
  
  display.setCursor(0, 45);
  display.print("J3-J6: ");
  for(int i=3; i<7; i++){
    display.print(armStatus.joints[i]);
    display.print(" ");
  }
  display.print(" G:");
  display.print(armStatus.gripper);

  display.display(); // 确保更新显示
}

// 初始化显示屏和传感器
void initDisplayAndSensor() {
  if (!mlx.begin()) { // 使用默认I2C地址初始化 MLX90614
    while (1);
  }

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // 使用默认I2C地址初始化 OLED
    while (1);
  }
  // 清除屏幕并显示初始化完成信息
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.println("ready!");
  display.display();
  delay(2000);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  displayMessage(); // 添加调用displayMessage函数更新显示内容
}

// 辅助函数：将中文任务名称转换为英文显示
String convertTaskNameForDisplay(const String& taskName) {
  // 将中文任务名称转换为对应的英文名称
  if (taskName == "轨迹计算") return "Traj Calc";
  if (taskName == "轨迹执行") return "Traj Exec";
  if (taskName == "轨迹完成") return "Traj Done";
  if (taskName == "记录轨迹") return "Record Traj";
  if (taskName == "正在执行") return "Executing";
  if (taskName == "计算中") return "Calculating";
  if (taskName == "验证轨迹") return "Verify Traj";
  if (taskName == "动作组播放") return "Play Actions";
  if (taskName == "动作记录") return "Record Action";
  if (taskName == "正在初始化") return "Initializing";
  if (taskName == "运动准备") return "Motion Ready";
  if (taskName == "拾放轨迹") return "Pick&Place";
  if (taskName == "轨迹验证") return "Traj Verify";
  if (taskName == "无法设置轨迹数据") return "Traj Data Error";
  if (taskName == "错误") return "Error";
  if (taskName == "警告") return "Warning";
  if (taskName == "准备运动") return "Ready Motion";
  if (taskName == "任务") return "Task";
  if (taskName == "进度") return "Progress";
  
  // 如果没有匹配项，直接返回原始任务名（可能是英文）
  return taskName;
}

// 显示进度消息
void displayProgressMessage(const String& task, int progress) {
  // 设置显示进度标志，防止被温度显示中断
  armStatus.isShowingProgress = true;
  
  // 清除屏幕
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  
  // 获取英文任务名称
  String displayTask = convertTaskNameForDisplay(task);
  
  // 显示任务名称
  display.setCursor(0, 0);
  display.print("Task: ");
  display.println(displayTask);
  
  // 显示进度百分比
  display.setCursor(0, 15);
  display.print("Progress: ");
  display.print(progress);
  display.println("%");
  
  // 绘制进度条
  display.drawRect(0, 30, 128, 10, WHITE);
  display.fillRect(2, 32, 124 * progress / 100, 6, WHITE);
  
  // 更新数据到串口
  Serial.print("Progress: ");
  Serial.print(task);
  Serial.print(" ");
  Serial.print(progress);
  Serial.println("%");
  
  // 更新显示
  display.display();
}

// 显示带有点数信息的进度消息
void displayProgressWithPoints(const String& task, int progress, int currentPoints, int totalPoints) {
  // 设置显示进度标志，防止被温度显示中断
  armStatus.isShowingProgress = true;
  
  // 清除屏幕
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  
  // 获取英文任务名称
  String displayTask = convertTaskNameForDisplay(task);
  
  // 显示任务名称
  display.setCursor(0, 0);
  display.print("Task: ");
  display.println(displayTask);
  
  // 显示进度百分比
  display.setCursor(0, 15);
  display.print("Progress: ");
  display.print(progress);
  display.println("%");
  
  // 绘制进度条
  display.drawRect(0, 30, 128, 10, WHITE);
  display.fillRect(2, 32, 124 * progress / 100, 6, WHITE);
  
  // 显示点数信息
  display.setCursor(0, 45);
  display.print("Points: ");
  display.print(currentPoints);
  display.print("/");
  display.print(totalPoints);
  
  // 更新数据到串口
  Serial.print("Progress: ");
  Serial.print(task);
  Serial.print(" ");
  Serial.print(progress);
  Serial.print("% Points: ");
  Serial.print(currentPoints);
  Serial.print("/");
  Serial.println(totalPoints);
  
  // 更新显示
  display.display();
}

// 显示错误消息
void displayErrorMessage(const String& error) {
  // 清除屏幕
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  
  // 显示错误标题
  display.setCursor(0, 0);
  display.println("Error:");
  
  // 转换并显示错误内容
  String displayError = convertTaskNameForDisplay(error);
  display.setCursor(0, 15);
  display.println(displayError);
  
  // 显示提示
  display.setCursor(0, 45);
  display.println("Press button to retry");
  
  // 更新到串口
  Serial.print("Error: ");
  Serial.println(error);
  
  // 更新显示
  display.display();
}

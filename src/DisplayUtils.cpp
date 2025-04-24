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

// 显示进度消息
void displayProgressMessage(const String& task, int progress) {
  // 清除屏幕
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  
  // 显示任务名称
  display.setCursor(0, 0);
  display.print("任务: ");
  display.println(task);
  
  // 显示进度百分比
  display.setCursor(0, 15);
  display.print("进度: ");
  display.print(progress);
  display.println("%");
  
  // 绘制进度条
  display.drawRect(0, 30, 128, 10, WHITE);
  display.fillRect(2, 32, 124 * progress / 100, 6, WHITE);
  
  // 更新数据到串口
  Serial.print("进度: ");
  Serial.print(task);
  Serial.print(" ");
  Serial.print(progress);
  Serial.println("%");
  
  // 更新显示
  display.display();
}

// 显示带有点数信息的进度消息
void displayProgressWithPoints(const String& task, int progress, int currentPoints, int totalPoints) {
  // 清除屏幕
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  
  // 显示任务名称
  display.setCursor(0, 0);
  display.print("任务: ");
  display.println(task);
  
  // 显示进度百分比
  display.setCursor(0, 10);
  display.print("进度: ");
  display.print(progress);
  display.println("%");
  
  // 显示点数信息
  display.setCursor(0, 20);
  display.print("点数: ");
  display.print(currentPoints);
  display.print("/");
  display.println(totalPoints);
  
  // 绘制进度条
  display.drawRect(0, 30, 128, 10, WHITE);
  display.fillRect(2, 32, 124 * progress / 100, 6, WHITE);
  
  // 更新数据到串口
  Serial.print("进度: ");
  Serial.print(task);
  Serial.print(" ");
  Serial.print(progress);
  Serial.print("% (点数: ");
  Serial.print(currentPoints);
  Serial.print("/");
  Serial.print(totalPoints);
  Serial.println(")");
  
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
  display.println("错误信息:");
  
  // 显示错误内容
  display.setCursor(0, 15);
  display.println(error);
  
  // 显示提示
  display.setCursor(0, 45);
  display.println("按下手柄按键重试");
  
  // 更新到串口
  Serial.print("错误: ");
  Serial.println(error);
  
  // 更新显示
  display.display();
}

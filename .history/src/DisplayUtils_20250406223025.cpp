#include "DisplayUtils.h"
#include "PS2Controller.h"  // 添加PS2控制器头文件

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
    display.print("Temp: ");
    display.print(ambientTemp);
    display.print(" C");
  }

  // 显示学号和姓名
  display.setCursor(0, 10);
  display.print("ID: ");
  display.print(studentID);
  display.setCursor(0, 20);
  display.print("Name: ");
  display.print(studentName);

  // 显示PS2控制器状态
  display.setCursor(0, 30);
  display.print("PS2: ");
  display.print(getCurrentControlMode() == MODE_JOINT ? "Joint " : "Grip ");
  if (getCurrentControlMode() == MODE_JOINT) {
    display.print(getCurrentJointIndex());
  }

  // 显示舵机状态
  display.setCursor(0, 40);
  display.print("J0-J2: ");
  for(int i=0; i<3; i++){
    display.print(armStatus.joints[i]);
    display.print(" ");
  }
  
  display.setCursor(0, 50);
  display.print("J3-J6: ");
  for(int i=3; i<7; i++){
    display.print(armStatus.joints[i]);
    display.print(" ");
  }

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

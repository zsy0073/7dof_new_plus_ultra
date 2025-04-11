# 7DOF 机械臂控制系统

## 项目简介
这是一个基于PlatformIO开发的7自由度机械臂控制系统。该项目实现了机械臂的精确定位、轨迹规划和动作控制功能，适用于教育、研究和工业应用场景。

## 功能特点
- 7个自由度的精确控制，支持关节空间和笛卡尔空间控制
- 多种运动模式支持（点位控制、连续轨迹、示教回放）
- 实时位置反馈和碰撞检测
- 可自定义动作序列和宏命令
- 与上位机通信接口（串口/USB/蓝牙）
- 内置逆运动学算法，支持末端执行器姿态控制
- 可视化调试工具

## 硬件要求
- 主控制器: ESP32-S3（建议使用WROOM-1-N16R8或更高配置）
- 舵机/电机: 7 × MG996R 舵机（关节1-3）+ 4 × DS3218MG 数字舵机（关节4-7）
- 传感器: MPU6050 IMU传感器、AS5600磁编码器（可选）
- 电源要求: 7.4V-12V DC，最小3A电流输出

## 软件环境
- PlatformIO Core 6.1.5+
- Arduino框架 2.0.5+
- 依赖库:
  - ESP32Servo (2.0.0+)
  - AsyncTCP (1.1.1+)
  - ESPAsyncWebServer (1.2.3+)
  - ArduinoJson (6.19.4+)
  - FastLED (3.5.0+)

## 安装与编译
1. 克隆此仓库到本地
   ```bash
   git clone https://github.com/yourusername/7dof_new_plus_ultra.git
   cd 7dof_new_plus_ultra
   ```
2. 使用PlatformIO打开项目
   ```bash
   platformio open
   ```
   或使用VS Code加载项目文件夹
3. 安装所需依赖库
   ```bash
   platformio lib install
   ```
4. 编译并上传到硬件设备
   ```bash
   platformio run --target upload
   ```

## 使用方法
1. 连接硬件并上电
2. 打开串口监视器（波特率115200）
3. 发送指令控制机械臂：
   - `HOME` - 回到初始位置
   - `MOVE J1 90` - 移动关节1到90度位置
   - `MOVETO X Y Z` - 移动末端执行器到指定坐标
   - `SAVE POS1` - 保存当前位置
   - `PLAY SEQ1` - 播放预设序列1

详细命令请参考[命令手册](docs/commands.md)。

## 项目结构
```
7dof_new_plus_ultra/
├── src/                # 源代码
│   ├── main.cpp        # 主程序入口
│   ├── kinematics.cpp  # 运动学计算
│   ├── servo_ctrl.cpp  # 舵机控制
│   └── communication.cpp # 通信模块
├── include/            # 头文件
│   ├── kinematics.h
│   ├── servo_ctrl.h
│   └── communication.h
├── lib/                # 项目依赖库
├── platformio.ini      # PlatformIO配置文件
├── docs/               # 文档
│   ├── commands.md     # 命令参考
│   ├── hardware.md     # 硬件连接图
│   └── api.md          # API参考
└── examples/           # 示例代码
    ├── basic_movement/
    └── trajectory/
```

## 贡献指南
我们欢迎各种形式的贡献，包括但不限于：
- 报告bugs和提出功能请求
- 提交代码改进或新功能
- 改进文档或添加教程

请遵循以下步骤：
1. Fork本仓库
2. 创建特性分支 (`git checkout -b feature/amazing-feature`)
3. 提交更改 (`git commit -m 'Add some amazing feature'`)
4. 推送到分支 (`git push origin feature/amazing-feature`)
5. 打开Pull Request

## 许可证
本项目采用MIT许可证 - 详情请查看 [LICENSE](LICENSE) 文件

## 联系方式
项目维护者: Your Name - your.email@example.com
项目链接: [https://github.com/yourusername/7dof_new_plus_ultra](https://github.com/yourusername/7dof_new_plus_ultra)

## 更新日志
- 2025-04-10: v1.0.0 初始版本发布
  - 实现基础舵机控制
  - 完成关节空间运动规划
  - 添加基础串口命令接口
- 2025-04-01: v0.9.0 测试版
  - 添加Web控制界面
  - 优化轨迹平滑算法

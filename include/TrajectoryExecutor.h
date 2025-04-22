#ifndef TRAJECTORY_EXECUTOR_H
#define TRAJECTORY_EXECUTOR_H

#include <Eigen.h>
#include "TrajectoryPlanner.h"
#include "RobotKinematics.h"
#include "ServoControl.h"

using namespace Eigen;

// 轨迹命令类型
enum class TrajectoryCommandType {
    JOINT_SPACE,    // 关节空间运动
    LINE,           // 笛卡尔直线
    ARC,            // 笛卡尔圆弧
    PICK_PLACE      // 抓取放置
};

// 轨迹命令结构
struct TrajectoryCommand {
    TrajectoryCommandType type;
    VectorXd jointAngles;     // 用于关节空间运动
    Vector3d position;        // 用于笛卡尔运动的目标位置
    Vector3d orientation;     // 用于笛卡尔运动的目标姿态(RPY角)
    Vector3d viaPoint;        // 用于圆弧轨迹的中间点
    double liftHeight;        // 用于Pick&Place的提升高度
    double duration;          // 运动时间
};

class TrajectoryExecutor {
public:
    TrajectoryExecutor(TrajectoryPlanner& planner, RobotKinematics& kinematics);
    
    // 解析串口命令
    bool parseCommand(const String& cmdStr, TrajectoryCommand& cmd);
    
    // 执行轨迹命令
    bool executeCommand(const TrajectoryCommand& cmd);
    
    // 检查轨迹是否完成
    bool isTrajectoryFinished() const;
    
    // 获取当前执行进度(0-1)
    float getProgress() const;
    
    // 更新轨迹执行
    void update();
    
private:
    TrajectoryPlanner& planner_;
    RobotKinematics& kinematics_;
    
    MatrixXd trajectory_;         // 当前轨迹
    VectorXd timePoints_;         // 轨迹时间点
    int currentPoint_ = 0;        // 当前执行到的轨迹点
    unsigned long startTime_ = 0;  // 轨迹开始时间
    bool isExecuting_ = false;    // 是否正在执行轨迹
    
    // 发送关节角度到舵机
    void sendToServos(const VectorXd& angles);
};

#endif // TRAJECTORY_EXECUTOR_H
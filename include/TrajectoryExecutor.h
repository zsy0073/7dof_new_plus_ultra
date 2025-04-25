#ifndef TRAJECTORY_EXECUTOR_H
#define TRAJECTORY_EXECUTOR_H

#include <Eigen.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include "TrajectoryPlanner.h"
#include "RobotKinematics.h"
#include "ServoControl.h"

using namespace Eigen;

// 轨迹点数据结构
struct TrajectoryPoint {
    float jointAngles[ARM_DOF];  // 关节角度数组(度)
    unsigned long timestamp;     // 时间戳
};

// 全局轨迹存储
#define MAX_TRAJECTORY_POINTS 1000
extern TrajectoryPoint g_recordedPoints[MAX_TRAJECTORY_POINTS];
extern int g_recordedPointCount;
extern SemaphoreHandle_t g_pointsMutex;

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
    
    bool parseCommand(const String& cmdStr, TrajectoryCommand& cmd);
    bool executeCommand(const TrajectoryCommand& cmd);
    void update();
    bool isTrajectoryFinished() const;
    float getProgress() const;
    
    // 恢复声明，但使用空实现保持兼容性
    void setCalculationQueues(QueueHandle_t commandQueue, QueueHandle_t resultQueue);
    bool setTrajectory(const MatrixXd& trajectory, const VectorXd& timePoints);
    
    VectorXd getCurrentJointAngles() const;
    
    // 获取轨迹规划器和运动学引用，便于计算任务使用
    TrajectoryPlanner& getPlanner() { return planner_; }
    RobotKinematics& getKinematics() { return kinematics_; }
    
    // 添加发送到舵机的方法声明
    void sendToServos(const VectorXd& angles);
    
    // 移动到安全的无奇异位置
    bool moveToSafePose();
    
    // 添加执行示例轨迹的方法
    bool executeExampleTrajectory();
    
    // 添加获取当前轨迹点的方法
    int getCurrentPoint() const { return currentPoint_; }
    
    // 添加检查是否正在执行轨迹的方法
    bool isExecuting() const { return isExecuting_; }
    
    // 记录轨迹点数据到全局存储 - 新方法
    void recordTrajectoryPoint(const VectorXd& angles);
    
    // 重置记录的轨迹点
    static void resetRecordedPoints();
    
    // 将记录的轨迹点以矩阵形式输出到串口
    static void outputTrajectoryMatrix();
    
    // 直接执行所有轨迹点，不依赖update方法
    bool executeAllTrajectoryPoints();
    
private:
    TrajectoryPlanner& planner_;
    RobotKinematics& kinematics_;
    
    MatrixXd trajectory_;
    VectorXd timePoints_;
    int currentPoint_ = 0;
    unsigned long startTime_ = 0;
    bool isExecuting_ = false;
    
    // 移除不再需要的队列引用
    // QueueHandle_t calcCommandQueue_ = nullptr;
    // QueueHandle_t calcResultQueue_ = nullptr;
    SemaphoreHandle_t trajectoryMutex_ = nullptr;
    
    // 添加当前关节角度存储
    VectorXd current_angles_ = VectorXd::Zero(ARM_DOF);
};

#endif // TRAJECTORY_EXECUTOR_H
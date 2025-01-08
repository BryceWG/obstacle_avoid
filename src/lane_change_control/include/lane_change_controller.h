#ifndef LANE_CHANGE_CONTROLLER_H
#define LANE_CHANGE_CONTROLLER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

class LaneChangeController {
public:
    LaneChangeController();
    ~LaneChangeController();

    // 启动变道动作
    void startLaneChange();
    
    // 运行控制循环
    void run();

private:
    // ROS节点句柄
    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;
    
    // 运动控制参数
    double linear_speed_;        // 基础线速度
    double max_angular_speed_;   // 最大角速度
    double lane_width_;          // 车道宽度
    
    // 变道状态
    bool changing_lane_;         // 变道状态标志
    ros::Time start_time_;      // 变道开始时间
    double total_time_;         // 变道总时长
    
    // 变道阶段时间参数（占总时长的比例）
    static constexpr double STAGE1_RATIO = 0.3;  // 转出阶段
    static constexpr double STAGE2_RATIO = 0.4;  // 直行阶段
    static constexpr double STAGE3_RATIO = 0.3;  // 回正阶段
    
    // 辅助函数
    void publishMotionCommand(double linear_x, double angular_z);
    double calculateAngularSpeed(double elapsed_time);
};

#endif // LANE_CHANGE_CONTROLLER_H 
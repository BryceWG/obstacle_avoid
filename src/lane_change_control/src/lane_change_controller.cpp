#include "../include/lane_change_controller.h"
#include <cmath>

LaneChangeController::LaneChangeController() : changing_lane_(false) {
    // 从参数服务器获取参数
    nh_.param("linear_speed", linear_speed_, 0.2);
    nh_.param("max_angular_speed", max_angular_speed_, 0.3);
    nh_.param("lane_width", lane_width_, 0.6);     // 默认车道宽度0.6米
    nh_.param("total_time", total_time_, 5.0);     // 默认变道总时长5秒
    
    // 发布速度命令
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    ROS_INFO("Lane Change Controller initialized");
    ROS_INFO("Parameters: linear_speed=%.2f, max_angular_speed=%.2f, lane_width=%.2f, total_time=%.2f",
             linear_speed_, max_angular_speed_, lane_width_, total_time_);
}

LaneChangeController::~LaneChangeController() {
    // 停止机器人
    publishMotionCommand(0, 0);
}

void LaneChangeController::startLaneChange() {
    if (!changing_lane_) {
        changing_lane_ = true;
        start_time_ = ros::Time::now();
        ROS_INFO("Starting lane change maneuver");
    }
}

void LaneChangeController::run() {
    ros::Rate rate(50);  // 50Hz控制频率
    
    while (ros::ok()) {
        if (changing_lane_) {
            double elapsed_time = (ros::Time::now() - start_time_).toSec();
            
            if (elapsed_time >= total_time_) {
                // 变道完成
                changing_lane_ = false;
                publishMotionCommand(linear_speed_, 0);
                ROS_INFO("Lane change completed");
            } else {
                // 计算当前应该的角速度
                double angular_speed = calculateAngularSpeed(elapsed_time);
                // 发布速度命令
                publishMotionCommand(linear_speed_, angular_speed);
            }
        } else {
            // 非变道状态，保持直行
            publishMotionCommand(linear_speed_, 0);
        }
        
        rate.sleep();
        ros::spinOnce();
    }
}

double LaneChangeController::calculateAngularSpeed(double elapsed_time) {
    double normalized_time = elapsed_time / total_time_;
    double angular_speed = 0.0;
    
    // 使用正弦函数生成平滑的角速度曲线
    if (normalized_time < STAGE1_RATIO) {
        // 第一阶段：向左转出
        double phase = normalized_time / STAGE1_RATIO * M_PI;
        angular_speed = max_angular_speed_ * sin(phase);
    }
    else if (normalized_time < STAGE1_RATIO + STAGE2_RATIO) {
        // 第二阶段：保持直行
        angular_speed = 0.0;
    }
    else {
        // 第三阶段：向右回正
        double phase = (normalized_time - STAGE1_RATIO - STAGE2_RATIO) / STAGE3_RATIO * M_PI;
        angular_speed = -max_angular_speed_ * sin(phase);
    }
    
    return angular_speed;
}

void LaneChangeController::publishMotionCommand(double linear_x, double angular_z) {
    geometry_msgs::Twist cmd;
    cmd.linear.x = linear_x;
    cmd.angular.z = angular_z;
    cmd_vel_pub_.publish(cmd);
} 
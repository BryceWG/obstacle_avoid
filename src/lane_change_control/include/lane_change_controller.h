#ifndef LANE_CHANGE_CONTROLLER_H
#define LANE_CHANGE_CONTROLLER_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class LaneChangeController {
public:
    LaneChangeController();
    ~LaneChangeController();

private:
    // ROS节点句柄
    ros::NodeHandle nh_;
    
    // 订阅者和发布者
    ros::Subscriber depth_sub_;
    ros::Publisher cmd_vel_pub_;
    
    // 参数
    double obstacle_threshold_;          // 障碍物检测阈值（毫米）
    double linear_speed_;                // 线速度
    double angular_speed_;               // 角速度
    bool changing_lane_;                 // 变道状态标志
    ros::Time start_time_;              // 变道开始时间
    
    // 障碍物检测相关
    static const int GRID_SIZE = 5;      // 5x5网格
    static const int DETECTION_THRESHOLD = 3;  // 连续检测阈值
    int detection_count_;                // 连续检测计数
    bool last_detection_;               // 上次检测结果
    
    // 回调函数
    void depthCallback(const sensor_msgs::ImageConstPtr& depth_msg);
    
    // 辅助函数
    bool detectObstacle(const cv::Mat& depth_image);
    void publishMotionCommand();
};

#endif // LANE_CHANGE_CONTROLLER_H 
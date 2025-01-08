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
    ros::Subscriber image_sub_;
    ros::Publisher cmd_vel_pub_;
    
    // 参数
    double obstacle_threshold_;          // 障碍物检测阈值（像素面积）
    double linear_speed_;                // 线速度
    double angular_speed_;               // 角速度
    bool changing_lane_;                 // 变道状态标志
    ros::Time start_time_;              // 变道开始时间
    
    // 障碍物检测相关
    static const int DETECTION_THRESHOLD = 3;  // 连续检测阈值
    int detection_count_;                // 连续检测计数
    bool last_detection_;               // 上次检测结果
    
    // 图像处理参数
    cv::Scalar obstacle_color_low_;     // 障碍物颜色范围下限
    cv::Scalar obstacle_color_high_;    // 障碍物颜色范围上限
    int min_obstacle_area_;             // 最小障碍物面积
    
    // 可视化窗口名称
    static const std::string WINDOW_ORIGINAL;
    static const std::string WINDOW_PROCESSED;
    static const std::string WINDOW_DEBUG;
    bool show_debug_windows_;           // 是否显示调试窗口
    
    // 回调函数
    void imageCallback(const sensor_msgs::ImageConstPtr& img_msg);
    
    // 辅助函数
    bool detectObstacle(const cv::Mat& rgb_image, std::vector<cv::Rect>& detected_obstacles);
    void publishMotionCommand();
    cv::Mat preprocessImage(const cv::Mat& input);
    void showDebugInfo(const cv::Mat& original, const cv::Mat& processed, 
                      const std::vector<cv::Rect>& obstacles);
};

#endif // LANE_CHANGE_CONTROLLER_H 
#ifndef LANE_CHANGE_CONTROLLER_H
#define LANE_CHANGE_CONTROLLER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Twist.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

class LaneChangeController {
public:
    LaneChangeController();
    ~LaneChangeController();

private:
    // ROS节点句柄
    ros::NodeHandle nh_;
    
    // 订阅者和发布者
    ros::Subscriber point_cloud_sub_;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher debug_pub_;    // 用于发布调试可视化信息
    
    // TF监听器
    tf::TransformListener tf_listener_;
    
    // 参数
    double obstacle_distance_threshold_;  // 障碍物检测阈值
    double safe_distance_;               // 安全距离
    double linear_speed_;                // 线速度
    double angular_speed_;               // 角速度
    bool changing_lane_;                 // 变道状态标志
    ros::Time start_time_;              // 变道开始时间
    
    // 回调函数
    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);
    
    // 辅助函数
    float detectObstacle(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    void publishMotionCommand();
    void publishDebugMarker(bool obstacle_detected);
};

#endif // LANE_CHANGE_CONTROLLER_H 
#include "../include/lane_change_controller.h"
#include <pcl_conversions/pcl_conversions.h>

LaneChangeController::LaneChangeController() : changing_lane_(false) {
    // 从参数服务器获取参数，如果没有设置则使用默认值
    nh_.param("obstacle_distance_threshold", obstacle_distance_threshold_, 1.0);
    nh_.param("safe_distance", safe_distance_, 0.5);
    nh_.param("linear_speed", linear_speed_, 0.2);
    nh_.param("angular_speed", angular_speed_, 0.2);

    // 订阅Kinect点云数据
    point_cloud_sub_ = nh_.subscribe("/camera/depth/points", 1, 
        &LaneChangeController::pointCloudCallback, this);
    
    // 发布速度命令
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
}

LaneChangeController::~LaneChangeController() {
    // 停止机器人
    geometry_msgs::Twist stop_cmd;
    cmd_vel_pub_.publish(stop_cmd);
}

void LaneChangeController::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
    // 将ROS消息转换为PCL点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // 检测前方障碍物
    if (detectObstacle(cloud)) {
        if (!changing_lane_) {
            ROS_INFO("检测到障碍物，开始变道");
            performLaneChange();
        }
    } else if (changing_lane_) {
        ROS_INFO("未检测到障碍物，恢复正常行驶");
        resumeNormalDriving();
    }
}

bool LaneChangeController::detectObstacle(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    // 检查前方区域是否有障碍物
    for (const auto& point : cloud->points) {
        // 只检查前方一定范围内的点
        if (point.x > 0 && point.x < obstacle_distance_threshold_ &&
            std::abs(point.y) < 0.3 && // 考虑车道宽度
            std::abs(point.z) < 1.0) {  // 考虑高度范围
            return true;
        }
    }
    return false;
}

void LaneChangeController::performLaneChange() {
    changing_lane_ = true;
    geometry_msgs::Twist cmd;
    
    // 设置向左变道的速度命令
    cmd.linear.x = linear_speed_;
    cmd.angular.z = angular_speed_;  // 正值表示向左转
    
    cmd_vel_pub_.publish(cmd);
}

void LaneChangeController::resumeNormalDriving() {
    changing_lane_ = false;
    geometry_msgs::Twist cmd;
    
    // 恢复直行
    cmd.linear.x = linear_speed_;
    cmd.angular.z = 0.0;
    
    cmd_vel_pub_.publish(cmd);
} 
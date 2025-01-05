#include "../include/lane_change_controller.h"
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>

LaneChangeController::LaneChangeController() : changing_lane_(false) {
    // 从参数服务器获取参数，如果没有设置则使用默认值
    nh_.param("obstacle_distance_threshold", obstacle_distance_threshold_, 1.0);
    nh_.param("safe_distance", safe_distance_, 0.5);
    nh_.param("linear_speed", linear_speed_, 0.5);  // 增加默认速度
    nh_.param("angular_speed", angular_speed_, 0.2);

    // 订阅Kinect点云数据
    point_cloud_sub_ = nh_.subscribe("/camera/depth/points", 1, 
        &LaneChangeController::pointCloudCallback, this);
    
    // 发布速度命令
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    // 发布调试信息
    debug_pub_ = nh_.advertise<visualization_msgs::Marker>("/lane_change_debug", 1);

    // 初始化起始时间
    start_time_ = ros::Time::now();
}

LaneChangeController::~LaneChangeController() {
    // 停止机器人
    geometry_msgs::Twist stop_cmd;
    cmd_vel_pub_.publish(stop_cmd);
}

void LaneChangeController::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
    static ros::Time last_detection_time = ros::Time::now();
    
    // 将ROS消息转换为PCL点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // 检测前方障碍物
    bool obstacle_detected = detectObstacle(cloud);
    
    // 每0.5秒输出一次检测状态
    if ((ros::Time::now() - last_detection_time).toSec() > 0.5) {
        ROS_INFO("Detection Status: %s", 
                obstacle_detected ? "OBSTACLE DETECTED" : "NO OBSTACLE");
        last_detection_time = ros::Time::now();
    }

    if (obstacle_detected) {
        if (!changing_lane_) {
            ROS_INFO("=== Starting Lane Change Maneuver ===");
            publishDebugMarker(true);
            start_time_ = ros::Time::now();
            changing_lane_ = true;
        }
    }

    // 发布运动命令
    publishMotionCommand();
}

bool LaneChangeController::detectObstacle(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    // 添加点云大小的调试信息
    ROS_INFO("Point cloud size: %lu points", cloud->points.size());
    
    // 记录最近的障碍物点
    float min_x = obstacle_distance_threshold_;
    bool found = false;
    
    // 检查前方区域是否有障碍物
    for (const auto& point : cloud->points) {
        // 过滤无效点
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
            continue;
        }
        
        // 只检查前方一定范围内的点
        if (point.x > 0 && point.x < obstacle_distance_threshold_ &&
            std::abs(point.y) < 0.3 && // 考虑车道宽度
            std::abs(point.z) < 1.0) {  // 考虑高度范围
            
            if (point.x < min_x) {
                min_x = point.x;
                found = true;
            }
        }
    }
    
    if (found) {
        ROS_INFO("Nearest obstacle detected at: x=%.2f m (threshold: %.2f m)", min_x, obstacle_distance_threshold_);
        return true;
    }
    
    return false;
}

void LaneChangeController::publishMotionCommand() {
    geometry_msgs::Twist cmd;
    static ros::Time last_log_time = ros::Time::now();
    
    if (!changing_lane_) {
        // 正常直行
        cmd.linear.x = linear_speed_;
        cmd.angular.z = 0.0;
        
        // 每1秒输出一次日志
        if ((ros::Time::now() - last_log_time).toSec() > 1.0) {
            ROS_INFO("Motion Status: NORMAL DRIVING");
            ROS_INFO("Command: forward=%.2f m/s", cmd.linear.x);
            last_log_time = ros::Time::now();
        }
    } else {
        // 计算变道过程中的时间
        double elapsed_time = (ros::Time::now() - start_time_).toSec();
        
        if (elapsed_time < 3.0) {  // 变道过程持续3秒
            // 使用差速驱动实现变道
            double phase = elapsed_time / 3.0;  // 0到1的变化
            
            // 使用正弦函数使转向更平滑
            double turn_rate = sin(phase * M_PI) * angular_speed_;
            
            cmd.linear.x = linear_speed_;
            cmd.angular.z = turn_rate;  // 正值表示向左转
            
            // 每0.5秒输出一次日志
            if ((ros::Time::now() - last_log_time).toSec() > 0.5) {
                ROS_INFO("Motion Status: LANE CHANGING (%.0f%%)", phase * 100);
                ROS_INFO("Command: forward=%.2f m/s, turn_rate=%.2f rad/s", 
                        cmd.linear.x, cmd.angular.z);
                last_log_time = ros::Time::now();
            }
        } else {
            // 变道完成，恢复直行
            changing_lane_ = false;
            cmd.linear.x = linear_speed_;
            cmd.angular.z = 0.0;
            ROS_INFO("Motion Status: LANE CHANGE COMPLETED");
            ROS_INFO("Command: forward=%.2f m/s, turn_rate=%.2f rad/s", 
                    cmd.linear.x, cmd.angular.z);
            publishDebugMarker(false);
        }
    }
    
    // 确保其他速度分量为0
    cmd.linear.y = 0.0;  // 不再使用横向速度
    cmd.linear.z = 0.0;
    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    
    cmd_vel_pub_.publish(cmd);
}

void LaneChangeController::publishDebugMarker(bool obstacle_detected) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time::now();
    marker.ns = "lane_change_debug";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;

    // 在前方显示一个球体标记
    marker.pose.position.x = obstacle_distance_threshold_ / 2;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0.5;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    // 根据是否检测到障碍物改变颜色
    marker.color.a = 1.0;
    if (obstacle_detected) {
        marker.color.r = 1.0;  // 红色表示检测到障碍物
        marker.color.g = 0.0;
        marker.color.b = 0.0;
    } else {
        marker.color.r = 0.0;
        marker.color.g = 1.0;  // 绿色表示安全
        marker.color.b = 0.0;
    }

    debug_pub_.publish(marker);
} 
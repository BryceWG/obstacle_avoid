#include "../include/lane_change_controller.h"
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>

LaneChangeController::LaneChangeController() : changing_lane_(false) {
    // 从参数服务器获取参数，如果没有设置则使用默认值
    nh_.param("obstacle_distance_threshold", obstacle_distance_threshold_, 1.0);
    nh_.param("safe_distance", safe_distance_, 0.5);
    nh_.param("linear_speed", linear_speed_, 0.2);  // 降低默认速度
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
    
    ROS_INFO("Lane Change Controller initialized");
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
    float obstacle_distance = detectObstacle(cloud);
    
    // 根据障碍物距离决定行为
    if (obstacle_distance > 0 && obstacle_distance < obstacle_distance_threshold_) {
        if (!changing_lane_) {
            ROS_INFO("Obstacle detected at %.2f meters, starting lane change", obstacle_distance);
            start_time_ = ros::Time::now();
            changing_lane_ = true;
        }
    }

    // 发布运动命令
    publishMotionCommand();
}

float LaneChangeController::detectObstacle(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    std::vector<float> valid_distances;
    int total_points = 0;
    int valid_points = 0;
    
    // 用于统计点的分布
    struct PointStats {
        int count = 0;
        float min_dist = std::numeric_limits<float>::max();
        float max_dist = -std::numeric_limits<float>::max();
    };
    
    // 划分不同区域统计点的分布
    PointStats center_stats;   // 中心区域
    PointStats left_stats;     // 左侧区域
    PointStats right_stats;    // 右侧区域
    
    // 定义检测区域（相机坐标系）
    float min_x = -2.0;    // 对应全局-Y方向的范围
    float max_x = 2.0;     // 对应全局-Y方向的范围
    float min_y = -2.0;    // 对应全局Z方向的范围（高度）
    float max_y = 2.0;     // 对应全局Z方向的范围（高度）
    float min_z = 0.1;     // 对应全局X方向的最小距离
    float max_z = 5.0;     // 对应全局X方向的最大距离
    
    // 遍历点云
    for (const auto& point : cloud->points) {
        total_points++;
        
        // 基本有效性检查
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
            continue;
        }

        // 统计不同区域的点
        if (std::abs(point.x) < 0.3) {  // 中心区域
            center_stats.count++;
            center_stats.min_dist = std::min(center_stats.min_dist, point.z);
            center_stats.max_dist = std::max(center_stats.max_dist, point.z);
        } else if (point.x < -0.3) {    // 左侧区域
            left_stats.count++;
            left_stats.min_dist = std::min(left_stats.min_dist, point.z);
            left_stats.max_dist = std::max(left_stats.max_dist, point.z);
        } else {                         // 右侧区域
            right_stats.count++;
            right_stats.min_dist = std::min(right_stats.min_dist, point.z);
            right_stats.max_dist = std::max(right_stats.max_dist, point.z);
        }

        // 检查点是否在感兴趣区域内
        if (point.x > min_x && point.x < max_x &&     // 左右范围检查
            point.y > min_y && point.y < max_y &&     // 高度范围检查
            point.z > min_z && point.z < max_z) {     // 前向距离检查
            valid_points++;
            valid_distances.push_back(point.z);
        }
    }
    
    // 输出详细的点云分布信息
    ROS_INFO("Point Distribution Analysis:");
    ROS_INFO("Center region: %d points, distance range: %.2f to %.2f m",
             center_stats.count,
             center_stats.min_dist != std::numeric_limits<float>::max() ? center_stats.min_dist : 0,
             center_stats.max_dist != -std::numeric_limits<float>::max() ? center_stats.max_dist : 0);
    ROS_INFO("Left region: %d points, distance range: %.2f to %.2f m",
             left_stats.count,
             left_stats.min_dist != std::numeric_limits<float>::max() ? left_stats.min_dist : 0,
             left_stats.max_dist != -std::numeric_limits<float>::max() ? left_stats.max_dist : 0);
    ROS_INFO("Right region: %d points, distance range: %.2f to %.2f m",
             right_stats.count,
             right_stats.min_dist != std::numeric_limits<float>::max() ? right_stats.min_dist : 0,
             right_stats.max_dist != -std::numeric_limits<float>::max() ? right_stats.max_dist : 0);
    
    // 输出总体统计信息
    ROS_INFO("Point cloud stats - Total: %d, Valid: %d", total_points, valid_points);
    
    // 如果没有有效点，返回-1表示无障碍物
    if (valid_distances.empty()) {
        ROS_INFO("No valid obstacles detected");
        return -1;
    }
    
    // 计算最近的障碍物距离
    float min_distance = *std::min_element(valid_distances.begin(), valid_distances.end());
    ROS_INFO("Nearest obstacle at %.2f meters", min_distance);
    
    return min_distance;
}

void LaneChangeController::publishMotionCommand() {
    geometry_msgs::Twist cmd;
    
    if (!changing_lane_) {
        // 正常直行
        cmd.linear.x = linear_speed_;
        cmd.angular.z = 0.0;
    } else {
        // 计算变道过程中的时间
        double elapsed_time = (ros::Time::now() - start_time_).toSec();
        
        if (elapsed_time < 3.0) {  // 变道过程持续3秒
            // 使用正弦函数使转向更平滑
            double phase = elapsed_time / 3.0;  // 0到1的变化
            double turn_rate = sin(phase * M_PI) * angular_speed_;
            
            // 在转向时适当降低前进速度
            cmd.linear.x = linear_speed_ * 0.8;  // 降低到80%的速度
            cmd.angular.z = turn_rate;
            
            ROS_INFO("Lane change: %.0f%%, speed=%.2f m/s, turn=%.2f rad/s", 
                    phase * 100, cmd.linear.x, cmd.angular.z);
        } else {
            // 变道完成
            changing_lane_ = false;
            cmd.linear.x = linear_speed_;
            cmd.angular.z = 0.0;
            ROS_INFO("Lane change completed");
        }
    }
    
    // 确保其他速度分量为0
    cmd.linear.y = 0.0;
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
#include "../include/lane_change_controller.h"
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <tf/transform_datatypes.h>

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

    // 发布调试信息
    debug_pub_ = nh_.advertise<visualization_msgs::Marker>("/lane_change_debug", 1);

    // 初始化Gazebo模型状态客户端
    model_state_client_ = nh_.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    get_state_client_ = nh_.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

    // 初始化起始时间
    start_time_ = ros::Time::now();
    current_yaw_ = 0.0;  // 初始航向角
    
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
    int filtered_ground = 0;
    int valid_points = 0;
    
    // 用于统计点的分布
    struct PointStats {
        int count = 0;
        float min_dist = std::numeric_limits<float>::max();
        float max_dist = -std::numeric_limits<float>::max();
        std::map<float, int> distance_histogram;  // 用于统计距离分布
        std::map<float, int> height_histogram;    // 用于统计高度分布
        float min_height = std::numeric_limits<float>::max();
        float max_height = -std::numeric_limits<float>::max();
    };
    
    // 划分不同区域统计点的分布
    PointStats center_stats;   // 中心区域
    PointStats left_stats;     // 左侧区域
    PointStats right_stats;    // 右侧区域
    
    // 定义检测区域（相机坐标系）
    float min_x = -2.0;    // 对应全局-Y方向的范围
    float max_x = 2.0;     // 对应全局-Y方向的范围
    float min_y = -0.1;    // 对应全局Z方向的范围（高度），提高最小高度以过滤地面
    float max_y = 2.0;     // 对应全局Z方向的范围（高度）
    float min_z = 0.1;     // 对应全局X方向的最小距离
    float max_z = 5.0;     // 对应全局X方向的最大距离

    // 地面点过滤参数
    const float ground_height_threshold = -0.1;  // 低于此高度视为地面点
    const float min_points_threshold = 1000;     // 最小有效点数阈值
    
    // 遍历点云
    for (const auto& point : cloud->points) {
        total_points++;
        
        // 基本有效性检查
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
            continue;
        }

        // 坐标系转换：
        // 相机坐标系 -> 全局坐标系
        // 相机x -> 全局-y
        // 相机y -> 全局-z
        // 相机z -> 全局x
        float global_x = point.z;    // 前向距离
        float global_y = -point.x;   // 左右位置
        float global_z = -point.y;   // 高度

        // 地面点过滤（使用全局坐标系的Z轴高度）
        if (global_z < ground_height_threshold) {
            filtered_ground++;
            continue;
        }

        // 统计不同区域的点
        PointStats* current_stats = nullptr;
        if (std::abs(global_y) < 0.3) {  // 中心区域
            current_stats = &center_stats;
        } else if (global_y < -0.3) {    // 左侧区域
            current_stats = &left_stats;
        } else {                         // 右侧区域
            current_stats = &right_stats;
        }
        
        if (current_stats) {
            current_stats->count++;
            current_stats->min_dist = std::min(current_stats->min_dist, global_x);
            current_stats->max_dist = std::max(current_stats->max_dist, global_x);
            current_stats->min_height = std::min(current_stats->min_height, global_z);
            current_stats->max_height = std::max(current_stats->max_height, global_z);
            
            // 将距离四舍五入到厘米级别
            float rounded_dist = std::round(global_x * 100) / 100;
            current_stats->distance_histogram[rounded_dist]++;
            
            // 将高度四舍五入到厘米级别
            float rounded_height = std::round(global_z * 100) / 100;
            current_stats->height_histogram[rounded_height]++;
        }

        // 检查点是否在感兴趣区域内
        if (global_y > min_x && global_y < max_x &&     // 左右范围检查
            global_z > min_y && global_z < max_y &&     // 高度范围检查
            global_x > min_z && global_x < max_z) {     // 前向距离检查
            valid_points++;
            valid_distances.push_back(global_x);
        }
    }
    
    // 输出详细的点云分布信息
    ROS_INFO("Point Distribution Analysis (Ground points filtered: %d):", filtered_ground);
    auto print_region_stats = [](const char* region_name, const PointStats& stats) {
        if (stats.count < 1) return;  // 跳过空区域
        
        ROS_INFO("%s region: %d points, distance range: %.2f to %.2f m, height range: %.2f to %.2f m", 
            region_name, stats.count, 
            (stats.min_dist != std::numeric_limits<float>::max()) ? stats.min_dist : 0,
            (stats.max_dist != -std::numeric_limits<float>::max()) ? stats.max_dist : 0,
            (stats.min_height != std::numeric_limits<float>::max()) ? stats.min_height : 0,
            (stats.max_height != -std::numeric_limits<float>::max()) ? stats.max_height : 0);
        
        // 输出距离直方图中最频繁的几个值
        std::vector<std::pair<float, int>> hist_vec(stats.distance_histogram.begin(), 
                                                   stats.distance_histogram.end());
        std::sort(hist_vec.begin(), hist_vec.end(), 
                 [](const auto& a, const auto& b) { return a.second > b.second; });
        
        ROS_INFO("Most common distances in %s region:", region_name);
        for (int i = 0; i < std::min(5, (int)hist_vec.size()); i++) {
            if (hist_vec[i].second > 100) {  // 只显示数量超过100的点
                ROS_INFO("  %.2f m: %d points", hist_vec[i].first, hist_vec[i].second);
            }
        }

        // 输出高度直方图中最频繁的几个值
        std::vector<std::pair<float, int>> height_hist_vec(stats.height_histogram.begin(), 
                                                          stats.height_histogram.end());
        std::sort(height_hist_vec.begin(), height_hist_vec.end(), 
                 [](const auto& a, const auto& b) { return a.second > b.second; });
        
        ROS_INFO("Most common heights in %s region:", region_name);
        for (int i = 0; i < std::min(5, (int)height_hist_vec.size()); i++) {
            if (height_hist_vec[i].second > 100) {  // 只显示数量超过100的点
                ROS_INFO("  %.2f m: %d points", height_hist_vec[i].first, height_hist_vec[i].second);
            }
        }
    };
    
    print_region_stats("Center", center_stats);
    print_region_stats("Left", left_stats);
    print_region_stats("Right", right_stats);
    
    // 输出总体统计信息
    ROS_INFO("Point cloud stats - Total: %d, Valid: %d", total_points, valid_points);
    
    // 如果没有足够的有效点，返回-1表示无障碍物
    if (valid_distances.size() < min_points_threshold) {
        ROS_INFO("Insufficient valid points for obstacle detection");
        return -1;
    }
    
    // 计算最近的障碍物距离
    // 使用百分位数而不是最小值，以减少噪声影响
    std::sort(valid_distances.begin(), valid_distances.end());
    int percentile_index = valid_distances.size() * 0.05;  // 使用5%分位数
    float obstacle_distance = valid_distances[percentile_index];
    ROS_INFO("Nearest obstacle at %.2f meters (5th percentile of %zu valid points)", 
             obstacle_distance, valid_distances.size());
    
    return obstacle_distance;
}

void LaneChangeController::publishMotionCommand() {
    if (!changing_lane_) {
        // 正常直行：只有前进速度
        geometry_msgs::Twist cmd;
        cmd.linear.x = linear_speed_;
        cmd.angular.z = 0.0;
        cmd_vel_pub_.publish(cmd);
    } else {
        // 计算变道过程中的时间
        double elapsed_time = (ros::Time::now() - start_time_).toSec();
        
        if (elapsed_time < 3.0) {  // 变道过程持续3秒
            // 使用正弦函数使转向更平滑
            double phase = elapsed_time / 3.0;  // 0到1的变化
            double y_offset = 0.0;  // Y方向偏移量
            
            if (elapsed_time < 1.5) {
                // 第一阶段：左转并向左移动
                current_yaw_ = 0.3 * sin(phase * M_PI);  // 最大转向角约17度
                y_offset = 0.5 * sin(phase * M_PI);  // 最大左偏0.5米
            } else {
                // 第二阶段：右转回正
                double phase2 = (elapsed_time - 1.5) / 1.5;  // 0到1的变化
                current_yaw_ = 0.3 * sin(M_PI * (1 - phase2));  // 从最大角度转回0
                y_offset = 0.5;  // 保持左偏0.5米
            }
            
            // 获取当前位置
            gazebo_msgs::GetModelState get_state;
            get_state.model_name = "simple_robot";
            get_state.reference_frame = "world";
            
            // 设置新的位姿
            gazebo_msgs::ModelState model_state;
            model_state.model_name = "simple_robot";
            model_state.pose.position.x = 3.5 + elapsed_time * linear_speed_;  // 保持前进
            model_state.pose.position.y = y_offset;  // 左右偏移
            model_state.pose.position.z = 0.1;  // 保持原有高度
            
            // 设置朝向
            tf::Quaternion q;
            q.setRPY(0, 0, current_yaw_);
            model_state.pose.orientation.x = q.x();
            model_state.pose.orientation.y = q.y();
            model_state.pose.orientation.z = q.z();
            model_state.pose.orientation.w = q.w();
            
            // 设置速度
            model_state.twist.linear.x = linear_speed_;
            model_state.twist.linear.y = 0;
            model_state.twist.linear.z = 0;
            model_state.twist.angular.x = 0;
            model_state.twist.angular.y = 0;
            model_state.twist.angular.z = 0;
            
            model_state.reference_frame = "world";
            
            // 应用新的状态
            gazebo_msgs::SetModelState srv;
            srv.request.model_state = model_state;
            
            if (model_state_client_.call(srv)) {
                if (srv.response.success) {
                    ROS_INFO("Lane change: %.0f%%, yaw=%.2f rad, offset=%.2f m", 
                            phase * 100, current_yaw_, y_offset);
                } else {
                    ROS_ERROR("Failed to set robot state: %s", srv.response.status_message.c_str());
                }
            } else {
                ROS_ERROR("Failed to call set_model_state service");
            }
        } else {
            // 变道完成，恢复直行
            changing_lane_ = false;
            geometry_msgs::Twist cmd;
            cmd.linear.x = linear_speed_;
            cmd.angular.z = 0.0;
            cmd_vel_pub_.publish(cmd);
            ROS_INFO("Lane change completed, resuming forward motion");
        }
    }
}

void LaneChangeController::setRobotPose(double yaw) {
    gazebo_msgs::SetModelState srv;
    gazebo_msgs::ModelState model_state;
    
    // 设置模型名称
    model_state.model_name = "simple_robot";
    
    // 保持当前位置不变，只改变朝向
    model_state.pose.position.x = 0.0;  // 这里可以根据需要设置位置
    model_state.pose.position.y = 0.0;
    model_state.pose.position.z = 0.0;
    
    // 设置朝向（欧拉角转四元数）
    tf::Quaternion q;
    q.setRPY(0, 0, yaw);  // Roll=0, Pitch=0, 只设置Yaw
    model_state.pose.orientation.x = q.x();
    model_state.pose.orientation.y = q.y();
    model_state.pose.orientation.z = q.z();
    model_state.pose.orientation.w = q.w();
    
    // 设置速度为0
    model_state.twist.linear.x = 0;
    model_state.twist.linear.y = 0;
    model_state.twist.linear.z = 0;
    model_state.twist.angular.x = 0;
    model_state.twist.angular.y = 0;
    model_state.twist.angular.z = 0;
    
    // 设置参考坐标系
    model_state.reference_frame = "world";
    
    srv.request.model_state = model_state;
    
    // 调用服务
    if (model_state_client_.call(srv)) {
        if (srv.response.success) {
            ROS_INFO("Successfully set robot pose, yaw = %.2f", yaw);
        } else {
            ROS_ERROR("Failed to set robot pose: %s", srv.response.status_message.c_str());
        }
    } else {
        ROS_ERROR("Failed to call set_model_state service");
    }
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
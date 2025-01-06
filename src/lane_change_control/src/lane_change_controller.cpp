#include "../include/lane_change_controller.h"

LaneChangeController::LaneChangeController() : changing_lane_(false), detection_count_(0), last_detection_(false) {
    // 从参数服务器获取参数，如果没有设置则使用默认值
    nh_.param("obstacle_threshold", obstacle_threshold_, 1000.0);  // 默认2米
    nh_.param("linear_speed", linear_speed_, 0.3);
    nh_.param("angular_speed", angular_speed_, 0.5);

    // 订阅Kinect深度图像
    depth_sub_ = nh_.subscribe("/kinect2/sd/image_depth", 1, 
        &LaneChangeController::depthCallback, this);
    
    // 发布速度命令
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    ROS_INFO("Lane Change Controller initialized");
}

LaneChangeController::~LaneChangeController() {
    // 停止机器人
    geometry_msgs::Twist stop_cmd;
    cmd_vel_pub_.publish(stop_cmd);
}

void LaneChangeController::depthCallback(const sensor_msgs::ImageConstPtr& depth_msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        // 转换深度图像
        cv_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // 检测前方障碍物
    bool current_detection = detectObstacle(cv_ptr->image);
    
    // 更新连续检测计数
    if (current_detection && last_detection_) {
        detection_count_++;
    } else if (!current_detection) {
        detection_count_ = 0;
    }
    last_detection_ = current_detection;

    // 如果连续检测到障碍物达到阈值，触发变道
    if (!changing_lane_ && detection_count_ >= DETECTION_THRESHOLD) {
        ROS_INFO("Obstacle confirmed after %d consecutive detections, starting lane change", DETECTION_THRESHOLD);
        start_time_ = ros::Time::now();
        changing_lane_ = true;
        detection_count_ = 0;  // 重置计数
    }

    // 发布运动命令
    publishMotionCommand();
}

bool LaneChangeController::detectObstacle(const cv::Mat& depth_image) {
    // 计算采样网格的步长
    int step_x = depth_image.cols / (GRID_SIZE * 2);  // 在中心区域采样
    int step_y = depth_image.rows / (GRID_SIZE * 2);
    
    // 计算起始位置（图像中心区域）
    int start_x = depth_image.cols/2 - (GRID_SIZE/2) * step_x;
    int start_y = depth_image.rows/2 - (GRID_SIZE/2) * step_y;
    
    int obstacle_count = 0;  // 检测到障碍物的点数
    
    // 在5x5网格中采样
    for(int i = 0; i < GRID_SIZE; i++) {
        for(int j = 0; j < GRID_SIZE; j++) {
            float depth = depth_image.at<float>(
                start_y + i * step_y, 
                start_x + j * step_x
            );
            
            // 检查深度值是否有效且在障碍物阈值范围内
            if(depth > 400 && depth < obstacle_threshold_) {
                obstacle_count++;
            }
        }
    }
    
    // 如果超过3个点检测到障碍物，就认为存在障碍物
    bool has_obstacle = (obstacle_count >= 3);
    if(has_obstacle) {
        ROS_INFO("Obstacle detected - %d points triggered", obstacle_count);
    }
    
    return has_obstacle;
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
        
        if (elapsed_time < 4.0) {  // 变道总时长4秒
            if (elapsed_time < 1.5) {
                // 第一阶段：左转
                cmd.linear.x = linear_speed_ * 0.8;  // 降低直行速度
                cmd.angular.z = angular_speed_;
            } else if (elapsed_time < 2.5) {
                // 第二阶段：直行
                cmd.linear.x = linear_speed_;
                cmd.angular.z = 0.0;
            } else {
                // 第三阶段：右转回正
                cmd.linear.x = linear_speed_ * 0.8;
                cmd.angular.z = -angular_speed_;
            }
        } else {
            // 变道完成
            changing_lane_ = false;
            cmd.linear.x = linear_speed_;
            cmd.angular.z = 0.0;
            ROS_INFO("Lane change completed");
        }
    }
    
    // 发布速度命令
    cmd_vel_pub_.publish(cmd);
} 
} 
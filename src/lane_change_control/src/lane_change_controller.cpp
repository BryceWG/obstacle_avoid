#include "../include/lane_change_controller.h"

LaneChangeController::LaneChangeController() : changing_lane_(false), detection_count_(0), last_detection_(false) {
    // 从参数服务器获取参数，如果没有设置则使用默认值
    nh_.param("obstacle_threshold", obstacle_threshold_, 5000.0);  // 默认像素面积阈值
    nh_.param("linear_speed", linear_speed_, 0.2);
    nh_.param("angular_speed", angular_speed_, 0.5);
    nh_.param("min_obstacle_area", min_obstacle_area_, 1000);  // 最小障碍物面积

    // 设置障碍物颜色范围（默认为红色）
    obstacle_color_low_ = cv::Scalar(0, 100, 100);   // HSV空间
    obstacle_color_high_ = cv::Scalar(10, 255, 255);

    // 订阅USB摄像头图像
    std::string camera_topic;
    nh_.param<std::string>("camera_topic", camera_topic, "/usb_cam/image_raw");
    image_sub_ = nh_.subscribe(camera_topic, 1, &LaneChangeController::imageCallback, this);
    
    // 发布速度命令
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    ROS_INFO("Lane Change Controller initialized");
}

LaneChangeController::~LaneChangeController() {
    // 停止机器人
    geometry_msgs::Twist stop_cmd;
    cmd_vel_pub_.publish(stop_cmd);
}

void LaneChangeController::imageCallback(const sensor_msgs::ImageConstPtr& img_msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if (cv_ptr->image.empty()) {
        ROS_ERROR("Empty image");
        return;
    }

    // 预处理图像
    cv::Mat processed_image = preprocessImage(cv_ptr->image);
    
    // 检测障碍物
    bool current_detection = detectObstacle(processed_image);
    
    // 更新连续检测计数
    if (current_detection) {
        detection_count_++;
        ROS_INFO("Obstacle detected, count: %d", detection_count_);
    } else {
        detection_count_ = 0;
    }
    last_detection_ = current_detection;

    // 如果连续检测到障碍物达到阈值，触发变道
    if (!changing_lane_ && detection_count_ >= DETECTION_THRESHOLD) {
        ROS_INFO("Obstacle confirmed, starting lane change");
        start_time_ = ros::Time::now();
        changing_lane_ = true;
        detection_count_ = 0;
    }

    // 发布运动命令
    publishMotionCommand();
}

cv::Mat LaneChangeController::preprocessImage(const cv::Mat& input) {
    cv::Mat hsv, mask;
    
    // 转换到HSV颜色空间
    cv::cvtColor(input, hsv, cv::COLOR_BGR2HSV);
    
    // 创建掩码
    cv::inRange(hsv, obstacle_color_low_, obstacle_color_high_, mask);
    
    // 形态学操作去噪
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
    
    return mask;
}

bool LaneChangeController::detectObstacle(const cv::Mat& rgb_image) {
    // 寻找轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(rgb_image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    // 分析轮廓
    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area > min_obstacle_area_) {
            // 计算轮廓的边界框
            cv::Rect bbox = cv::boundingRect(contour);
            
            // 检查障碍物是否在图像的中央区域
            int center_x = bbox.x + bbox.width/2;
            if (center_x > rgb_image.cols/3 && center_x < 2*rgb_image.cols/3) {
                return true;
            }
        }
    }
    
    return false;
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
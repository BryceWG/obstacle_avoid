#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <dirent.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "geometry_msgs/Twist.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "kcftracker.hpp"

static const std::string RGB_WINDOW = "RGB Image window";

#define Max_linear_speed 0.5
#define Min_linear_speed 0
#define Max_rotation_speed 0.75

// 添加面积比例阈值
#define CLOSER_THRESHOLD 1.2
#define FARTHER_THRESHOLD 0.8

class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Publisher cmd_vel_pub_;

    float linear_speed;
    float rotation_speed;
    float k_rotation_speed;
    float h_rotation_speed_left;
    float h_rotation_speed_right;

    int ERROR_OFFSET_X_left1;
    int ERROR_OFFSET_X_left2;
    int ERROR_OFFSET_X_right1;
    int ERROR_OFFSET_X_right2;

    cv::Mat rgbimage;
    cv::Rect selectRect;
    cv::Point origin;
    cv::Rect result;
    double initial_area_;  // 添加初始面积记录

    bool select_flag;
    bool bRenewROI;
    bool bBeginKCF;

    bool HOG;
    bool FIXEDWINDOW;
    bool MULTISCALE;
    bool SILENT;
    bool LAB;

    KCFTracker tracker;

    float max_rotation_speed;
    int image_center_x;      // 图像中心x坐标
    int center_threshold;    // 中心区域阈值

public:
    ImageConverter()
        : it_(nh_), tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB)
    {
        // 从参数服务器获取参数
        nh_.param("linear_speed", linear_speed, 0.2f);
        nh_.param("angular_speed", rotation_speed, 0.2f);
        
        // 订阅USB摄像头的图像话题
        std::string camera_topic;
        nh_.param<std::string>("camera_topic", camera_topic, "/usb_cam/image_raw");
        image_sub_ = it_.subscribe(camera_topic, 1, &ImageConverter::imageCb, this);
        
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

        k_rotation_speed = 0.004;
        h_rotation_speed_left = 0.95;
        h_rotation_speed_right = 0.81;

        ERROR_OFFSET_X_left1 = 50;
        ERROR_OFFSET_X_left2 = 200;
        ERROR_OFFSET_X_right1 = 240;
        ERROR_OFFSET_X_right2 = 390;

        select_flag = false;
        bRenewROI = false;
        bBeginKCF = false;
        initial_area_ = 0;

        HOG = true;
        FIXEDWINDOW = false;
        MULTISCALE = true;
        SILENT = true;
        LAB = false;

        cv::namedWindow(RGB_WINDOW);

        // 初始化转向控制参数
        max_rotation_speed = Max_rotation_speed;
        center_threshold = 30;  // 中心区域阈值，可以通过参数服务器配置
    }

    ~ImageConverter()
    {
        cv::destroyWindow(RGB_WINDOW);
    }

    double getRelativeDistance(const cv::Rect& current_rect) {
        if (initial_area_ == 0) return 0;
        
        double current_area = current_rect.area();
        double area_ratio = current_area / initial_area_;
        
        if (area_ratio > CLOSER_THRESHOLD) {
            return -1.0 * (area_ratio - 1.0);  // 负值表示更近
        } else if (area_ratio < FARTHER_THRESHOLD) {
            return 1.0 * (1.0 - area_ratio);   // 正值表示更远
        }
        return 0.0; // 距离相近
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv_ptr->image.copyTo(rgbimage);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cv::setMouseCallback(RGB_WINDOW, onMouse, this);

        if(bRenewROI)
        {
            tracker.init(selectRect, rgbimage);
            initial_area_ = selectRect.area();  // 记录初始面积
            bBeginKCF = true;
            bRenewROI = false;
        }

        if(bBeginKCF)
        {
            result = tracker.update(rgbimage);
            cv::rectangle(rgbimage, result, cv::Scalar(0, 255, 255), 1, 8);
            
            // 计算距离变化并调整速度
            double distance_change = getRelativeDistance(result);
            geometry_msgs::Twist cmd;
            
            // 基于距离变化调整速度
            if (distance_change < 0) {
                // 目标更近，减速
                cmd.linear.x = std::max(0.0, linear_speed * (1 + distance_change));
            } else if (distance_change > 0) {
                // 目标更远，加速
                cmd.linear.x = std::min((double)Max_linear_speed, linear_speed * (1 + distance_change));
            } else {
                // 保持当前速度
                cmd.linear.x = linear_speed;
            }

            // 计算目标中心点和图像中心点
            int target_center_x = result.x + result.width/2;
            image_center_x = rgbimage.cols/2;
            
            // 计算偏差
            int error = target_center_x - image_center_x;
            
            // 根据偏差计算转向速度
            if (abs(error) < center_threshold) {
                // 在中心区域内，不需要转向
                cmd.angular.z = 0;
            } else {
                // 根据偏差计算转向速度，使用比例控制
                cmd.angular.z = -k_rotation_speed * error;
                
                // 限制最大转向速度
                if (cmd.angular.z > max_rotation_speed) {
                    cmd.angular.z = max_rotation_speed;
                } else if (cmd.angular.z < -max_rotation_speed) {
                    cmd.angular.z = -max_rotation_speed;
                }
            }

            // 添加调试信息
            ROS_INFO("Target center: %d, Image center: %d, Error: %d, Angular: %.2f", 
                     target_center_x, image_center_x, error, cmd.angular.z);

            cmd_vel_pub_.publish(cmd);
        }
        else
            cv::rectangle(rgbimage, selectRect, cv::Scalar(255, 0, 0), 2, 8, 0);

        // 绘制图像中心线（用于调试）
        if (rgbimage.cols > 0) {
            cv::line(rgbimage, 
                    cv::Point(rgbimage.cols/2, 0),
                    cv::Point(rgbimage.cols/2, rgbimage.rows),
                    cv::Scalar(0, 255, 0), 1);
        }

        cv::imshow(RGB_WINDOW, rgbimage);
        cv::waitKey(1);
    }

    static void onMouse(int event, int x, int y, int, void* userdata)
    {
        ImageConverter* self = static_cast<ImageConverter*>(userdata);
        self->onMouseImpl(event, x, y);
    }

    void onMouseImpl(int event, int x, int y)
    {
        if (select_flag)
        {
            selectRect.x = MIN(origin.x, x);
            selectRect.y = MIN(origin.y, y);
            selectRect.width = abs(x - origin.x);
            selectRect.height = abs(y - origin.y);
            selectRect &= cv::Rect(0, 0, rgbimage.cols, rgbimage.rows);
        }
        if (event == CV_EVENT_LBUTTONDOWN)
        {
            bBeginKCF = false;
            select_flag = true;
            origin = cv::Point(x, y);
            selectRect = cv::Rect(x, y, 0, 0);
        }
        else if (event == CV_EVENT_LBUTTONUP)
        {
            select_flag = false;
            bRenewROI = true;
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "usb_tracker");
    ImageConverter ic;
    ros::spin();
    return 0;
}


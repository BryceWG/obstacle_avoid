#include "../include/lane_change_controller.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "lane_change_controller");
    
    LaneChangeController controller;
    
    // 等待1秒让节点完全初始化
    ros::Duration(1.0).sleep();
    
    // 启动变道
    controller.startLaneChange();
    
    // 运行控制循环
    controller.run();
    
    return 0;
} 
#include "../include/lane_change_controller.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "lane_change_controller");
    
    LaneChangeController controller;
    
    ros::spin();
    
    return 0;
} 
#include "path_tracking_controller.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_tracking_controller");
    PathTrackingController controller;
    ros::spin();
    return 0;
}
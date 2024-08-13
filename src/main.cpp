#include "factor/imuPreintegration.hpp"
#include "factor/lidarOdomtery.hpp"

#include "feature/featureExtraction.hpp"

int main(int argc, char *argv[]) {
    ROS_INFO("\033[1;32m----> LiLoc Started.\033[0m");

    ros::init(argc, argv, "LiLoc_node");
    ros::NodeHandle nh;

    ROS_INFO("\033[1;32m----> Feature Extraction.\033[0m");
    FeatureExtraction feature;

    ROS_INFO("\033[1;32m----> IMU Preintegration.\033[0m");
    IMUPreintegration imuFactor;
    

    ros::spin();

    return 0;
}
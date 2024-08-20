#include "factor/imuPreintegration.hpp"

#include "feature/featureExtraction.hpp"
#include "factor/lidarOdometry.hpp"

#include "preprocess/preprocess.hpp"

#include "database/dataLoader.hpp"

#include "initialization/initialization.hpp"

#include "optimization/gtsam.hpp"

#include "optimization/lm.hpp"

int main(int argc, char *argv[]) {
    ROS_INFO("\033[1;32m----> LiLoc Started.\033[0m");

    ros::init(argc, argv, "LiLoc_node");
    ros::NodeHandle nh;

    // ROS_INFO("\033[1;32m----> IMU Preintegration.\033[0m");
    // IMUPreintegration imuFactor;

    ROS_INFO("\033[1;32m----> Preprocess.\033[0m");
    Preprocess *preprocess(new Preprocess());

    ROS_INFO("\033[1;32m----> Initialization.\033[0m");
    Initialization *init(new Initialization());

    ROS_INFO("\033[1;32m----> Optimization.\033[0m");
    GTSAM *gtsam(new GTSAM());
    LM *lm(new LM());

    // ROS_INFO("\033[1;32m----> LiDAR Odometry.\033[0m");
    // LidarOdometry lidarFactor;

    // ROS_INFO("\033[1;32m----> Data Loader.\033[0m");
    // DataLoader loader("haihia");

    KD_TREE<PointType> *edge_tree_lio(new KD_TREE<PointType>());
    KD_TREE<PointType> *surf_tree_lio(new KD_TREE<PointType>());
    KD_TREE<PointType> *edge_tree_prior(new KD_TREE<PointType>());
    KD_TREE<PointType> *surf_tree_prior(new KD_TREE<PointType>());

    ros::Publisher pub_edge = nh.advertise<sensor_msgs::PointCloud2>("/edge_tree", 20);
    ros::Publisher pub_surf = nh.advertise<sensor_msgs::PointCloud2>("/surf_tree", 20);

    bool first_frame = true;

    while (ros::ok()) {
        ros::spinOnce();
        
        // Step 1: Get measurement
        if(!preprocess->SyncMeasurements()) {
            continue;
        }

        SensorMeasurement cur_mea = preprocess->sensor_measurement;

        // Step 2: Initialization 
        if (init->mode != ModeType::LIO && init->mode != ModeType::RELO) {
            ROS_ERROR_STREAM("Initialization Type Error !");
            ros::shutdown();
        }

        if (!init->lio_init && init->mode == ModeType::LIO) {
            if (!init->initStaticLio(cur_mea, edge_tree_lio, surf_tree_lio)) {
                ROS_WARN_STREAM(" LIO Initialization Loop ...");
                continue;
            } else {
                if (!gtsam->initialize(init->initLioState)) {
                    ROS_ERROR_STREAM("LIO GTSAM Error !");
                    ros::shutdown();
                }
            }
        } else if (!init->relo_init && init->mode == ModeType::RELO) {
            if (!init->initStaticLio(cur_mea, edge_tree_lio, surf_tree_lio) 
                || !init->initDynamicRelo(cur_mea)) 
            {
                ROS_WARN_STREAM(" RELO Initialization Loop ...");
                continue;
            } else {
                if (!gtsam->initialize(init->initReloState)) {
                    ROS_ERROR_STREAM("LIO GTSAM Error !");
                    ros::shutdown();
                }
            }
        }

        // Step 3: Optimization
        
        // imu preintegration factor
        gtsam->addImuFactor(cur_mea);

        // lidar odometry factor FIXME: wrong
        State predict_state = gtsam->imu_state;
        State trans_state;
        trans_state.pose.setIdentity();
        trans_state.pose = predict_state.pose * ext_state.pose;
        lm->runptimization(edge_tree_lio, cur_mea.edge_ptr_, 
                           surf_tree_lio, cur_mea.plane_ptr_,
                           trans_state);

        Eigen::Matrix3d rot = trans_state.pose.block(0, 0, 3, 3);
        Eigen::Vector3d trans = trans_state.pose.block(0, 3, 3, 1);
        Eigen::Vector4d quater = rotationToQuaternion(rot);
        gtsam::Pose3 lidar_pose = gtsam::Pose3(gtsam::Rot3(quater(3), quater(0), quater(1), quater(2)), gtsam::Point3(trans(0), trans(1), trans(2)));
        gtsam->addLidarFactor(lidar_pose, false);

        // scan matching factor

        // gtsam->optimize(1);
        // std::cout << gtsam->cur_state.pose << std::endl;

        // Publish kdtree
        pcl::PointCloud<PointType>::Ptr edge_cloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr surf_cloud(new pcl::PointCloud<PointType>());
        tree2Cloud(edge_tree_lio, edge_cloud);
        tree2Cloud(surf_tree_lio, surf_cloud);
        auto lidar_timestamp_ros = ros::Time().fromSec(cur_mea.bag_time_);
        publishCloud(pub_edge, edge_cloud, lidar_timestamp_ros, "map");
        publishCloud(pub_surf, surf_cloud, lidar_timestamp_ros, "map");


        continue;  // FIXME: why not using rate.sleep() ?
    }

    return 0;
}
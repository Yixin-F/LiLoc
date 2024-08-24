#include "utility.h"

#include <pclomp/ndt_omp.h>
#include <pclomp/gicp_omp.h>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

using namespace gtsam;
using symbol_shorthand::X; // Pose3 (x, y, z, r, p, y)
using symbol_shorthand::V; // Vel   (xdot, ydot, zdot)
using symbol_shorthand::B; // Bias  (ax, ay, az, gx, gy, gz)

/******************** Yaml Loader *******************/
std::string global_dir;
double downsample_resolution;

std::string odom_child_frame_id;
std::string lidarTopic;
bool have_ring_time_channel;
double lidarMinRange;
double lidarMaxRange;

std::string imuTopic;
double imuAccNoise;
double imuGyrNoise;
double gravity;
int imuFrequency;

std::vector<double> LiDAR2Imu_TransV;
std::vector<double> LiDAR2Imu_RotV;
std::vector<double> LiDAR2GT_TransV;
std::vector<double> LiDAR2GT_RotV;

Eigen::Vector3d LiDAR2Imu_Trans;
Eigen::Matrix3d LiDAR2Imu_Rot;
Eigen::Vector3d LiDAR2GT_Trans;
Eigen::Matrix3d LiDAR2GT_Rot;

double trans_detla, rot_detla;

double cool_time_duration, key_interval;

double odomLinearNoise, odomAngularNoise;
int active_factors, preserve_factors;

std::string ndt_neighbor_search_method;
double ndt_resolution, ndt_epsilon, penalty_threshold, penalty_weight;


/******************** System Members *******************/
ros::Subscriber imu_sub, points_sub, initialpose_sub;
ros::Publisher pose_pub, path_pub, LidarOdom_pub, ImuOdom_pub, globalmap_pub, goodvoxel_pub;

gtsam::Pose3 lidar2Imu;
gtsam::Pose3 lidar2gt;

boost::shared_ptr<pcl::VoxelGrid<PointT>> downsample_filter(new pcl::VoxelGrid<PointT>());

bool first_query = true, firstkey = true;

std::mutex imu_data_mutex;
std::deque<sensor_msgs::Imu> imuQueImu;
std::deque<sensor_msgs::Imu> imuQueOpt;
double lastImuT_opt = -1;
double lastImuT_imu = -1;
gtsam::NavState currentIMUState;

NonlinearFactorGraph newgraph;
gtsam::ISAM2 *optimizer;
gtsam::Values initialEstimate;
gtsam::Values isamCurrentEstimate;
gtsam::PreintegratedImuMeasurements *imuIntegratorOpt_;
gtsam::PreintegratedImuMeasurements *imuIntegratorImu_;
bool systemInitialized = false, doneFirstOpt = false;
int key_count = 0, time_count = 0, time_sum = 0, time_avg = 0;

gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise;
gtsam::noiseModel::Diagonal::shared_ptr priorVelNoise;
gtsam::noiseModel::Diagonal::shared_ptr priorBiasNoise;
gtsam::noiseModel::Diagonal::shared_ptr correctionNoise;
gtsam::Vector noiseModelBetweenBias;

gtsam::Pose3 prevPose_;
gtsam::Vector3 prevVel_;
gtsam::NavState prevState_;
gtsam::imuBias::ConstantBias prevBias_;
gtsam::NavState prevStateOdom;
gtsam::imuBias::ConstantBias prevBiasOdom;
gtsam::Pose3 lidarPose;
gtsam::Pose3 lidarPose_temp;
gtsam::Pose3 gt_pose;

pcl::Registration<PointT, PointT>::Ptr registration;
pclomp::NormalDistributionsTransform<PointT, PointT>::Ptr ndt(new pclomp::NormalDistributionsTransform<PointT, PointT>());
pclomp::GeneralizedIterativeClosestPoint<PointT, PointT>::Ptr gicp(new pclomp::GeneralizedIterativeClosestPoint<PointT, PointT>());

ofstream foutC;

/******************** Functions *******************/
void imuHandler(const sensor_msgs::ImuConstPtr& imu_msg) {
    std::lock_guard<std::mutex> lock(imu_data_mutex);

    sensor_msgs::Imu thisImu = imuConverter(*imu_msg);
    imuQueOpt.push_back(thisImu);
    imuQueImu.push_back(thisImu);

    double imuTime = getROSTime(&thisImu);
    double dt = (lastImuT_imu < 0) ? (1.0 / imuFrequency) : (imuTime - lastImuT_imu);
    if (dt < 1e-16) dt = 1e-16;

    lastImuT_imu = imuTime;

    // integrate this single imu message
    const auto& acc = thisImu.linear_acceleration;
    const auto& gyro = thisImu.angular_velocity;
    imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(acc.x, acc.y, acc.z), gtsam::Vector3(gyro.x, gyro.y, gyro.z), dt);

    // predict odometry
    currentIMUState = imuIntegratorImu_->predict(prevStateOdom, prevBiasOdom);

    // publish odometry
    nav_msgs::Odometry ImuOdom;
    ImuOdom.header.stamp = thisImu.header.stamp;
    ImuOdom.header.frame_id = "map";
    ImuOdom.child_frame_id = "odom_imu";

    gtsam::Pose3 imuPose = gtsam::Pose3(currentIMUState.quaternion(), currentIMUState.position());
    
    ImuOdom.pose.pose.position.x = imuPose.translation().x();
    ImuOdom.pose.pose.position.y = imuPose.translation().y();
    ImuOdom.pose.pose.position.z = imuPose.translation().z();
    ImuOdom.pose.pose.orientation.x = imuPose.rotation().toQuaternion().x();
    ImuOdom.pose.pose.orientation.y = imuPose.rotation().toQuaternion().y();
    ImuOdom.pose.pose.orientation.z = imuPose.rotation().toQuaternion().z();
    ImuOdom.pose.pose.orientation.w = imuPose.rotation().toQuaternion().w();
        
    ImuOdom.twist.twist.linear.x = currentIMUState.velocity().x();
    ImuOdom.twist.twist.linear.y = currentIMUState.velocity().y();
    ImuOdom.twist.twist.linear.z = currentIMUState.velocity().z();
    ImuOdom.twist.twist.angular.x = thisImu.angular_velocity.x + prevBiasOdom.gyroscope().x();
    ImuOdom.twist.twist.angular.y = thisImu.angular_velocity.y + prevBiasOdom.gyroscope().y();
    ImuOdom.twist.twist.angular.z = thisImu.angular_velocity.z + prevBiasOdom.gyroscope().z();
    ImuOdom_pub.publish(ImuOdom);
}

void pointHandler() {

}

void initPoseHandler() {

}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "LiLoc_node");
    ros::NodeHandle nh;

    std::cout << "\n"
              << "*************************************************" << "\n"
              << "**************** Hi, I'm LiLoc ! ****************" << "\n"
              << "*************************************************" << "\n"
              << std::endl;
    
    nh.param<std::string>("parameters/global_dir", global_dir, " ");
    nh.param<double>("parameters/downsample_resolution", downsample_resolution, 0.2);

    nh.param<std::string>("parameters/odom_child_frame_id", odom_child_frame_id, " ");
    nh.param<std::string>("parameters/lidarTopic", lidarTopic, " ");
    nh.param<bool>("parameters/have_ring_time_channel", have_ring_time_channel, true);
    nh.param<double>("parameters/lidarMinRange", lidarMinRange, 2.0);
    nh.param<double>("parameters/lidarMaxRange", lidarMaxRange, 80.0);

    nh.param<std::string>("parameters/imuTopic", imuTopic, " ");
    nh.param<double>("parameters/imuAccNoise", imuAccNoise, 0.01);
    nh.param<double>("parameters/imuGyrNoise", imuGyrNoise, 0.01);
    nh.param<double>("parameters/gravity", gravity, 9.8);
    nh.param<int>("parameters/imuFrequency", imuFrequency, 100);

    nh.param<vector<double>>("parameters/LiDAR2Imu_TransV", LiDAR2Imu_TransV, std::vector<double>());
    nh.param<vector<double>>("parameters/LiDAR2Imu_TransV", LiDAR2Imu_RotV, std::vector<double>());
    nh.param<vector<double>>("parameters/LiDAR2Imu_TransV", LiDAR2GT_TransV, std::vector<double>());
    nh.param<vector<double>>("parameters/LiDAR2Imu_TransV", LiDAR2GT_RotV, std::vector<double>());
    
    LiDAR2Imu_Trans = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(LiDAR2Imu_TransV.data(), 3, 1);
    LiDAR2Imu_Rot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(LiDAR2Imu_RotV.data(), 3, 3);
    LiDAR2GT_Trans = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(LiDAR2GT_TransV.data(), 3, 1);
    LiDAR2GT_Rot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(LiDAR2GT_RotV.data(), 3, 3);

    nh.param<double>("gtsam/odomLinearNoise", odomLinearNoise, 0.08);
    nh.param<double>("gtsam/odomAngularNoise", odomAngularNoise, 0.05);
    nh.param<int>("gtsam/active_factors", active_factors, 120);
    nh.param<int>("gtsam/active_factors", active_factors, 10);

    nh.param<std::string>("localization/ndt_neighbor_search_method", ndt_neighbor_search_method, " ");
    nh.param<double>("localization/ndt_resolution", ndt_resolution, 2.0);
    nh.param<double>("localization/ndt_epsilon", ndt_epsilon, 0.01);
    nh.param<double>("localization/penalty_threshold", penalty_threshold, 0.6);
    nh.param<double>("localization/penalty_weight", penalty_weight, 1.0);

    imu_sub = nh.subscribe(imuTopic, 2000, imuHandler);
    // points_sub = nh.subscribe<sensor_msgs::PointCloud2>(lidarTopic, 200, pointHandler);
    // initialpose_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 8, initPoseHandler);
        
    globalmap_pub = nh.advertise<sensor_msgs::PointCloud2>("/globalmap", 1);
    goodvoxel_pub = nh.advertise<sensor_msgs::PointCloud2>("/goodvoxel", 1000);
    ImuOdom_pub = nh.advertise<nav_msgs::Odometry>("/imu_incremental", 2000);
    LidarOdom_pub = nh.advertise<nav_msgs::Odometry>("/lidar_incremental", 2000);
    pose_pub = nh.advertise<nav_msgs::Odometry>("/loc_odom", 2000);
    path_pub = nh.advertise<nav_msgs::Path>("/loc_path", 1);

    lidar2Imu = gtsam::Pose3(gtsam::Rot3(LiDAR2Imu_Rot), gtsam::Point3(LiDAR2Imu_Trans.x(), LiDAR2Imu_Trans.y(), LiDAR2Imu_Trans.z()));
    lidar2gt = gtsam::Pose3(gtsam::Rot3(LiDAR2GT_Rot), gtsam::Point3(LiDAR2GT_Trans.x(), LiDAR2GT_Trans.y(), LiDAR2GT_Trans.z()));

    downsample_filter->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);

    // gtsam initialization
    boost::shared_ptr<gtsam::PreintegrationParams> p = gtsam::PreintegrationParams::MakeSharedU(gravity);
    p->accelerometerCovariance  = gtsam::Matrix33::Identity(3, 3) * pow(imuAccNoise, 2); // acc white noise in continuous
    p->gyroscopeCovariance      = gtsam::Matrix33::Identity(3, 3) * pow(imuGyrNoise, 2); // gyro white noise in continuous
    p->integrationCovariance    = gtsam::Matrix33::Identity(3, 3) * pow(1e-4, 2); // error committed in integrating position from velocities
    gtsam::imuBias::ConstantBias prior_imu_bias((gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished()); // assume zero initial bias
    priorPoseNoise  = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished()); // rad,rad,rad,m, m, m
    priorVelNoise   = gtsam::noiseModel::Isotropic::Sigma(3, 1e2); // m/s
    priorBiasNoise  = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3); // 1e-2 ~ 1e-3 seems to be good
    correctionNoise = gtsam::noiseModel::Isotropic::Sigma(6, 1); // meter
    noiseModelBetweenBias = (gtsam::Vector(6) << 3.99395e-03, 3.99395e-03, 3.99395e-03, 1.56363e-03, 1.56363e-03, 1.56363e-03).finished();
    imuIntegratorOpt_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias);
    imuIntegratorImu_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias);

    // TODO: localization settings, change this module
    ndt->setTransformationEpsilon(ndt_epsilon);
    ndt->setResolution(ndt_resolution);

    if(ndt_neighbor_search_method == "DIRECT1") {
        ndt->setNeighborhoodSearchMethod(pclomp::DIRECT1);
        registration = ndt;
    } 
    else if(ndt_neighbor_search_method == "DIRECT7") {
        ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
        registration = ndt;        
    }
    else if(ndt_neighbor_search_method == "KDTREE") {
        ndt->setNeighborhoodSearchMethod(pclomp::KDTREE);
        registration = ndt;
    } 
    else if(ndt_neighbor_search_method == "GICP_OMP"){
        registration = gicp;
    }
    else {
        ROS_WARN("Invalid search method was given ...");
    }

    // poses output file
    foutC = ofstream(global_dir + "poses.txt", ios::ate);
    foutC.setf(ios::fixed, ios::floatfield);
    
    ROS_INFO("Initialization Done ..");

    ros::spin();

    return 0;
}
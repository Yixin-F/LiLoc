#pragma once

#ifndef _UTILITY_H_
#define _UTILITY_H_

#define PCL_NO_PRECOMPILE 

#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <common_lib.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h> 
#include <pcl_conversions/pcl_conversions.h>

#include <opencv2/opencv.hpp>

#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
 
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>

#include "math_tools.h"

using gtsam::symbol_shorthand::X; // Pose3 (x, y, z, r, p, y)
using gtsam::symbol_shorthand::V; // Vel   (xdot, ydot, zdot)
using gtsam::symbol_shorthand::B; // Bias  (ax, ay, az, gx, gy, gz)

typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType> Cloud;
typedef Cloud::Ptr CloudPtr;

struct PointXYZIRPYT {
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time))

typedef PointXYZIRPYT  PointTypePose;
typedef pcl::PointCloud<PointTypePose> Trajectory;
typedef Trajectory::Ptr TrajectoryPtr;

std::shared_ptr<CommonLib::common_lib> common_lib_;

enum class SensorType { VELODYNE, OUSTER, LIVOX, ROBOSENSE, MULRAN };

enum class ModeType { LIO, RELO };

const static inline int kSessionStartIdxOffset = 1000000; // int max 2147483647 so ok.

const double kAccScale = 9.80665;

class ParamServer {
public:
    ros::NodeHandle nh;

    std::string pointCloudTopic;
    std::string imuTopic;
    std::string odomTopic;

    ModeType mode;

    int numberOfCores;

    std::string lidarFrame;
    std::string baselinkFrame;
    std::string odometryFrame;
    std::string mapFrame;

    std::string savePCDDirectory;
    std::string saveSessionDirectory;

    SensorType sensor;
    int N_SCAN;
    int Horizon_SCAN;

    bool have_ring_time_channel;

    int downsampleRate;
    int point_filter_num;

    float lidarMinRange;
    float lidarMaxRange;

    int imuType;
    float imuRate;
    float imuAccNoise;
    float imuGyrNoise;
    float imuAccBiasN;
    float imuGyrBiasN;
    float imuGravity;
    float imuRPYWeight;

    bool correct;

    std::vector<double> extRotV;
    std::vector<double> extRPYV;
    std::vector<double> extTransV;

    Eigen::Matrix3d extRot;
    Eigen::Matrix3d extRPY;

    Eigen::Vector3d extTrans;
    Eigen::Quaterniond extQRPY;

    float z_tollerance; 
    float rotation_tollerance;

    std::string regMethod;
    float ndtResolution;
    float ndtEpsilon;

    float timeInterval;

    double mappingProcessInterval;
    
    float surroundingKeyframeMapLeafSize;
    float mappingSurfLeafSize;

    float surroundingkeyframeAddingDistThreshold; 
    float surroundingkeyframeAddingAngleThreshold; 
    float surroundingKeyframeDensity;
    float surroundingKeyframeSearchRadius;

    float globalMapVisualizationSearchRadius;
    float globalMapVisualizationPoseDensity;
    float globalMapVisualizationLeafSize;

    ~ParamServer() { }

    ParamServer() {
        nh.param<std::string>("System/pointCloudTopic", pointCloudTopic, "points_raw");
        nh.param<std::string>("System/imuTopic", imuTopic, "imu_correct");
        nh.param<std::string>("System/odomTopic", odomTopic, "odometry/imu");

        std::string modeStr;
        nh.param<std::string>("System/mode", modeStr, "lio");
        if (modeStr == "lio") {
            mode = ModeType::LIO;
        }
        else if (modeStr == "relo") {
            mode = ModeType::RELO;
        }
        else {
            ROS_ERROR_STREAM("Invalid Mode Type (must be either 'lio' or 'relo'): " << modeStr);
            ros::shutdown();
        }

        nh.param<int>("System/numberOfCores", numberOfCores, 4);

        nh.param<std::string>("System/lidarFrame", lidarFrame, "base_link");
        nh.param<std::string>("System/baselinkFrame", baselinkFrame, "base_link");
        nh.param<std::string>("System/odometryFrame", odometryFrame, "odom");
        nh.param<std::string>("System/mapFrame", mapFrame, "map");

        nh.param<std::string>("System/savePCDDirectory", savePCDDirectory, "/Downloads/LOAM/");
        nh.param<std::string>("System/saveSessionDirectory", saveSessionDirectory, "/Downloads/LOAM/");

        std::string sensorStr;
        nh.param<std::string>("Sensors/sensor", sensorStr, " ");
        if (sensorStr == "velodyne")
        {
            sensor = SensorType::VELODYNE;
        }
        else if (sensorStr == "ouster")
        {
            sensor = SensorType::OUSTER;
        }
        else if (sensorStr == "livox")
        {
            sensor = SensorType::LIVOX;
        } 
        else if  (sensorStr == "robosense") {
            sensor = SensorType::ROBOSENSE;
        }
        else if (sensorStr == "mulran")
        {
            sensor = SensorType::MULRAN;
        } 
        else {
            ROS_ERROR_STREAM("Invalid Sensor Type (must be either 'velodyne' or 'ouster' or 'livox' or 'robosense' or 'mulran'): " << sensorStr);
            ros::shutdown();
        }

        nh.param<int>("Sensors/N_SCAN", N_SCAN, 16);
        nh.param<int>("Sensors/Horizon_SCAN", Horizon_SCAN, 1800);

        nh.param<bool>("Sensors/have_ring_time_channel", have_ring_time_channel, true);

        nh.param<int>("Sensors/downsampleRate", downsampleRate, 1);
        nh.param<int>("Sensors/point_filter_num", point_filter_num, 3);

        nh.param<float>("Sensors/lidarMinRange", lidarMinRange, 1.0);
        nh.param<float>("Sensors/lidarMaxRange", lidarMaxRange, 1000.0);

        nh.param<int>("Sensors/imuType", imuType, 0);
        nh.param<float>("Sensors/imuRate", imuRate, 500.0);
        nh.param<float>("Sensors/imuAccNoise", imuAccNoise, 0.01);
        nh.param<float>("Sensors/imuGyrNoise", imuGyrNoise, 0.001);
        nh.param<float>("Sensors/imuAccBiasN", imuAccBiasN, 0.0002);
        nh.param<float>("Sensors/imuGyrBiasN", imuGyrBiasN, 0.00003);
        nh.param<float>("Sensors/imuGravity", imuGravity, 9.80511);
        nh.param<float>("Sensors/imuRPYWeight", imuRPYWeight, 0.01);

        nh.param<bool>("Sensors/correct", correct, false);

        nh.param<std::vector<double>>("Sensors/extrinsicRot", extRotV, std::vector<double>());
        nh.param<std::vector<double>>("Sensors/extrinsicRPY", extRPYV, std::vector<double>());
        nh.param<std::vector<double>>("Sensors/extrinsicTrans", extTransV, std::vector<double>());

        extRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRotV.data(), 3, 3);
        extRPY = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRPYV.data(), 3, 3);
        extTrans = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extTransV.data(), 3, 1);
        extQRPY = Eigen::Quaterniond(extRPY).inverse();

        nh.param<float>("Mapping/z_tollerance", z_tollerance, FLT_MAX);
        nh.param<float>("Mapping/rotation_tollerance", rotation_tollerance, FLT_MAX);

        nh.param<std::string>("Mapping/regMethod", regMethod, "DIRECT1");
        nh.param<float>("Mapping/ndtResolution", ndtResolution, 1.0);
        nh.param<float>("Mapping/ndtEpsilon", ndtEpsilon, 0.01);

        nh.param<float>("Mapping/timeInterval", timeInterval, 0.2);

        nh.param<double>("Mapping/mappingProcessInterval", mappingProcessInterval, 0.15);

        nh.param<float>("Mapping/mappingSurfLeafSize", mappingSurfLeafSize, 0.2);
        nh.param<float>("Mapping/surroundingKeyframeMapLeafSize", surroundingKeyframeMapLeafSize, 0.4);

        nh.param<float>("Mapping/surroundingkeyframeAddingDistThreshold", surroundingkeyframeAddingDistThreshold, 1.0);
        nh.param<float>("Mapping/surroundingkeyframeAddingAngleThreshold", surroundingkeyframeAddingAngleThreshold, 0.2);
        nh.param<float>("Mapping/surroundingKeyframeDensity", surroundingKeyframeDensity, 1.0);
        nh.param<float>("Mapping/surroundingKeyframeSearchRadius", surroundingKeyframeSearchRadius, 50.0);

        nh.param<float>("Mapping/globalMapVisualizationSearchRadius", globalMapVisualizationSearchRadius, 1e3);
        nh.param<float>("Mapping/globalMapVisualizationPoseDensity", globalMapVisualizationPoseDensity, 10.0);
        nh.param<float>("Mapping/globalMapVisualizationLeafSize", globalMapVisualizationLeafSize, 1.0);

        usleep(100);
    }

    sensor_msgs::Imu imuConverter(const sensor_msgs::Imu& imu_in) {
        sensor_msgs::Imu imu_out = imu_in;
        // rotate acceleration
        Eigen::Vector3d acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);
        acc = extRot * acc;
        imu_out.linear_acceleration.x = acc.x();
        imu_out.linear_acceleration.y = acc.y();
        imu_out.linear_acceleration.z = acc.z();
        // rotate gyroscope
        Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
        gyr = extRot * gyr;
        imu_out.angular_velocity.x = gyr.x();
        imu_out.angular_velocity.y = gyr.y();
        imu_out.angular_velocity.z = gyr.z();

        if (imuType) {
            // rotate roll pitch yaw
            Eigen::Quaterniond q_from(imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z);
            Eigen::Quaterniond q_final = q_from * extQRPY;
            imu_out.orientation.x = q_final.x();
            imu_out.orientation.y = q_final.y();
            imu_out.orientation.z = q_final.z();
            imu_out.orientation.w = q_final.w();

            if (sqrt(q_final.x()*q_final.x() + q_final.y()*q_final.y() + q_final.z()*q_final.z() + q_final.w()*q_final.w()) < 0.1)
            {
                ROS_ERROR("Invalid quaternion, please use a 9-axis IMU!");
                ros::shutdown();
            }
        }

        return imu_out;
    }
};

template<typename T>
sensor_msgs::PointCloud2 publishCloud(const ros::Publisher& thisPub, const T& thisCloud, ros::Time thisStamp, std::string thisFrame) {
    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;

    if (thisPub.getNumSubscribers() != 0)
        thisPub.publish(tempCloud);

    return tempCloud;
}

template<typename T>
double ROS_TIME(T msg) {
    return msg->header.stamp.toSec();
}


template<typename T>
void imuAngular2rosAngular(sensor_msgs::Imu *thisImuMsg, T *angular_x, T *angular_y, T *angular_z) {
    *angular_x = thisImuMsg->angular_velocity.x;
    *angular_y = thisImuMsg->angular_velocity.y;
    *angular_z = thisImuMsg->angular_velocity.z;
}


template<typename T>
void imuAccel2rosAccel(sensor_msgs::Imu *thisImuMsg, T *acc_x, T *acc_y, T *acc_z) {
    *acc_x = thisImuMsg->linear_acceleration.x;
    *acc_y = thisImuMsg->linear_acceleration.y;
    *acc_z = thisImuMsg->linear_acceleration.z;
}


template<typename T>
void imuRPY2rosRPY(sensor_msgs::Imu *thisImuMsg, T *rosRoll, T *rosPitch, T *rosYaw) {
    double imuRoll, imuPitch, imuYaw;
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(thisImuMsg->orientation, orientation);
    tf::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);

    *rosRoll = imuRoll;
    *rosPitch = imuPitch;
    *rosYaw = imuYaw;
}

pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose* transformIn){
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);

    Eigen::Affine3f transCur = pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z, transformIn->roll, transformIn->pitch, transformIn->yaw);
    
    const int numberOfCores = 8;
    #pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < cloudSize; ++i)
    {
        const auto &pointFrom = cloudIn->points[i];
        cloudOut->points[i].x = transCur(0, 0) * pointFrom.x + transCur(0, 1) * pointFrom.y + transCur(0, 2) * pointFrom.z + transCur(0, 3);
        cloudOut->points[i].y = transCur(1, 0) * pointFrom.x + transCur(1, 1) * pointFrom.y + transCur(1, 2) * pointFrom.z + transCur(1, 3);
        cloudOut->points[i].z = transCur(2, 0) * pointFrom.x + transCur(2, 1) * pointFrom.y + transCur(2, 2) * pointFrom.z + transCur(2, 3);
        cloudOut->points[i].intensity = pointFrom.intensity;
    }
    return cloudOut;
}

gtsam::Pose3 pclPointTogtsamPose3(PointTypePose thisPoint) {
    return gtsam::Pose3(gtsam::Rot3::RzRyRx(double(thisPoint.roll), double(thisPoint.pitch), double(thisPoint.yaw)),
                        gtsam::Point3(double(thisPoint.x),    double(thisPoint.y),     double(thisPoint.z)));
}

PointTypePose gtsamPose3ToPclPoint(gtsam::Pose3 point) {
    PointTypePose pose;
    pose.x = point.translation().x();
    pose.y = point.translation().y();
    pose.z = point.translation().z();

    pose.roll = point.rotation().roll();
    pose.pitch = point.rotation().pitch();
    pose.yaw = point.rotation().yaw();

    return pose;
}

Eigen::Affine3f pclPointToAffine3f(PointTypePose thisPoint) { 
    return pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z, thisPoint.roll, thisPoint.pitch, thisPoint.yaw);
}

int ungenGlobalNodeIdx (const int& _session_idx, const int& _idx_in_graph) {
    return (_idx_in_graph - 1) / (_session_idx * kSessionStartIdxOffset);
} // ungenGlobalNodeIdx

int genGlobalNodeIdx (const int& _session_idx, const int& _node_offset) {
    return (_session_idx * kSessionStartIdxOffset) + _node_offset + 1;
} // genGlobalNodeIdx

int genAnchorNodeIdx (const int& _session_idx) {
    return (_session_idx * kSessionStartIdxOffset);
} // genAnchorNodeIdx

void writeVertex(const int _node_idx, const gtsam::Pose3& _initPose, std::vector<std::string>& vertices_str){
    gtsam::Point3 t = _initPose.translation();
    gtsam::Rot3 R = _initPose.rotation();

    std::string curVertexInfo {
        "VERTEX_SE3:QUAT " + std::to_string(_node_idx) + " "
         + std::to_string(t.x()) + " " + std::to_string(t.y()) + " " + std::to_string(t.z())  + " " 
        + std::to_string(R.toQuaternion().x()) + " " + std::to_string(R.toQuaternion().y()) + " " 
        + std::to_string(R.toQuaternion().z()) + " " + std::to_string(R.toQuaternion().w()) };

    vertices_str.emplace_back(curVertexInfo);
}

void writeEdge(const std::pair<int, int> _node_idx_pair, const gtsam::Pose3& _relPose, std::vector<std::string>& edges_str){
    gtsam::Point3 t = _relPose.translation();
    gtsam::Rot3 R = _relPose.rotation();

    std::string curEdgeInfo {
        "EDGE_SE3:QUAT " + std::to_string(_node_idx_pair.first) + " " + std::to_string(_node_idx_pair.second) + " "
        + std::to_string(t.x()) + " " + std::to_string(t.y()) + " " + std::to_string(t.z())  + " " 
        + std::to_string(R.toQuaternion().x()) + " " + std::to_string(R.toQuaternion().y()) + " " 
        + std::to_string(R.toQuaternion().z()) + " " + std::to_string(R.toQuaternion().w()) };

    edges_str.emplace_back(curEdgeInfo);  
}

template <typename T>
inline bool HasInf(const T& p) {
  return (std::isinf(p.x) || std::isinf(p.y) || std::isinf(p.z));
}

template <typename T>
inline bool HasNan(const T& p) {
  return (std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z));
}

template <typename T>
inline bool IsNear(const T& p1, const T& p2) {
  return ((abs(p1.x - p2.x) < 1e-7) || (abs(p1.y - p2.y) < 1e-7) ||
          (abs(p1.z - p2.z) < 1e-7));
}

template<typename T> 
float pointDistance(const T& p) { 
    return sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

template<typename T>
float pointDistance(const T& p1, const T& p2) {
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
}

#endif

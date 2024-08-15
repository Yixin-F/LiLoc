#pragma once
#ifndef _UTILITY_H_
#define _UTILITY_H_
#define PCL_NO_PRECOMPILE

#include <ros/ros.h>

#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <visualization_msgs/MarkerArray.h>

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

#include <tf_conversions/tf_eigen.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <boost/circular_buffer.hpp>
#include <boost/format.hpp>

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

// lidar type
struct VelodynePointXYZIRT {
    PCL_ADD_POINT4D;
    PCL_ADD_INTENSITY;
    int ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(VelodynePointXYZIRT,
        (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
        (int, ring, ring)(float, time, time))

struct OusterPointXYZIRT {
    PCL_ADD_POINT4D;
    float intensity;
    float t;
    float reflectivity;
    int ring;
    float noise;
    float range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPointXYZIRT,
        (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
        (float, t, t)(float, reflectivity, reflectivity)
        (int, ring, ring)(float, noise, noise)(float, range, range))

// pose type
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

typedef PointXYZIRPYT PointTypePose;

// point type
typedef pcl::PointXYZINormal PointType;

enum class SensorType {
    VELODYNE, OUSTER, HORIZON, MID40, MID360, AVIA
};

enum Feature {
    Nor, Poss_Plane, Real_Plane, Edge_Jump, Edge_Plane, Wire, ZeroPoint
};

enum Surround {
    Prev, Next
};

enum E_jump {
    Nr_nor, Nr_zero, Nr_180, Nr_inf, Nr_blind
};

struct orgtype {
    double range;
    double dista; 
    double angle[2];
    double intersect;
    E_jump edj[2];
    Feature ftype;
    orgtype() {
        range = 0;
        edj[Prev] = Nr_nor;
        edj[Next] = Nr_nor;
        ftype = Nor;
        intersect = 2;
    }
};

struct SensorMeasurement {
    double bag_time_{0.0};

    double lidar_start_time_{0.0};
    double lidar_end_time_{0.0};

    pcl::PointCloud<PointType>::Ptr cloud_ptr_{};
    pcl::PointCloud<PointType>::Ptr plane_ptr_{};
    pcl::PointCloud<PointType>::Ptr edge_ptr_{};

    std::deque<sensor_msgs::Imu> imu_buff_;
};

class ParamServer {
public:

    ros::NodeHandle nh;

    int robot_id;  // online multi-session exploration
    std::string root_suffix;  // lifelong localization with prior knowledge
    int numberOfCores;

    // topic names
    std::string pointCloudTopic;
    std::string imuTopic;
    std::string imuOdomTopic;
    std::string lidarOdomTopic;

    float timeScale;

    // LiDAR settings
    SensorType sensor;
    int N_SCAN;
    int Horizon_SCAN;
    float lidarMinRange;
    float lidarMaxRange;
    int mapFrequence;

    // IMU settings
    int imuFrequence;
    float imuAccNoise;
    float imuGyrNoise;
    float imuAccBiasN;
    float imuGyrBiasN;
    float imuGravity;
    float imuRPYWeight;

    Eigen::Vector3d imuAccBias_N;
    Eigen::Vector3d imuGyrBias_N;
    Eigen::Vector3d imuGravity_N;
    std::vector<double> imuAccBias_NV;
    std::vector<double> imuGyrBias_NV;
    std::vector<double> imuGravity_NV;

    std::vector<double> extRotV;
    std::vector<double> extRPYV;
    std::vector<double> extTransV;
    Eigen::Matrix3d extRot;
    Eigen::Matrix3d extRPY;
    Eigen::Vector3d extTrans;
    Eigen::Quaterniond extQRPY;
    Eigen::Quaterniond q_sensor_body;
    Eigen::Vector3d t_sensor_body;
    Eigen::Quaterniond q_body_sensor;
    Eigen::Vector3d t_body_sensor;

    // feature
    float blind;
    float inf_bound;
    int group_size;
    float disA;
    float disB;
    float p2l_ratio;
    float limit_maxmid;
    float limit_midmin;
    float limit_maxmin;
    float jump_up_limit;
    float jump_down_limit;
    float cos160;
    float edgea;
    float edgeb;
    float smallp_intersect;
    float smallp_ratio;
    int point_filter_num;

    // filter params
    float mappingCornerLeafSize;
    float mappingSurfLeafSize;
    float globalMapLeafSize;

    // Surrounding map
    float surroundingkeyframeAddingDistThreshold;
    float surroundingkeyframeAddingAngleThreshold;
    float surroundingkeyframeIntervalTime;
    float surroundingKeyframeSearchRadius;

    ~ParamServer() { }

    ParamServer() {
        nh.param<int>("system/robot_id", robot_id, 0);
        nh.param<std::string>("system/root_suffix", root_suffix, " ");
        // ROS_INFO_STREAM("ROOT SUFFIX: " << root_suffix);
        nh.param<int>("system/numberOfCores", numberOfCores, 8);

        nh.param<std::string>("system/pointCloudTopic", pointCloudTopic, " ");
        nh.param<std::string>("system/imuTopic", imuTopic, " ");
        nh.param<std::string>("system/imuOdomTopic", imuOdomTopic, " ");
        nh.param<std::string>("system/lidarOdomTopic", lidarOdomTopic, " ");

        nh.param<float>("system/timeScale", timeScale, 1000.0);

        std::string sensorStr;
        nh.param<std::string>("factor/sensor", sensorStr, " ");
        if (sensorStr == "velodyne") {
            sensor = SensorType::VELODYNE;
        } else if (sensorStr == "ouster") {
            sensor = SensorType::OUSTER;
        } else if (sensorStr == "horizon") {
            sensor = SensorType::HORIZON;
        } else if (sensorStr == "mid40") {
            sensor = SensorType::MID40;
        } else if (sensorStr == "mid360") {
            sensor = SensorType::MID360;
        } else if (sensorStr == "avia") {
            sensor = SensorType::AVIA;
        } else {
            ROS_ERROR_STREAM("Invalid sensor type (must be either 'velodyne', 'ouster', 'horizon', 'mid40', 'mid360' or 'avia'): " << sensorStr);
            ros::shutdown();
        }
        // ROS_INFO_STREAM("SENSOR TYPE: " << sensorStr);

        nh.param<int>("factor/N_SCAN", N_SCAN, 16);
        nh.param<int>("factor/Horizon_SCAN", Horizon_SCAN, 1800);
        nh.param<float>("factor/lidarMinRange", lidarMinRange, 2.0);
        nh.param<float>("factor/lidarMaxRange", lidarMaxRange, 80.0);
        nh.param<int>("factor/mapFrequence", mapFrequence, 10);

        nh.param<int>("factor/imuFrequence", imuFrequence, 100);
        nh.param<float>("factor/imuAccNoise", imuAccNoise, 0.01);
        nh.param<float>("factor/imuGyrNoise", imuGyrNoise, 0.01);
        nh.param<float>("factor/imuAccBiasN", imuAccBiasN, 0.01);
        nh.param<float>("factor/imuGyrBiasN", imuGyrBiasN, 0.01);
        nh.param<float>("factor/imuGravity", imuGravity, 9.8);
        nh.param<float>("factor/imuRPYWeight", imuRPYWeight, 0.01);

        nh.param<std::vector<double>>("factor/extrinsicRot", extRotV, std::vector<double>());
        nh.param<std::vector<double>>("factor/extrinsicRPY", extRPYV, std::vector<double>());
        nh.param<std::vector<double>>("factor/extrinsicTrans", extTransV, std::vector<double>());
        extRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRotV.data(), 3, 3);
        extRPY = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRPYV.data(), 3, 3);
        extTrans = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extTransV.data(), 3, 1);
        extQRPY = Eigen::Quaterniond(extRPY);

        q_sensor_body = Eigen::Quaterniond(extRPY);
        t_sensor_body = extTrans;
        q_body_sensor = q_sensor_body.inverse();
        t_body_sensor = -(q_sensor_body.inverse() * t_sensor_body);

        nh.param<float>("feature/blind", blind, 0.5);
        nh.param<float>("feature/inf_bound", inf_bound, 10);
        nh.param<int>("feature/group_size", group_size, 8);
        nh.param<float>("feature/disA", disA, 0.01);
        nh.param<float>("feature/disB", disB, 0.1);
        nh.param<float>("feature/p2l_ratio", p2l_ratio, 400);
        nh.param<float>("feature/limit_maxmid", limit_maxmid, 9);
        nh.param<float>("feature/limit_midmin", limit_midmin, 16);
        nh.param<float>("feature/limit_maxmin", limit_maxmin, 3.24);
        nh.param<float>("feature/jump_up_limit", jump_up_limit, 175.0);
        nh.param<float>("feature/jump_down_limit", jump_down_limit, 5.0);
        nh.param<float>("feature/cos160", cos160, 160.0);
        nh.param<float>("feature/edgea", edgea, 3);
        nh.param<float>("feature/edgeb", edgeb, 0.05);
        nh.param<float>("feature/smallp_intersect", smallp_intersect, 170.0);
        nh.param<float>("feature/smallp_ratio", smallp_ratio, 1.2);
        nh.param<int>("feature/point_filter_num", point_filter_num, 4);
        jump_up_limit = cos(jump_up_limit / 180 * M_PI);
        jump_down_limit = cos(jump_down_limit / 180 * M_PI);
        cos160 = cos(cos160 / 180 * M_PI);
        smallp_intersect = cos(smallp_intersect / 180 * M_PI);

        nh.param<float>("mapping/mappingCornerLeafSize", mappingCornerLeafSize, 0.2);
        nh.param<float>("mapping/mappingSurfLeafSize", mappingSurfLeafSize, 0.2);
        nh.param<float>("mapping/globalMapLeafSize", globalMapLeafSize, 0.4);

        nh.param<float>("mapping/surroundingkeyframeAddingDistThreshold", surroundingkeyframeAddingDistThreshold, 0.5);
        nh.param<float>("mapping/surroundingkeyframeAddingAngleThreshold", surroundingkeyframeAddingAngleThreshold, 0.2);
        nh.param<float>("mapping/surroundingkeyframeIntervalTime", surroundingkeyframeIntervalTime, 0.1);
        nh.param<float>("mapping/surroundingKeyframeSearchRadius", surroundingKeyframeSearchRadius, 50.0);

        // ROS_INFO_STREAM("PARAMETERS LOAD DONE ...");
    }

    sensor_msgs::Imu imuConverter(const sensor_msgs::Imu &imu_in) {
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

        // rotate roll pitch yaw
        Eigen::Quaterniond q_from(imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y,
                                  imu_in.orientation.z);
        Eigen::Quaterniond q_final = extQRPY;

        q_final.normalize();
        imu_out.orientation.x = q_final.x();
        imu_out.orientation.y = q_final.y();
        imu_out.orientation.z = q_final.z();
        imu_out.orientation.w = q_final.w();

        if (sqrt(q_final.x() * q_final.x() + q_final.y() * q_final.y() + q_final.z() * q_final.z() + q_final.w() * q_final.w()) < 0.1) {
            ROS_ERROR("Invalid quaternion, please use a 9-axis IMU!");
            ros::shutdown();
        }

        return imu_out;
    }
};

template<typename T>
sensor_msgs::PointCloud2 publishCloud(const ros::Publisher &thisPub, const T &thisCloud, ros::Time thisStamp, std::string thisFrame) {
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

template <typename T>
float pointDistance(const T& p) {
    return sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

template <typename T>
float pointDistance(const T& p1, const T& p2) {
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
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

#endif
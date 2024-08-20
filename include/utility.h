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

#include <experimental/filesystem> // file gcc>=8
#include <experimental/optional>

#include "ikd-Tree/ikd_Tree.h"
#include "math_tools.h"

namespace fs = std::experimental::filesystem;

#define NUM_MATCH_POINTS 5
#define MATCH_DIS 2
#define PLANE_TH 0.1
#define LINE_TH 3.0
#define LM_ITERATION 20

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

enum class ModeType {
    LIO, RELO
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

// measurement
struct SensorMeasurement {
    double bag_time_{0.0};

    double lidar_start_time_{0.0};
    double lidar_end_time_{0.0};

    pcl::PointCloud<PointType>::Ptr cloud_ptr_{};
    pcl::PointCloud<PointType>::Ptr plane_ptr_{};
    pcl::PointCloud<PointType>::Ptr edge_ptr_{};

    std::deque<sensor_msgs::Imu> imu_buff_;
};

// edge
struct Edge {
    int from_idx;
    int to_idx;
    gtsam::Pose3 relative;
};

// node
struct Node {
    int idx;
    gtsam::Pose3 initial;
};

// pose
struct pose
{
    Eigen::Vector3d t;
    Eigen::Matrix3d R;
};

// g2o format
struct G2oLineInfo {
    std::string type;

    int prev_idx = -1; // for vertex, this member is null
    int curr_idx;

    std::vector<double> trans;
    std::vector<double> quat;

    inline static const std::string kVertexTypeName = "VERTEX_SE3:QUAT";
    inline static const std::string kEdgeTypeName = "EDGE_SE3:QUAT";
}; 

using SessionNodes = std::multimap<int, Node>; // from_idx, Node
using SessionEdges = std::multimap<int, Edge>; // from_idx, Edge

struct State {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    Eigen::Vector3d vel = Eigen::Vector3d::Zero();
    Eigen::Vector3d ba = Eigen::Vector3d::Zero();
    Eigen::Vector3d bg = Eigen::Vector3d::Zero();
};

State ext_state;

class ParamServer {
public:

    ros::NodeHandle nh;

    int robot_id;  // online multi-session exploration
    std::string root_suffix;  // lifelong localization with prior knowledge
    int numberOfCores;

    ModeType mode;

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
    std::vector<double> extTransV;
    Eigen::Matrix3d extRot;
    Eigen::Vector3d extTrans;
    Eigen::Matrix3d extRot_inv;
    Eigen::Vector3d extTrans_inv;

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

        std::string modeStr;
        nh.param<std::string>("system/mode", modeStr, "lio");
        if (modeStr == "lio") {
            mode = ModeType::LIO;
        } else if (modeStr == "relo") {
            mode = ModeType::RELO;
        }

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
        nh.param<std::vector<double>>("factor/extrinsicTrans", extTransV, std::vector<double>());
        extRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRotV.data(), 3, 3);
        extTrans = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extTransV.data(), 3, 1);
        extRot_inv = extRot.inverse();
        extTrans_inv = - extRot.inverse() * extTrans;

        ext_state.pose.setIdentity();
        ext_state.pose.block(0, 0, 3, 3) = extRot;
        ext_state.pose.block(0, 3, 3, 1) = extTrans;

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
        acc = extRot_inv * acc;
        imu_out.linear_acceleration.x = acc.x();
        imu_out.linear_acceleration.y = acc.y();
        imu_out.linear_acceleration.z = acc.z();

        // rotate gyroscope
        Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
        gyr = extRot_inv * gyr;
        imu_out.angular_velocity.x = gyr.x();
        imu_out.angular_velocity.y = gyr.y();
        imu_out.angular_velocity.z = gyr.z();

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

bool isTwoStringSame(std::string _str1, std::string _str2) {
	return !(_str1.compare(_str2));
}

// read g2o, example：VERTEX_SE3:QUAT 99 -61.332581 -9.253125 0.131973 -0.004256 -0.005810 -0.625732 0.780005
G2oLineInfo splitG2oFileLine(std::string _str_line) {
    std::stringstream ss(_str_line);

	std::vector<std::string> parsed_elms ;
    std::string elm;
	char delimiter = ' ';
    while (getline(ss, elm, delimiter)) {
        parsed_elms.push_back(elm); // convert string to "double"
    }

	G2oLineInfo parsed;

	if( isTwoStringSame(parsed_elms.at(0), G2oLineInfo::kVertexTypeName) )
	{
		parsed.type = parsed_elms.at(0);
		parsed.curr_idx = std::stoi(parsed_elms.at(1));
		parsed.trans.push_back(std::stod(parsed_elms.at(2)));
		parsed.trans.push_back(std::stod(parsed_elms.at(3)));
		parsed.trans.push_back(std::stod(parsed_elms.at(4)));
		parsed.quat.push_back(std::stod(parsed_elms.at(5)));
		parsed.quat.push_back(std::stod(parsed_elms.at(6)));
		parsed.quat.push_back(std::stod(parsed_elms.at(7)));
		parsed.quat.push_back(std::stod(parsed_elms.at(8)));
	}
	if( isTwoStringSame(parsed_elms.at(0), G2oLineInfo::kEdgeTypeName) )
	{
		parsed.type = parsed_elms.at(0);
		parsed.prev_idx = std::stoi(parsed_elms.at(1));
		parsed.curr_idx = std::stoi(parsed_elms.at(2));
		parsed.trans.push_back(std::stod(parsed_elms.at(3)));
		parsed.trans.push_back(std::stod(parsed_elms.at(4)));
		parsed.trans.push_back(std::stod(parsed_elms.at(5)));
		parsed.quat.push_back(std::stod(parsed_elms.at(6)));
		parsed.quat.push_back(std::stod(parsed_elms.at(7)));
		parsed.quat.push_back(std::stod(parsed_elms.at(8)));
		parsed.quat.push_back(std::stod(parsed_elms.at(9)));
	}

	return parsed;
}

// filesort by name
bool fileNameSort(std::string name1_, std::string name2_){
    std::string::size_type iPos1 = name1_.find_last_of('/') + 1;
	std::string filename1 = name1_.substr(iPos1, name1_.length() - iPos1);
	std::string name1 = filename1.substr(0, filename1.rfind("."));

    std::string::size_type iPos2 = name2_.find_last_of('/') + 1;
    std::string filename2 = name2_.substr(iPos2, name2_.length() - iPos2);
	std::string name2 = filename2.substr(0, filename2.rfind(".")); 

    return std::stoi(name1) < std::stoi(name2);
}

template <typename C, typename D, typename Getter>
void ComputeMeanAndCovDiag(const C& data,
                           D& mean,
                           D& cov_diag,
                           Getter&& getter) {
  size_t len = data.size();
  assert(len > 1);
  // clang-format off
    mean = std::accumulate(data.begin(), data.end(), D::Zero().eval(),
                           [&getter](const D& sum, const auto& data) -> D { return sum + getter(data); }) / len;
    cov_diag = std::accumulate(data.begin(), data.end(), D::Zero().eval(),
                               [&mean, &getter](const D& sum, const auto& data) -> D {
                                   return sum + (getter(data) - mean).cwiseAbs2().eval();
                               }) / (len - 1);
  // clang-format on
}

template<typename T>
bool esti_plane(Eigen::Matrix<T, 4, 1> &pca_result, const KD_TREE<PointType>::PointVector &point, const T &threshold) {  // plane
    Eigen::Matrix<T, NUM_MATCH_POINTS, 3> A;
    Eigen::Matrix<T, NUM_MATCH_POINTS, 1> b;
    A.setZero();
    b.setOnes();
    b *= -1.0f;

    for (int j = 0; j < NUM_MATCH_POINTS; j++){
        A(j, 0) = point[j].x;
        A(j, 1) = point[j].y;
        A(j, 2) = point[j].z;
    }

    Eigen::Matrix<T, 3, 1> normvec = A.colPivHouseholderQr().solve(b);

    T n = normvec.norm();
    pca_result(0) = normvec(0) / n;
    pca_result(1) = normvec(1) / n;
    pca_result(2) = normvec(2) / n;
    pca_result(3) = 1.0 / n;

    for (int j = 0; j < NUM_MATCH_POINTS; j++)
    {
        if (fabs(pca_result(0) * point[j].x + pca_result(1) * point[j].y + pca_result(2) * point[j].z + pca_result(3)) > threshold)
        {
            return false;
        }
    }
    return true;
}

template<typename T>
bool esti_line(Eigen::Matrix<T, 4, 1> &pca_result, const KD_TREE<PointType>::PointVector &point, const T &threshold, const PointType& pointWorld) {
    Eigen::Matrix<float, 3, 3> matA1;
    Eigen::Matrix<float, 1, 3> matD1;
    Eigen::Matrix<float, 3, 3> matV1;

    float cx = 0, cy = 0, cz = 0;
    for (const auto& pt : point) {
        cx += pt.x;
        cy += pt.y;
        cz += pt.z;
    }
    cx /= NUM_MATCH_POINTS;
    cy /= NUM_MATCH_POINTS;
    cz /= NUM_MATCH_POINTS;

    float a11 = 0, a12 = 0, a13 = 0, a22 = 0, a23 = 0, a33 = 0;
    for (const auto& pt : point) {
        float ax = pt.x - cx;
        float ay = pt.y - cy;
        float az = pt.z - cz;

        a11 += ax * ax; a12 += ax * ay; a13 += ax * az;
        a22 += ay * ay; a23 += ay * az;
        a33 += az * az;
    }
    a11 /= NUM_MATCH_POINTS; 
    a12 /= NUM_MATCH_POINTS; 
    a13 /= NUM_MATCH_POINTS; 
    a22 /= NUM_MATCH_POINTS; 
    a23 /= NUM_MATCH_POINTS; 
    a33 /= NUM_MATCH_POINTS;

    matA1(0, 0) = a11; matA1(0, 1) = a12; matA1(0, 2) = a13;
    matA1(1, 0) = a12; matA1(1, 2) = a22; matA1(1, 3) = a23;
    matA1(2, 0) = a13; matA1(2, 1) = a23; matA1(2, 2) = a33;

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigensolver(matA1);

    if (eigensolver.info() != Eigen::Success) 
        return false;
            
    matD1 = eigensolver.eigenvalues();
    matV1 = eigensolver.eigenvectors();

    if (matD1(0) < matD1(1) * threshold) {
        return false;
    }

    float x0 = pointWorld.x;
    float y0 = pointWorld.y;
    float z0 = pointWorld.z;
    float x1 = cx + 0.1 * matV1(0, 0);
    float y1 = cy + 0.1 * matV1(0, 1);
    float z1 = cz + 0.1 * matV1(0, 2);
    float x2 = cx - 0.1 * matV1(0, 0);
    float y2 = cy - 0.1 * matV1(0, 1);
    float z2 = cz - 0.1 * matV1(0, 2);

    float a012 = sqrt(((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) * ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                 + ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) * ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                 + ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)) * ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)));

    float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));

    float la = ((y1 - y2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
               + (z1 - z2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))) / a012 / l12;

    float lb = -((x1 - x2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
               - (z1 - z2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

    float lc = -((x1 - x2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
               + (y1 - y2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

    float ld2 = a012 / l12;

    pca_result(0) = la; pca_result(1) = lb; pca_result(2) = lc; pca_result(3) = ld2; 

    return true;
}

template<typename T>
void pointAssociateToMap(T const * const pi, T * const po, const State& state) {
    Eigen::Matrix4d pose_world = state.pose;
    po->x = pose_world(0 ,0) * pi->x + pose_world(0, 1) * pi->y + pose_world(0, 2) * pi->z + pose_world(0, 3);
    po->y = pose_world(1, 0) * pi->x + pose_world(1, 1) * pi->y + pose_world(1, 2) * pi->z + pose_world(1, 3);
    po->z = pose_world(2, 0) * pi->x + pose_world(2, 1) * pi->y + pose_world(2, 2) * pi->z + pose_world(2, 3);
    po->intensity = pi->intensity;
}

template<typename T>
void state2RPYXYZ(T *rpyxyz, const State& state) {
    Eigen::Matrix4d pose = state.pose;

    Eigen::Matrix3d rot = pose.block(0, 0, 3, 3);
    Eigen::Matrix<double, 3, 1> rpy = RotMtoEuler(rot);
    rpyxyz[0] = T(rpy(0));
    rpyxyz[1] = T(rpy(1));
    rpyxyz[2] = T(rpy(2));

    Eigen::Vector3d trans = pose.block(0, 3, 3, 1);
    rpyxyz[3] = T(trans(0));
    rpyxyz[4] = T(trans(1));
    rpyxyz[5] = T(trans(2));
}

template<typename T>
void RPYXYZ2State(T *rpyxyz, State& state) {
    Eigen::Matrix3d rot = Exp(double(rpyxyz[0]), double(rpyxyz[1]), double(rpyxyz[2]));
    Eigen::Matrix<double, 3, 1> xyz((rpyxyz[3]), (rpyxyz[4]), (rpyxyz[5]));

    state.pose.block(0, 0, 3, 3) = rot;
    state.pose.block(0, 3, 3, 1) = xyz;
}

void tree2Cloud(KD_TREE<PointType> *tree, pcl::PointCloud<PointType>::Ptr& cloud) {
    KD_TREE<PointType>::PointVector().swap(tree->PCL_Storage);  
    tree->flatten(tree->Root_Node, tree->PCL_Storage, NOT_RECORD);
    cloud->points = tree->PCL_Storage;
}

#endif
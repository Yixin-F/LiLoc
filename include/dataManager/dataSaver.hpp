#ifndef _DATASAVER_HPP_
#define _DATASAVER_HPP_

#include <rosbag/bag.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_msgs/TFMessage.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/dataset.h>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/search/impl/search.hpp>

#include <fstream>
#include <iostream>

using namespace std;
using namespace gtsam;

using PointT = pcl::PointXYZI;

namespace dataManager {

class DataSaver {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    DataSaver() { }

    ~DataSaver() { }

    DataSaver(string _base_dir, string _sequence_name) {
        this->base_dir = _base_dir;
        this->sequence_name = _sequence_name;

        if (_base_dir.back() != '/') {
            _base_dir.append("/");
        }
        save_directory = _base_dir + sequence_name + '/';
        std::cout << "SAVE DIR:" << save_directory << std::endl;

        auto unused = system((std::string("exec rm -r ") + save_directory).c_str());
        unused = system((std::string("mkdir -p ") + save_directory).c_str());
    }

    void setDir(string _base_dir, string _sequence_name) {
        this->base_dir = _base_dir;
        this->sequence_name = _sequence_name;

        if (_base_dir.back() != '/') {
            _base_dir.append("/");
        }
        save_directory = _base_dir + sequence_name + '/';

        auto unused = system((std::string("exec rm -r ") + save_directory).c_str());
        unused = system((std::string("mkdir -p ") + save_directory).c_str());
    }

    void setConfigDir(string _config_dir) {
        if (_config_dir.back() != '/') {
            _config_dir.append("/");
        }
        this->config_directory = _config_dir;
    }

    void setExtrinc(bool _use_imu, Eigen::Vector3d _t_body_sensor, Eigen::Quaterniond _q_body_sensor) {
        this->use_imu_frame = _use_imu;
        this->t_body_sensor = _t_body_sensor;
        this->q_body_sensor = _q_body_sensor;
    }

    void saveOptimizedVerticesKITTI(gtsam::Values _estimates) {
        std::fstream stream(save_directory + "optimized_odom_kitti.txt", std::fstream::out);
        stream.precision(15);
        for (const auto &key_value : _estimates) {
            auto p = dynamic_cast<const GenericValue<Pose3> *>(&key_value.value);
            if (!p) continue;

            const Pose3 &pose = p->value();

            Point3 t = pose.translation();
            Rot3 R = pose.rotation();
            auto col1 = R.column(1);  // Point3
            auto col2 = R.column(2);  // Point3
            auto col3 = R.column(3);  // Point3

            stream << col1.x() << " " << col2.x() << " " << col3.x() << " " << t.x()
                   << " " << col1.y() << " " << col2.y() << " " << col3.y() << " "
                   << t.y() << " " << col1.z() << " " << col2.z() << " " << col3.z()
                   << " " << t.z() << std::endl;
        }
    }

    void saveTimes(vector<double> _keyframeTimes) {
        if (_keyframeTimes.empty()) {
            //    LOG(ERROR) << "EMPTY KEYFRAME TIMES!";
            return;
        }
        this->keyframeTimes = _keyframeTimes;
        std::fstream pgTimeSaveStream(save_directory + "times.txt",
                                  std::fstream::out);
        pgTimeSaveStream.precision(15);
        // save timestamp
        for (auto const timestamp : keyframeTimes) {
            pgTimeSaveStream << timestamp << std::endl;
        }
        pgTimeSaveStream.close();
    }

    void saveOptimizedVerticesTUM(gtsam::Values _estimates) {
        std::fstream stream(save_directory + "optimized_odom_tum.txt", std::fstream::out);
        stream.precision(15);
        for (int i = 0; i < _estimates.size(); i++) {
            auto &pose = _estimates.at(i).cast<gtsam::Pose3>();
            gtsam::Point3 p = pose.translation();
            gtsam::Quaternion q = pose.rotation().toQuaternion();
            stream << keyframeTimes.at(i) << " " << p.x() << " " << p.y() << " "
                   << p.z() << " " << q.x() << " " << q.y() << " " << q.z() << " "
                   << q.w() << std::endl;
        }
    }

    void saveGraphGtsam(gtsam::NonlinearFactorGraph gtSAMgraph, gtsam::ISAM2 *isam, gtsam::Values isamCurrentEstimate) {
        gtsam::writeG2o(gtSAMgraph, isamCurrentEstimate, save_directory + "pose_graph.g2o");
        gtsam::writeG2o(isam->getFactorsUnsafe(), isamCurrentEstimate, save_directory + "pose_graph.g2o");
    }

    void saveGraph(std::vector<nav_msgs::Odometry> keyframePosesOdom) {
        std::fstream g2o_outfile(save_directory + "odom.g2o", std::fstream::out);
        g2o_outfile.precision(15);
        // g2o_outfile << std::fixed << std::setprecision(9);

        for (int i = 0; i < keyframePosesOdom.size(); i++) {
            nav_msgs::Odometry odometry = keyframePosesOdom.at(i);
            double time = odometry.header.stamp.toSec();

            g2o_outfile << "VERTEX_SE3:QUAT " << std::to_string(i) << " ";
            g2o_outfile << odometry.pose.pose.position.x << " ";
            g2o_outfile << odometry.pose.pose.position.y << " ";
            g2o_outfile << odometry.pose.pose.position.z << " ";
            g2o_outfile << odometry.pose.pose.orientation.x << " ";
            g2o_outfile << odometry.pose.pose.orientation.y << " ";
            g2o_outfile << odometry.pose.pose.orientation.z << " ";
            g2o_outfile << odometry.pose.pose.orientation.w << std::endl;
        }
        g2o_outfile.close();
    }

    void saveResultBag(std::vector<nav_msgs::Odometry> allOdometryVec, std::vector<sensor_msgs::PointCloud2> allResVec) {
        rosbag::Bag result_bag;
        result_bag.open(save_directory + sequence_name + "_result.bag",
                    rosbag::bagmode::Write);

        for (int i = 0; i < allOdometryVec.size(); i++) {
            nav_msgs::Odometry _laserOdometry = allOdometryVec.at(i);
            result_bag.write("pgo_odometry", _laserOdometry.header.stamp, _laserOdometry);
        }

        for (int i = 0; i < allResVec.size(); i++) {
            sensor_msgs::PointCloud2 _laserCloudFullRes = allResVec.at(i);
            result_bag.write("cloud_deskewed", _laserCloudFullRes.header.stamp, _laserCloudFullRes);
        }
        result_bag.close();
    }

    void saveResultBag(std::vector<nav_msgs::Odometry> allOdometryVec, std::vector<sensor_msgs::PointCloud2> allResVec, std::vector<geometry_msgs::TransformStamped> trans_vec) {
        rosbag::Bag result_bag;
        result_bag.open(save_directory + sequence_name + "_result.bag", rosbag::bagmode::Write);

        tf2_msgs::TFMessage tf_message;
        for (int i = 0; i < allOdometryVec.size(); i++) {
            nav_msgs::Odometry _laserOdometry = allOdometryVec.at(i);
            result_bag.write("pgo_odometry", _laserOdometry.header.stamp, _laserOdometry);

            sensor_msgs::PointCloud2 _laserCloudFullRes = allResVec.at(i);
            result_bag.write("cloud_deskewed", _laserCloudFullRes.header.stamp, _laserCloudFullRes);

            geometry_msgs::TransformStamped transform_stamped = trans_vec.at(i);
            tf_message.transforms.push_back(transform_stamped);
            result_bag.write("tf", transform_stamped.header.stamp, tf_message);
        }
        result_bag.close();
    }

    void savePointCloudMap(std::vector<nav_msgs::Odometry> allOdometryVec, std::vector<pcl::PointCloud<PointT>::Ptr> allResVec) {
        std::cout << "odom and cloud size: " << allOdometryVec.size() << ", " << allResVec.size();

        int odom_size = std::min(allOdometryVec.size(), allResVec.size());

        if (allOdometryVec.size() != allResVec.size()) {
            std::cout << " point cloud size do not equal to odom size!";
            return;
        }

        pcl::PointCloud<PointT>::Ptr laserCloudRaw(new pcl::PointCloud<PointT>());  // giseop
        pcl::PointCloud<PointT>::Ptr laserCloudTrans(new pcl::PointCloud<PointT>());  // giseop
        pcl::PointCloud<PointT>::Ptr globalmap(new pcl::PointCloud<PointT>());  // giseop
        for (int i = 0; i < odom_size; ++i) {
            nav_msgs::Odometry odom = allOdometryVec.at(i);
            laserCloudRaw = allResVec.at(i);

            Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
            transform.rotate(Eigen::Quaterniond(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,
                                                odom.pose.pose.orientation.y, odom.pose.pose.orientation.z));
            transform.pretranslate(Eigen::Vector3d(odom.pose.pose.position.x,
                                                   odom.pose.pose.position.y,
                                                   odom.pose.pose.position.z));

            pcl::transformPointCloud(*laserCloudRaw, *laserCloudTrans, transform.matrix());
            *globalmap += *laserCloudTrans;
        }

        // save point cloud in lidar frame
        // if you want to save it in body frame(imu)
        // i will update it later
        if (!globalmap->empty()) {
            globalmap->width = globalmap->points.size();
            globalmap->height = 1;
            globalmap->is_dense = false;

            try {
                pcl::io::savePCDFileASCII(save_directory + "global_map_lidar.pcd",
                                      *globalmap);
                cout << "current scan saved to : " << save_directory << ", " << globalmap->points.size() << endl;
            } 
            catch (std::exception e) {
                ROS_ERROR_STREAM("SAVE PCD ERROR :" <<  globalmap->points.size());
            }

            // all cloud must rotate to body axis
            if (use_imu_frame) {
                for (int j = 0; j < globalmap->points.size(); ++j) {
                    PointT &pt = globalmap->points.at(j);
                    Eigen::Vector3d translation(pt.x, pt.y, pt.z);
                    translation = q_body_sensor * translation + t_body_sensor;

                    pt.x = translation[0];
                    pt.y = translation[1];
                    pt.z = translation[2];
                }
                try {
                    pcl::io::savePCDFileASCII(save_directory + "globalmap_imu.pcd", *globalmap);
                    cout << "current scan saved to : " << save_directory << ", " << globalmap->points.size() << endl;
                } 
                catch (std::exception e) {
                    ROS_ERROR_STREAM("SAVE PCD ERROR :" <<  globalmap->points.size());
                }
            }
        } 
        else
            std::cout << "EMPTY POINT CLOUD";
    }

    void savePointCloudMap(pcl::PointCloud<PointT> &cloud_ptr) {
        if (cloud_ptr.empty()) {
            std::cout << "empty global map cloud!" << std::endl;
            return;
        }
        try {
            pcl::io::savePCDFileASCII(save_directory + "globalmap_lidar_feature.pcd", cloud_ptr);
        } 
        catch (pcl::IOException) {
            std::cout << "  save map failed!!! " << cloud_ptr.size() << std::endl;

        }
    }

private:
    string base_dir, sequence_name;
    string save_directory, config_directory;

    vector<string> configParameter;

    bool use_imu_frame = false;
    Eigen::Quaterniond q_body_sensor;
    Eigen::Vector3d t_body_sensor;

    vector<double> keyframeTimes;
};

}

#endif  // _DATASAVER_H_

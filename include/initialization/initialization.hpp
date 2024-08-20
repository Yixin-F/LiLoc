#ifndef _INITIALIZATION_HPP_
#define _INITIALIZATION_HPP_

#include "../utility.h"
#include "../sophus/so3.hpp"

class Initialization : public ParamServer {
public:
    bool lio_init = false;
    bool relo_init = false;

    SensorMeasurement initMeasurement;
    State initLioState;
    State initReloState;

    const int max_init_imu_count = 20;
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> imu_init_buff;

    Eigen::Vector3d mean_acc_ = Eigen::Vector3d(0, 0, -1.0);
    Eigen::Vector3d mean_gyr_ = Eigen::Vector3d(0, 0, 0);
    double acc_cov{1.0};
    double gyr_cov{1.0};

    ~Initialization() { }
    Initialization() { }

    bool initStaticLio(const SensorMeasurement &initMeasurement, 
                       KD_TREE<PointType> *edge_tree_lio_, KD_TREE<PointType> *surf_tree_lio_)
    {
        if (lio_init) {
            return false;
        }

        const auto& acc = initMeasurement.imu_buff_.front().linear_acceleration;
        const auto& gyr = initMeasurement.imu_buff_.front().angular_velocity;
        imu_init_buff.emplace_back(Eigen::Vector3d(acc.x, acc.y, acc.z), Eigen::Vector3d(gyr.x, gyr.y, gyr.z));

        for (const auto& imu_msg : initMeasurement.imu_buff_) {
            Eigen::Vector3d acc(imu_msg.linear_acceleration.x,
                                imu_msg.linear_acceleration.y,
                                imu_msg.linear_acceleration.z);
            Eigen::Vector3d gyr(imu_msg.angular_velocity.x,
                                imu_msg.angular_velocity.y,
                                imu_msg.angular_velocity.z);

            imu_init_buff.emplace_back(acc, gyr);
        }

        if (imu_init_buff.size() < max_init_imu_count) {
            return false;
        }

        Eigen::Vector3d acc_cov, gyr_cov;
        ComputeMeanAndCovDiag(
            imu_init_buff,
            mean_acc_,
            acc_cov,
            [](const std::pair<Eigen::Vector3d, Eigen::Vector3d>& imu_data) {
            return imu_data.first;
        });
        ComputeMeanAndCovDiag(
            imu_init_buff,
            mean_gyr_,
            gyr_cov,
            [](const std::pair<Eigen::Vector3d, Eigen::Vector3d>& imu_data) {
            return imu_data.second;
        });

        // Compute initial attitude via Schmidt orthogonalization.
        // The roll and pitch are aligned with the direction of gravity, but the yaw is random.
        Eigen::Vector3d z_axis = mean_acc_.normalized();
        Eigen::Vector3d e1(1, 0, 0);
        Eigen::Vector3d x_axis = e1 - z_axis * z_axis.transpose() * e1;
        x_axis.normalize();
        Eigen::Vector3d y_axis = Sophus::SO3d::hat(z_axis).matrix() * x_axis;
        y_axis.normalize();

        Eigen::Matrix3d init_R;
        init_R.block<3, 1>(0, 0) = x_axis;
        init_R.block<3, 1>(0, 1) = y_axis;
        init_R.block<3, 1>(0, 2) = z_axis;
        Eigen::Quaterniond init_q(init_R);
        initLioState.pose.block<3, 3>(0, 0) = init_q.normalized().toRotationMatrix().transpose();

        Eigen::Vector3d init_ba = Eigen::Vector3d::Zero();
        ComputeMeanAndCovDiag(
            imu_init_buff,
            init_ba,
            acc_cov,
            [this](const std::pair<Eigen::Vector3d, Eigen::Vector3d>& imu_data) {
            Eigen::Vector3d temp_ba = imu_data.first - initLioState.pose.block<3, 3>(0, 0).transpose() * Eigen::Vector3d(0.0, 0.0, imuGravity);
            return temp_ba;
        });

        initLioState.pose.block<3, 1>(0, 3).setZero();
        // init velocity
        initLioState.vel.setZero();
        // init bg
        initLioState.bg = mean_gyr_;
        // init ba
        initLioState.ba = init_ba;

        std::cout << "imu static, mean_acc_: " << mean_acc_.transpose()
                  << " init_ba: " << init_ba.transpose() << ", ori pose: " << "\n"
                  << initLioState.pose.block<3, 3>(0, 0) << std::endl;
        
        ext_state.pose = initLioState.pose * ext_state.pose;  // FIXME: check

        pcl::PointCloud<PointType>::Ptr edge_world(new pcl::PointCloud<PointType>());
        size_t edge_size = initMeasurement.edge_ptr_->size();
        if (edge_tree_lio_->Root_Node == nullptr) {
            if (edge_size > 5) {
                edge_tree_lio_->set_downsample_param(mappingCornerLeafSize);
                for (size_t i = 0; i < edge_size; i++) {
                    PointType pointworld;
                    PointType pointbody = initMeasurement.edge_ptr_->points[i];
                    pointAssociateToMap(&pointbody, &pointworld, ext_state);
                    edge_world->push_back(pointworld);
                }
                edge_tree_lio_->Build(edge_world->points);
            }
        }

        pcl::PointCloud<PointType>::Ptr surf_world(new pcl::PointCloud<PointType>());
        size_t surf_size = initMeasurement.plane_ptr_->size();
        if (surf_tree_lio_->Root_Node == nullptr) {
            if (surf_size > 5) {
                surf_tree_lio_->set_downsample_param(mappingCornerLeafSize);
                for (size_t i = 0; i < surf_size; i++) {
                    PointType pointworld;
                    PointType pointbody = initMeasurement.plane_ptr_->points[i];
                    pointAssociateToMap(&pointbody, &pointworld, ext_state);
                    surf_world->push_back(pointworld);
                }
                surf_tree_lio_->Build(surf_world->points);
            }
        }

        lio_init = true;

        return true;
    }

    bool initDynamicRelo(const SensorMeasurement &initMeasurement) {
        if (!lio_init || relo_init) {
            return false;
        }

        // TODO: like gim in hkust

        lio_init = true;
        relo_init = true;

        return true;
    }

    

};

#endif
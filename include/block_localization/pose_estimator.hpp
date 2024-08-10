#ifndef POSE_ESTIMATOR_HPP
#define POSE_ESTIMATOR_HPP

#include <memory>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pclomp/ndt_omp.h>
#include <pcl/filters/voxel_grid.h>

#include <block_localization/pose_system.hpp>
#include <kkl/alg/unscented_kalman_filter.hpp>

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
namespace block_localization {

/**
 * @brief scan matching-based pose estimator
 */
    class PoseEstimator {
    public:
        Eigen::Matrix4d trans_delta;
        Eigen::Matrix4d trans;
        Eigen::Matrix4d trans_pre=Eigen::Matrix4d::Identity();



        typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VectorXt;
        using PointT = pcl::PointXYZI;

        /**
         * @brief constructor
         * @param registration        registration method
         * @param stamp               timestamp
         * @param pos                 initial position
         * @param quat                initial orientation
         * @param cool_time_duration  during "cool time", prediction is not performed
         */
        PoseEstimator(pcl::Registration<PointT, PointT>::Ptr& registration, const ros::Time& stamp, const Eigen::Vector3d& pos, const Eigen::Quaterniond& quat_data, double cool_time_duration = 1.0)
                : init_stamp(stamp),
                  registration(registration),
                  cool_time_duration(cool_time_duration)
        {
            process_noise = Eigen::MatrixXd::Identity(16, 16);
            process_noise.middleRows(0, 3) *= 1.0;// middleRows(a,b): from row a to row (a+b-1)
            process_noise.middleRows(3, 3) *= 1.0;
            process_noise.middleRows(6, 4) *= 0.5;
            process_noise.middleRows(10, 3) *= 1e-6;
            process_noise.middleRows(13, 3) *= 1e-6;

            Eigen::MatrixXd measurement_noise = Eigen::MatrixXd::Identity(7, 7);
            measurement_noise.middleRows(0, 3) *= 0.01;
            measurement_noise.middleRows(3, 4) *= 0.001;

            Eigen::VectorXd mean(16);
            mean.middleRows(0, 3) = pos;
            mean.middleRows(3, 3).setZero();
            mean.middleRows(6, 4) = Eigen::Vector4d(quat_data.w(), quat_data.x(), quat_data.y(), quat_data.z());
            mean.middleRows(10, 3).setZero();
            mean.middleRows(13, 3).setZero();
            state=mean;
            std::cout<<"init_state:"<<state(0)<<" "<<state(1)<<" "<<state(2)<<std::endl;
            Eigen::MatrixXd cov = Eigen::MatrixXd::Identity(16, 16) * 0.01;

        }

        /**
         * @brief predict
         * @param stamp    timestamp
         * @param acc      acceleration
         * @param gyro     angular velocity
         */
        void predict(const ros::Time& stamp, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyro,PoseSystem &system) {
            if((stamp - init_stamp).toSec() < cool_time_duration || prev_stamp.is_zero() || prev_stamp == stamp) {
                prev_stamp = stamp;
                return ;
            }
//            this->system=system;
            double dt = (stamp - prev_stamp).toSec();
            prev_stamp = stamp;
            Eigen::VectorXd control(6);
            control.head<3>() = acc;
            control.tail<3>() = gyro;
            Eigen::VectorXd pre_state=state;
            std::cout<<"pre_state:"<<pre_state(0)<<" "<<pre_state(1)<<" "<<pre_state(2)<<std::endl;
            std::cout<<"control:"<<control(0)<<" "<<control(1)<<" "<<control(2)<<std::endl;
//            state = system.f(pre_state, control);
            std::cout<<"post_state:"<<state(0)<<" "<<state(1)<<" "<<state(2)<<std::endl;

        }
        void update(const gtsam::NavState &latestState,const gtsam::imuBias::ConstantBias &latestBias)
        {
            Eigen::Quaterniond stateQuat(latestState.quaternion().w(),latestState.quaternion().x(),latestState.quaternion().y(),latestState.quaternion().z());
            stateQuat.normalize();
            this->state.middleRows(6,4) = Eigen::Vector4d(stateQuat.w(),stateQuat.x(),stateQuat.y(),stateQuat.z());
            this->state.middleRows(0,3) = Eigen::Vector3d(latestState.position().x(),latestState.position().y(),latestState.position().z());
            this->state.middleRows(3,3) = Eigen::Vector3d(latestState.velocity().x(),latestState.velocity().y(),latestState.velocity().z());
            this->state.middleRows(10,3)= Eigen::Vector3d(latestBias.accelerometer().x(),latestBias.accelerometer().y(),latestBias.accelerometer().z());
            this->state.middleRows(13,3)= Eigen::Vector3d(latestBias.gyroscope().x(),latestBias.gyroscope().y(),latestBias.gyroscope().z());

            trans_pre.block<3, 3>(0, 0) = this->quat().toRotationMatrix();
            trans_pre.block<3, 1>(0, 3) = this->pos();
        }
        /**
         * @brief correct
         * @param cloud   input cloud
         * @return cloud aligned to the globalmap
         */
        pcl::PointCloud<PointT>::Ptr correct(const pcl::PointCloud<PointT>::ConstPtr& cloud) {

            Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();
            init_guess.block<3, 3>(0, 0) = this->quat().cast<float>().toRotationMatrix();
            init_guess.block<3, 1>(0, 3) = this->pos().cast<float>();

            pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());
            registration->setInputSource(cloud);
            registration->align(*aligned, init_guess);


            Eigen::Matrix4f trans_temp;
            trans_temp = registration->getFinalTransformation();
            trans=trans_temp.cast<double>();
//            std::cout<<"trans:"<<trans(0)<<" "<<trans(1)<<" "<<trans(2)<<std::endl;

            this->state.middleRows(0,3)=trans.block<3, 1>(0, 3);
            Eigen::Quaterniond update_quat = Eigen::Quaterniond(trans.block<3, 3>(0, 0)).normalized();
            this->state.middleRows(6,4)=Eigen::Vector4d(update_quat.w(), update_quat.x(), update_quat.y(), update_quat.z());

            trans_delta=getDeltaTrans(trans_pre,trans);
            Eigen::Vector3d p_print = trans_delta.block<3, 1>(0, 3);//used to test incremental odometry


//            Eigen::Vector3f p = trans.block<3, 1>(0, 3);
//            Eigen::Quaternionf q(trans.block<3, 3>(0, 0));
//
//            if(quat().coeffs().dot(q.coeffs()) < 0.0f) {
//                q.coeffs() *= -1.0f;
//            }

            return aligned;
        }

        /* getters */
//        Eigen::Vector3f pos() const {
//            return Eigen::Vector3f(ukf->mean[0], ukf->mean[1], ukf->mean[2]);
//        }
//        Eigen::Vector3d pos() const {
//            Eigen::Vector3d p_tre = trans_pre.block<3, 1>(0, 3);
//            return Eigen::Vector3d(p_tre(0), p_tre(1), p_tre(2));
//        }
        Eigen::Vector3d pos() const {
            return Eigen::Vector3d(state(0),state(1),state(2));
        }

//        Eigen::Vector3f vel() const {
//            return Eigen::Vector3f(ukf->mean[3], ukf->mean[4], ukf->mean[5]);
//        }

//        Eigen::Quaternionf quat() const {
//            return Eigen::Quaternionf(ukf->mean[6], ukf->mean[7], ukf->mean[8], ukf->mean[9]).normalized();
//        }
//        Eigen::Quaterniond quat() const {
//            return Eigen::Quaterniond(trans_pre.block<3, 3>(0, 0)).normalized();
//        }
        Eigen::Quaterniond quat() const {
            return Eigen::Quaterniond(state(6),state(7),state(8),state(9)).normalized();
        }

//        Eigen::Matrix4f matrix() const {
//            Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
//            m.block<3, 3>(0, 0) = quat().toRotationMatrix();
//            m.block<3, 1>(0, 3) = pos();
//            return m;
//        }

        Eigen::Matrix4d getDeltaTrans(Eigen::Matrix4d trans_pre,Eigen::Matrix4d trans_cur){
            Eigen::Vector3d p_tre = trans_pre.block<3, 1>(0, 3);
            Eigen::Quaterniond q_pre(trans_pre.block<3, 3>(0, 0));
            Eigen::Vector3d p_cur = trans_cur.block<3, 1>(0, 3);
            Eigen::Quaterniond q_cur(trans_cur.block<3, 3>(0, 0));
            Eigen::Matrix4d delta=Eigen::Matrix4d::Identity();
            delta.block<3,1>(0,3)=p_cur-p_tre;
            delta.block<3, 3>(0, 0)=(q_cur*q_pre.inverse()).toRotationMatrix();
            return delta;
        }

    private:
        Eigen::VectorXd state;
        ros::Time init_stamp;         // when the estimator was initialized
        ros::Time prev_stamp;         // when the estimator was updated last time
        double cool_time_duration;
//        PoseSystem system;

        Eigen::MatrixXd process_noise;
//        std::unique_ptr<kkl::alg::UnscentedKalmanFilterX<float, PoseSystem>> ukf;
        pcl::Registration<PointT, PointT>::Ptr registration;

    };

}

#endif // POSE_ESTIMATOR_HPP

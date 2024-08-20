#ifndef _GTSAM_HPP_ 
#define _GTSAM_HPP_

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

#include "../utility.h"

using gtsam::symbol_shorthand::B;  // Bias  (ax, ay, az, gx, gy, gz)
using gtsam::symbol_shorthand::V;  // Vel   (xdot, ydot, zdot)
using gtsam::symbol_shorthand::X;  // Pose3 (x, y, z, r, p, y)

class GTSAM : public ParamServer {
public:
    bool initialized = false;
    
    gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorVelNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorBiasNoise;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise2;
    gtsam::Vector noiseModelBetweenBias;

    gtsam::PreintegratedImuMeasurements *imuIntegratorImu_;

    gtsam::Pose3 prevPose_;
    gtsam::Vector3 prevVel_;
    gtsam::NavState prevState_;
    gtsam::imuBias::ConstantBias prevBias_;

    gtsam::NavState prevStateOdom;
    gtsam::imuBias::ConstantBias prevBiasOdom;

    gtsam::ISAM2 optimizer;
    gtsam::NonlinearFactorGraph graphFactors;
    gtsam::Values graphValues;

    State imu_state;
    State cur_state;

    gtsam::Pose3 imu2Lidar = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(-extTrans.x(), -extTrans.y(), -extTrans.z()));
    gtsam::Pose3 lidar2Imu = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(extTrans.x(), extTrans.y(), extTrans.z()));

    int key = 1;

    ~GTSAM() { }
    GTSAM() { 
        reset();
    }

    bool initialize(const State& init_state) {
        if (initialized) {
            return false;
        }

        boost::shared_ptr<gtsam::PreintegrationParams> p = gtsam::PreintegrationParams::MakeSharedU(imuGravity);
        p->accelerometerCovariance = gtsam::Matrix33::Identity(3, 3) * pow(imuAccNoise, 2);  // acc white noise in continuous
        p->gyroscopeCovariance = gtsam::Matrix33::Identity(3, 3) * pow(imuGyrNoise, 2);  // gyro white noise in continuous
        p->integrationCovariance = gtsam::Matrix33::Identity(3, 3) * pow(1e-4, 2);  // error committed in integrating position from velocities
        gtsam::imuBias::ConstantBias prior_imu_bias((gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());;  // assume zero initial bias

        gtsam::PreintegratedImuMeasurements *imuIntegrator(new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias));
        imuIntegratorImu_ = std::move(imuIntegrator);    

        priorPoseNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished());  // rad, rad, rad, m, m, m
        priorVelNoise = gtsam::noiseModel::Isotropic::Sigma(3, 1e4);  // m/s
        priorBiasNoise = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3);  // 1e-2 ~ 1e-3 seems to be good
        correctionNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1).finished());  // rad, rad, rad, m, m, m
        correctionNoise2 = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1, 1, 1, 1, 1, 1).finished());  // rad, rad, rad, m, m, m
        noiseModelBetweenBias = (gtsam::Vector(6) << imuAccBiasN, imuAccBiasN, imuAccBiasN, imuGyrBiasN, imuGyrBiasN, imuGyrBiasN).finished();

        // initial pose
        Eigen::Matrix4d init_pose = init_state.pose;
        Eigen::Vector4d init_quat = rotationToQuaternion(init_pose.block(0, 0, 3, 3));
        Eigen::Vector3d init_trans = init_pose.block(0, 3, 3, 1);
        prevPose_ = gtsam::Pose3(gtsam::Rot3(init_quat(3), init_quat(0), init_quat(1), init_quat(2)), gtsam::Point3(init_trans(0), init_trans(1), init_trans(2)));
        gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, priorPoseNoise);
        graphFactors.add(priorPose);

        // initial velocity
        Eigen::Vector3d init_vel = init_state.vel;
        prevVel_ = gtsam::Vector3(init_vel(0), init_vel(1), init_vel(2));
        gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, priorVelNoise);
        graphFactors.add(priorVel);

        // initial bias
        Eigen::Vector3d init_ba = init_state.ba;
        Eigen::Vector3d init_bg = init_state.bg;
        gtsam::Vector3 accelBias(init_ba(0), init_ba(1), init_ba(2));
        gtsam::Vector3 gyroBias(init_bg(0), init_bg(1), init_bg(2));
        prevBias_ = gtsam::imuBias::ConstantBias(accelBias, gyroBias);
        gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, priorBiasNoise);
        graphFactors.add(priorBias);

        // add values
        graphValues.insert(X(0), prevPose_);
        graphValues.insert(V(0), prevVel_);
        graphValues.insert(B(0), prevBias_);

        // optimize once
        optimizer.update(graphFactors, graphValues);
        graphFactors.resize(0);
        graphValues.clear();

        imuIntegratorImu_->resetIntegrationAndSetBias(prevBias_);

        initialized = true;

        return true;
    }

    void addImuFactor(const SensorMeasurement& mea) {
        static double last_imu_time = -1;

        // integrate imu data
        for (const auto& imu : mea.imu_buff_) {
            sensor_msgs::Imu thisImu = imu;
            double imuTime = ROS_TIME(&thisImu);

            double dt = (last_imu_time < 0) ? (1.0 / imuFrequence) : (imuTime - last_imu_time);

            if (dt <= 0) { // FIXME: bug, because the end and front imu is the same data
                continue;
            }

            imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(thisImu.linear_acceleration.x, thisImu.linear_acceleration.y, thisImu.linear_acceleration.z),
                                                    gtsam::Vector3(thisImu.angular_velocity.x, thisImu.angular_velocity.y, thisImu.angular_velocity.z), dt);
            last_imu_time = imuTime;
        }

        // add imu factor to graph
        const gtsam::PreintegratedImuMeasurements &preint_imu = dynamic_cast<const gtsam::PreintegratedImuMeasurements &>(*imuIntegratorImu_);
        gtsam::ImuFactor imu_factor(X(key - 1), V(key - 1), X(key), V(key), B(key - 1), preint_imu);
        graphFactors.add(imu_factor);

        // add imu bias between factor
        graphFactors.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(B(key - 1), B(key), 
                         gtsam::imuBias::ConstantBias(), gtsam::noiseModel::Diagonal::Sigmas(
                         sqrt(imuIntegratorImu_->deltaTij()) * noiseModelBetweenBias)));
        
        // insert predicted values
        gtsam::NavState propState_ = imuIntegratorImu_->predict(prevState_, prevBias_);
        graphValues.insert(X(key), propState_.pose());
        graphValues.insert(V(key), propState_.v());
        graphValues.insert(B(key), prevBias_);

        imu_state = convertNavStateToState(propState_, prevBias_);
    }

    void addLidarFactor(const gtsam::Pose3& lidarPose, const bool &degenerate) {
        // add pose factor
        gtsam::Pose3 curPose = lidarPose;
        gtsam::PriorFactor<gtsam::Pose3> pose_factor(X(key), curPose, degenerate ? correctionNoise2 : correctionNoise);
        graphFactors.add(pose_factor);

        key ++;
    }

    void optimize(int iter) {
        optimizer.update(graphFactors, graphValues);
        while(--iter) {
            optimizer.update();
        }
        graphFactors.resize(0);
        graphValues.clear();

        gtsam::Values result = optimizer.calculateEstimate();
        prevPose_ = result.at<gtsam::Pose3>(X(key));
        prevVel_ = result.at<gtsam::Vector3>(V(key));
        prevState_ = gtsam::NavState(prevPose_, prevVel_);
        prevBias_ = result.at<gtsam::imuBias::ConstantBias>(B(key));

        imuIntegratorImu_->resetIntegrationAndSetBias(prevBias_);

        prevStateOdom = prevState_;
        prevBiasOdom = prevBias_;

        cur_state = convertNavStateToState(prevState_, prevBias_);

        key ++;
    }

    void reset() {
        gtsam::ISAM2Params optParameters;
        optParameters.relinearizeThreshold = 0.1;
        optParameters.relinearizeSkip = 1;
        optParameters.factorization = gtsam::ISAM2Params::QR;
        optimizer = gtsam::ISAM2(optParameters);

        gtsam::NonlinearFactorGraph newGraphFactors;
        graphFactors = newGraphFactors;

        gtsam::Values NewGraphValues;
        graphValues = NewGraphValues;

        initialized = false;
        key = 1;
    }

    bool failureDetection() {
        Eigen::Vector3f vel(prevVel_.x(), prevVel_.y(), prevVel_.z());
        if (vel.norm() > 30) {
            ROS_WARN("Large velocity, reset IMU-preintegration!");
            return true;
        }

        Eigen::Vector3f ba(prevBias_.accelerometer().x(), prevBias_.accelerometer().y(), prevBias_.accelerometer().z());
        Eigen::Vector3f bg(prevBias_.gyroscope().x(), prevBias_.gyroscope().y(), prevBias_.gyroscope().z());
        if (ba.norm() > 1.0 || bg.norm() > 1.0) {
            ROS_WARN("Large bias, reset IMU-preintegration!");
            return true;
        }

        return false;
    }

    State convertNavStateToState(const gtsam::NavState& navState, const gtsam::imuBias::ConstantBias& bias) {
        State state;

        state.pose.block<3, 3>(0, 0) = navState.attitude().matrix();
        state.pose.block<3, 1>(0, 3) = navState.position();

        state.vel = navState.velocity();

        state.ba = bias.accelerometer();
        state.bg = bias.gyroscope();

        return state;
    }
};

#endif
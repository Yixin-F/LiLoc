#include "utility.h"

#include "block_localization/cloud_info.h"

class LiLoc : public ParamServer {
private:
    std::mutex imu_mtx;
    std::mutex pose_mtx;

    ros::Subscriber subImu;
    ros::Subscriber subCloudInfo;
    ros::Subscriber subInitPose;

    ros::Publisher pubImuOdom;
    ros::Publisher pubLidarOdom;
    ros::Publisher pubLidarPath;

    bool initialized = false;

    gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorVelNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorBiasNoise;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise2;
    gtsam::Vector noiseModelBetweenBias;

    gtsam::PreintegratedImuMeasurements *imuIntegratorOpt_;
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

    gtsam::Pose3 imu2Lidar;
    gtsam::Pose3 lidar2Imu;

    int key = 1;

    std::deque<sensor_msgs::Imu> imuQueImu;
    std::deque<sensor_msgs::Imu> imuQueOpt;

    bool doneFirstOpt = false;

    double imu_timestamp;
    double cloud_timestamp;

public:
    ~LiLoc() { }

    LiLoc() {
        subImu = nh.subscribe(imuTopic, 2000, &LiLoc::imuHandler, this, ros::TransportHints().tcpNoDelay());
        subCloudInfo = nh.subscribe("/cloud_info", 5, &LiLoc::LiLocMainProcess, this, ros::TransportHints().tcpNoDelay());
        subInitPose = nh.subscribe("/initialpose", 8, &LiLoc::initialposeCB, this, ros::TransportHints().tcpNoDelay());

        pubImuOdom = nh.advertise<nav_msgs::Odometry>("/imu_incremental", 2000);
        pubLidarOdom = nh.advertise<nav_msgs::Odometry>("/lidar_odom", 2000);

        imu2Lidar = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(-extTrans.x(), -extTrans.y(), -extTrans.z()));
        lidar2Imu = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(extTrans.x(), extTrans.y(), extTrans.z()));

        boost::shared_ptr<gtsam::PreintegrationParams> p = gtsam::PreintegrationParams::MakeSharedU(imuGravity);
        p->accelerometerCovariance = gtsam::Matrix33::Identity(3, 3) * pow(imuAccNoise, 2);  // acc white noise in continuous
        p->gyroscopeCovariance = gtsam::Matrix33::Identity(3, 3) * pow(imuGyrNoise, 2);  // gyro white noise in continuous
        p->integrationCovariance = gtsam::Matrix33::Identity(3, 3) * pow(1e-4, 2);  // error committed in integrating position from velocities
        gtsam::imuBias::ConstantBias prior_imu_bias((gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());;  // assume zero initial bias

        imuIntegratorOpt_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias);
        imuIntegratorImu_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias);

        priorVelNoise = gtsam::noiseModel::Isotropic::Sigma(3, 1e4);  // m/s
        priorBiasNoise = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3);  // 1e-2 ~ 1e-3 seems to be good
        correctionNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1).finished());  // rad, rad, rad, m, m, m
        correctionNoise2 = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1, 1, 1, 1, 1, 1).finished());  // rad, rad, rad, m, m, m

        noiseModelBetweenBias = (gtsam::Vector(6) << imuAccBiasN, imuAccBiasN, imuAccBiasN, imuGyrBiasN, imuGyrBiasN, imuGyrBiasN).finished();

        reset();
    }

    void imuHandler(const sensor_msgs::Imu::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(imu_mtx);

        sensor_msgs::Imu thisImu = imuConverter(*msg);

        static double last_imu_timestamp = -1.0;
        static double last_imu_use_timestamp = -1.0;
        static sensor_msgs::Imu last_imu = thisImu;

        // parameters for EMA filter
        static double a = 0.8;
        static double b = 1.0 - a;

        imu_timestamp = thisImu.header.stamp.toSec();

        if (imu_timestamp < last_imu_timestamp) {
            ROS_WARN_STREAM("imu loop back, clear buffer");
            imuQueOpt.clear();
            imuQueImu.clear();
        }

        // EMA filter for accelerometer
        thisImu.linear_acceleration.x = thisImu.linear_acceleration.x * a + last_imu.linear_acceleration.x * b;
        thisImu.linear_acceleration.y = thisImu.linear_acceleration.y * a + last_imu.linear_acceleration.y * b;
        thisImu.linear_acceleration.z = thisImu.linear_acceleration.z * a + last_imu.linear_acceleration.z * b;

        imuQueOpt.push_back(thisImu);
        imuQueImu.push_back(thisImu);

        last_imu_timestamp = imu_timestamp;
        last_imu = thisImu;

        if (!doneFirstOpt) {
            return ;
        }
        
        double imuTime = ROS_TIME(&thisImu);
        double dt = (last_imu_use_timestamp < 0) ? (1.0 / imuFrequence) : (imuTime - last_imu_use_timestamp);
        if (dt < 1e-16) dt = 1e-16;

        last_imu_use_timestamp = imuTime;

        // integrate this single imu message
        const auto& acc = thisImu.linear_acceleration;
        const auto& gyro = thisImu.angular_velocity;
        imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(acc.x, acc.y, acc.z), gtsam::Vector3(gyro.x, gyro.y, gyro.z), dt);
        
        // predict odometry
        gtsam::NavState currentState = imuIntegratorImu_->predict(prevStateOdom, prevBiasOdom);

        // publish odometry
        nav_msgs::Odometry odom;
        odom.header.stamp = thisImu.header.stamp;

        odom.header.frame_id = "map";
        odom.child_frame_id = "odom_imu";

        // transform imu pose to lidar
        gtsam::Pose3 imuPose = gtsam::Pose3(currentState.quaternion(), currentState.position());
        gtsam::Pose3 odomPose = imuPose.compose(imu2Lidar);

        odom.pose.pose.position.x = odomPose.translation().x();
        odom.pose.pose.position.y = odomPose.translation().y();
        odom.pose.pose.position.z = odomPose.translation().z();
        odom.pose.pose.orientation.x = odomPose.rotation().toQuaternion().x();
        odom.pose.pose.orientation.y = odomPose.rotation().toQuaternion().y();
        odom.pose.pose.orientation.z = odomPose.rotation().toQuaternion().z();
        odom.pose.pose.orientation.w = odomPose.rotation().toQuaternion().w();
        
        odom.twist.twist.linear.x = currentState.velocity().x();
        odom.twist.twist.linear.y = currentState.velocity().y();
        odom.twist.twist.linear.z = currentState.velocity().z();
        odom.twist.twist.angular.x = thisImu.angular_velocity.x + prevBiasOdom.gyroscope().x();
        odom.twist.twist.angular.y = thisImu.angular_velocity.y + prevBiasOdom.gyroscope().y();
        odom.twist.twist.angular.z = thisImu.angular_velocity.z + prevBiasOdom.gyroscope().z();
        pubImuOdom.publish(odom);
    }

    void initialposeCB(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_msg) {

    }

    void initialize(const State& init_state) {
        if (initialized) {
            return ;
        }

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

        imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);

        initialized = true;
    }

    void LiLocMainProcess(const block_localization::cloud_infoConstPtr& msg) {
        sensor_msgs::PointCloud2 points_msg = msg->cloud_deskewed;

        cloud_timestamp = ROS_TIME(&points_msg);

        if (!initialized) {
            // TODO: init
            // initialize()
        }

        std::lock_guard<std::mutex> lock(pose_mtx);


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

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "liloc_node");

  ROS_INFO("\033[1;32m----> LiLoc Started.\033[0m");

  

  return 0;
}
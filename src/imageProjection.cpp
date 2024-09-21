#include "utility.h"
#include "liloc/cloud_info.h"
#include <livox_ros_driver/CustomMsg.h>

struct VelodynePointXYZIRT {
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint16_t, ring, ring) (float, time, time)
)

struct OusterPointXYZIRT {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t ring;
    uint16_t noise;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint32_t, t, t) (uint16_t, reflectivity, reflectivity)
    (uint8_t, ring, ring) (uint16_t, noise, noise) (uint32_t, range, range)
)

struct RobosensePointXYZIRT {
    PCL_ADD_POINT4D
    float intensity;
    uint16_t ring;
    double timestamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(RobosensePointXYZIRT, 
      (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
      (uint16_t, ring, ring)(double, timestamp, timestamp)
)

// mulran datasets
struct MulranPointXYZIRT {
    PCL_ADD_POINT4D
    float intensity;
    uint32_t t;
    int ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 }EIGEN_ALIGN16;
 POINT_CLOUD_REGISTER_POINT_STRUCT (MulranPointXYZIRT,
     (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
     (uint32_t, t, t) (int, ring, ring)
 )

// Use the Velodyne point format as a common representation
using PointXYZIRT = VelodynePointXYZIRT;

const int queueLength = 2000;

class ImageProjection : public ParamServer {
private:
    std::mutex imuLock;
    std::mutex odoLock;

    ros::Subscriber subLaserCloud;
    ros::Publisher  pubLaserCloud;
    
    ros::Publisher pubExtractedCloud;
    ros::Publisher pubLaserCloudInfo;

    ros::Subscriber subImu;
    std::deque<sensor_msgs::Imu> imuQueue;

    ros::Subscriber subOdom;
    std::deque<nav_msgs::Odometry> odomQueue;

    std::deque<sensor_msgs::PointCloud2> cloudQueue;
    std::deque<livox_ros_driver::CustomMsg> cloudQueueLivox;

    sensor_msgs::PointCloud2 currentCloudMsg;
    livox_ros_driver::CustomMsg currentCloudMsgLivox;

    double *imuTime = new double[queueLength];
    double *imuRotX = new double[queueLength];
    double *imuRotY = new double[queueLength];
    double *imuRotZ = new double[queueLength];

    int imuPointerCur;
    bool firstPointFlag;
    Eigen::Affine3f transStartInverse;

    pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;
    pcl::PointCloud<OusterPointXYZIRT>::Ptr tmpOusterCloudIn;
    pcl::PointCloud<MulranPointXYZIRT>::Ptr tmpMulranCloudIn;
    pcl::PointCloud<PointType>::Ptr   fullCloud;

    int deskewFlag;

    bool odomDeskewFlag;
    float odomIncreX;
    float odomIncreY;
    float odomIncreZ;

    liloc::cloud_info cloudInfo;
    double timeScanCur;
    double timeScanEnd;
    std_msgs::Header cloudHeader;

public:
    ImageProjection(): deskewFlag(0) 
    {
        subImu        = nh.subscribe<sensor_msgs::Imu>(imuTopic, 2000, &ImageProjection::imuHandler, this, ros::TransportHints().tcpNoDelay());
        subOdom       = nh.subscribe<nav_msgs::Odometry>(odomTopic+"_incremental", 2000, &ImageProjection::odometryHandler, this, ros::TransportHints().tcpNoDelay());

        if (sensor == SensorType::LIVOX) {
            subLaserCloud = nh.subscribe<livox_ros_driver::CustomMsg>(pointCloudTopic, 5, &ImageProjection::cloudHandlerLivox, this, ros::TransportHints().tcpNoDelay());
        }
        else {
            subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic, 5, &ImageProjection::cloudHandler, this, ros::TransportHints().tcpNoDelay());
        }

        pubExtractedCloud = nh.advertise<sensor_msgs::PointCloud2> ("liloc/deskew/cloud_deskewed", 1);
        pubLaserCloudInfo = nh.advertise<liloc::cloud_info> ("liloc/deskew/cloud_info", 1);

        allocateMemory();
        resetParameters();

        pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    }

    void allocateMemory() {
        laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
        tmpOusterCloudIn.reset(new pcl::PointCloud<OusterPointXYZIRT>());
        tmpMulranCloudIn.reset(new pcl::PointCloud<MulranPointXYZIRT>());
        fullCloud.reset(new pcl::PointCloud<PointType>());

        resetParameters();
    }

    void resetParameters() {
        laserCloudIn->clear();
        fullCloud->clear();

        imuPointerCur = 0;
        firstPointFlag = true;
        odomDeskewFlag = false;

        for (int i = 0; i < queueLength; ++i)
        {
            imuTime[i] = 0;
            imuRotX[i] = 0;
            imuRotY[i] = 0;
            imuRotZ[i] = 0;
        }
    }

    ~ImageProjection(){}

    void imuHandler(const sensor_msgs::Imu::ConstPtr& imuMsg) {
        sensor_msgs::Imu thisImu = imuConverter(*imuMsg);

        if (correct) {
            thisImu.linear_acceleration.x = thisImu.linear_acceleration.x * kAccScale;
            thisImu.linear_acceleration.y = thisImu.linear_acceleration.y * kAccScale;
            thisImu.linear_acceleration.z = thisImu.linear_acceleration.z * kAccScale;
        }

        std::lock_guard<std::mutex> lock1(imuLock);
        imuQueue.push_back(thisImu);
    }

    void odometryHandler(const nav_msgs::Odometry::ConstPtr& odometryMsg) {
        std::lock_guard<std::mutex> lock2(odoLock);
        odomQueue.push_back(*odometryMsg);
    }

    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg) {
        if (!cachePointCloud(laserCloudMsg))
            return;

        if (!deskewInfo())
            return;

        projectPointCloud();

        publishClouds();

        resetParameters();
    }

    void cloudHandlerLivox(const livox_ros_driver::CustomMsg::ConstPtr &laserCloudMsg) {
        if (!cachePointCloudLivox(laserCloudMsg))
            return;

        if (!deskewInfo())
            return;

        projectPointCloud();

        publishClouds();

        resetParameters();
    }

    bool cachePointCloudLivox(const livox_ros_driver::CustomMsg::ConstPtr &laserCloudMsg) {
        static bool first_scan = true;
        static double last_time = 0.0;
        static double first_scan_time = 0.0;

        cloudQueueLivox.push_back(*laserCloudMsg);
        if (cloudQueueLivox.size() <= 2){
            return false;
        }

        currentCloudMsgLivox = std::move(cloudQueueLivox.front());
        cloudQueueLivox.pop_front();

        double cur_time = currentCloudMsgLivox.header.stamp.toSec();

        if (cur_time < last_time) {
            ROS_WARN("Livox Cloud Loop .");
            cloudQueueLivox.clear();
            last_time = cur_time;
        }

        if (first_scan) {
            first_scan_time = cur_time;
            first_scan = false;
        }
            
        double time_offset = (cur_time - first_scan_time) * (double)(1000);

        for (size_t i = 1; i < currentCloudMsgLivox.point_num; ++i) {
             if ((currentCloudMsgLivox.points[i].line < N_SCAN) &&
                ((currentCloudMsgLivox.points[i].tag & 0x30) == 0x10 || (currentCloudMsgLivox.points[i].tag & 0x30) == 0x00) &&
                !HasInf(currentCloudMsgLivox.points[i]) && !HasNan(currentCloudMsgLivox.points[i]) &&
                !IsNear(currentCloudMsgLivox.points[i], currentCloudMsgLivox.points[i - 1])) 
                {
                    PointXYZIRT point;
                    point.x = currentCloudMsgLivox.points[i].x;
                    point.y = currentCloudMsgLivox.points[i].y;
                    point.z = currentCloudMsgLivox.points[i].z;

                    point.time = time_offset + currentCloudMsgLivox.points[i].offset_time * 1e-6;  // ms

                    point.ring = currentCloudMsgLivox.points[i].line;

                    laserCloudIn->push_back(point);
                }
        }

        std::sort(laserCloudIn->points.begin(), laserCloudIn->points.end(), [](const PointXYZIRT& x, const PointXYZIRT& y) 
                        -> bool { return (x.time < y.time); });

        cloudHeader = currentCloudMsgLivox.header;
        timeScanCur = cloudHeader.stamp.toSec();
        timeScanEnd = timeScanCur + laserCloudIn->points.back().time / (double)(1000);

        first_scan = true;
        last_time = cur_time;

        deskewFlag = 1;

        return true;
    }

    bool cachePointCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg) {
        cloudQueue.push_back(*laserCloudMsg);
        if (cloudQueue.size() <= 2)
            return false;

        currentCloudMsg = std::move(cloudQueue.front());
        cloudQueue.pop_front();

        // for no ring and time channel pointcloud
        if (!have_ring_time_channel) {
            pcl::PointCloud<PointXYZIRT>::Ptr lidarCloudIn(new pcl::PointCloud<PointXYZIRT>());
            pcl::moveFromROSMsg(currentCloudMsg, *lidarCloudIn);

            bool halfPassed = false;
            int cloudNum = lidarCloudIn->points.size();

            double startOrientation = -atan2(lidarCloudIn->points[0].y, lidarCloudIn->points[0].x);
            double endOrientation = -atan2(lidarCloudIn->points[lidarCloudIn->points.size() - 1].y, lidarCloudIn->points[lidarCloudIn->points.size() - 1].x) + 2 * M_PI;
            if (endOrientation - startOrientation > 3 * M_PI) {
                endOrientation -= 2*M_PI;
            }
            else if (endOrientation - startOrientation < M_PI) {
                endOrientation += 2 * M_PI;
            }
            double orientationDiff = endOrientation - startOrientation;
            PointXYZIRT point;
            for (int i = 0; i < cloudNum; ++i) {
                point.x = lidarCloudIn->points[i].x;
                point.y = lidarCloudIn->points[i].y;
                point.z = lidarCloudIn->points[i].z;
                float ori = -atan2(point.y, point.x);
                if (!halfPassed) {
                    if (ori < startOrientation - M_PI / 2) {
                        ori += 2 * M_PI;
                    } else if (ori > startOrientation + M_PI * 3 / 2) {
                        ori -= 2 * M_PI;
                    }
                    if (ori - startOrientation > M_PI) {
                        halfPassed = true;
                    }
                } else {
                    ori += 2 * M_PI;
                    if (ori < endOrientation - M_PI * 3 / 2) {
                        ori += 2 * M_PI;
                    } else if (ori > endOrientation + M_PI / 2) {
                        ori -= 2 * M_PI;
                    }
                }
                float relTime = (ori - startOrientation) / orientationDiff;

                lidarCloudIn->points[i].time = 0.1 * relTime;
            }

            *laserCloudIn += *lidarCloudIn;

            deskewFlag = 1;
        }

        else {
            if (sensor == SensorType::VELODYNE) {
                pcl::moveFromROSMsg(currentCloudMsg, *laserCloudIn);
            }
            else if (sensor == SensorType::OUSTER) {
                pcl::moveFromROSMsg(currentCloudMsg, *tmpOusterCloudIn);
                laserCloudIn->points.resize(tmpOusterCloudIn->size());
                laserCloudIn->is_dense = tmpOusterCloudIn->is_dense;
                for (size_t i = 0; i < tmpOusterCloudIn->size(); i++) {
                    auto &src = tmpOusterCloudIn->points[i];
                    auto &dst = laserCloudIn->points[i];
                    dst.x = src.x;
                    dst.y = src.y;
                    dst.z = src.z;
                    dst.intensity = src.intensity;
                    dst.ring = src.ring;
                    dst.time = src.t * 1e-9f;
                }
            }
            else if (sensor == SensorType::MULRAN) {
                pcl::moveFromROSMsg(currentCloudMsg, *tmpMulranCloudIn);
                laserCloudIn->points.resize(tmpMulranCloudIn->size());
                laserCloudIn->is_dense = tmpMulranCloudIn->is_dense;
                for (size_t i = 0; i < tmpMulranCloudIn->size(); i++) {
                    auto &src = tmpMulranCloudIn->points[i];
                    auto &dst = laserCloudIn->points[i];
                    dst.x = src.x;
                    dst.y = src.y;
                    dst.z = src.z;
                    dst.intensity = src.intensity;
                    dst.ring = src.ring;
                    dst.time = static_cast<float>(src.t);
                }
            }
            else if (sensor == SensorType::ROBOSENSE) {
                pcl::PointCloud<RobosensePointXYZIRT>::Ptr tmpRobosenseCloudIn(new pcl::PointCloud<RobosensePointXYZIRT>());
                pcl::moveFromROSMsg(currentCloudMsg, *tmpRobosenseCloudIn);
                laserCloudIn->points.resize(tmpRobosenseCloudIn->size());
                laserCloudIn->is_dense = tmpRobosenseCloudIn->is_dense;

                double start_stamptime = tmpRobosenseCloudIn->points[0].timestamp;
                for (size_t i = 0; i < tmpRobosenseCloudIn->size(); i++) {
                    auto &src = tmpRobosenseCloudIn->points[i];
                    auto &dst = laserCloudIn->points[i];
                    dst.x = src.x;
                    dst.y = src.y;
                    dst.z = src.z;
                    dst.intensity = src.intensity;
                    dst.ring = src.ring;
                    dst.time = src.timestamp - start_stamptime;
                }
            }
            else {
                ROS_ERROR_STREAM("Unknown Sensor Type: " << int(sensor));
                ros::shutdown();
            }

            static int ringFlag = 0;
            if (ringFlag == 0) {
                ringFlag = -1;
                for (int i = 0; i < (int)currentCloudMsg.fields.size(); ++i) {
                    if (currentCloudMsg.fields[i].name == "ring") {
                        ringFlag = 1;
                        break;
                    }
                }
                if (ringFlag == -1) {
                    ROS_ERROR("Point cloud ring channel not available, please configure your point cloud data!");
                    ros::shutdown();
                }
            }

            if (deskewFlag == 0) {
                deskewFlag = -1;
                for (auto &field : currentCloudMsg.fields) {
                    if (field.name == "time" || field.name == "t") {
                        deskewFlag = 1;
                        break;
                    }
                }
                if (deskewFlag == -1)
                    ROS_WARN("Point cloud timestamp not available, deskew function disabled, system will drift significantly!");
            }
        }

        cloudHeader = currentCloudMsg.header;
        timeScanCur = cloudHeader.stamp.toSec();
        timeScanEnd = timeScanCur + laserCloudIn->points.back().time;

        if (laserCloudIn->is_dense == false) {
            ROS_ERROR("Point cloud is not in dense format, please remove NaN points first!");
            ros::shutdown();
        }

        return true;
    }

    bool deskewInfo() {
        std::lock_guard<std::mutex> lock1(imuLock);
        std::lock_guard<std::mutex> lock2(odoLock);

        if (imuQueue.empty() || imuQueue.front().header.stamp.toSec() > timeScanCur || imuQueue.back().header.stamp.toSec() < timeScanEnd) {
            ROS_DEBUG("Waiting for IMU data ...");
            return false;
        }

        imuDeskewInfo();

        odomDeskewInfo();

        return true;
    }

    void imuDeskewInfo() {
        cloudInfo.imuAvailable = false;

        while (!imuQueue.empty()) {
            if (imuQueue.front().header.stamp.toSec() < timeScanCur - 0.01)
                imuQueue.pop_front();
            else
                break;
        }

        if (imuQueue.empty())
            return;

        imuPointerCur = 0;

        for (int i = 0; i < (int)imuQueue.size(); ++i) {
            sensor_msgs::Imu thisImuMsg = imuQueue[i];
            double currentImuTime = thisImuMsg.header.stamp.toSec();

            if (imuType) {
                // get roll, pitch, and yaw estimation for this scan
                if (currentImuTime <= timeScanCur)
                    imuRPY2rosRPY(&thisImuMsg, &cloudInfo.imuRollInit, &cloudInfo.imuPitchInit, &cloudInfo.imuYawInit);
            }

            if (currentImuTime > timeScanEnd + 0.01)
                break;

            if (imuPointerCur == 0) {
                imuRotX[0] = 0;
                imuRotY[0] = 0;
                imuRotZ[0] = 0;
                imuTime[0] = currentImuTime;
                ++imuPointerCur;
                continue;
            }

            // get angular velocity
            double angular_x, angular_y, angular_z;
            imuAngular2rosAngular(&thisImuMsg, &angular_x, &angular_y, &angular_z);

            // integrate rotation
            double timeDiff = currentImuTime - imuTime[imuPointerCur-1];
            imuRotX[imuPointerCur] = imuRotX[imuPointerCur-1] + angular_x * timeDiff;
            imuRotY[imuPointerCur] = imuRotY[imuPointerCur-1] + angular_y * timeDiff;
            imuRotZ[imuPointerCur] = imuRotZ[imuPointerCur-1] + angular_z * timeDiff;
            imuTime[imuPointerCur] = currentImuTime;
            ++imuPointerCur;
        }

        --imuPointerCur;

        if (imuPointerCur <= 0)
            return;

        cloudInfo.imuAvailable = true;
    }

    void odomDeskewInfo() {
        cloudInfo.odomAvailable = false;
        static float sync_diff_time = (imuRate >= 300) ? 0.01 : 0.20;
        while (!odomQueue.empty()) {
            if (odomQueue.front().header.stamp.toSec() < timeScanCur - sync_diff_time)
                odomQueue.pop_front();
            else
                break;
        }

        if (odomQueue.empty())
            return;

        if (odomQueue.front().header.stamp.toSec() > timeScanCur)
            return;

        // get start odometry at the beinning of the scan
        nav_msgs::Odometry startOdomMsg;

        for (int i = 0; i < (int)odomQueue.size(); ++i) {
            startOdomMsg = odomQueue[i];

            if (ROS_TIME(&startOdomMsg) < timeScanCur)
                continue;
            else
                break;
        }

        tf::Quaternion orientation;
        tf::quaternionMsgToTF(startOdomMsg.pose.pose.orientation, orientation);

        double roll, pitch, yaw;
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

        cloudInfo.initialGuessX = startOdomMsg.pose.pose.position.x;
        cloudInfo.initialGuessY = startOdomMsg.pose.pose.position.y;
        cloudInfo.initialGuessZ = startOdomMsg.pose.pose.position.z;
        cloudInfo.initialGuessRoll  = roll;
        cloudInfo.initialGuessPitch = pitch;
        cloudInfo.initialGuessYaw   = yaw;

        cloudInfo.odomAvailable = true;

        odomDeskewFlag = false;

        if (odomQueue.back().header.stamp.toSec() < timeScanEnd)
            return;

        nav_msgs::Odometry endOdomMsg;

        for (int i = 0; i < (int)odomQueue.size(); ++i) {
            endOdomMsg = odomQueue[i];

            if (ROS_TIME(&endOdomMsg) < timeScanEnd)
                continue;
            else
                break;
        }

        if (int(round(startOdomMsg.pose.covariance[0])) != int(round(endOdomMsg.pose.covariance[0])))
            return;

        Eigen::Affine3f transBegin = pcl::getTransformation(startOdomMsg.pose.pose.position.x, startOdomMsg.pose.pose.position.y, startOdomMsg.pose.pose.position.z, roll, pitch, yaw);

        tf::quaternionMsgToTF(endOdomMsg.pose.pose.orientation, orientation);
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        Eigen::Affine3f transEnd = pcl::getTransformation(endOdomMsg.pose.pose.position.x, endOdomMsg.pose.pose.position.y, endOdomMsg.pose.pose.position.z, roll, pitch, yaw);

        Eigen::Affine3f transBt = transBegin.inverse() * transEnd;

        float rollIncre, pitchIncre, yawIncre;
        pcl::getTranslationAndEulerAngles(transBt, odomIncreX, odomIncreY, odomIncreZ, rollIncre, pitchIncre, yawIncre);

        odomDeskewFlag = true;
    }

    void findRotation(double pointTime, float *rotXCur, float *rotYCur, float *rotZCur) {
        *rotXCur = 0; *rotYCur = 0; *rotZCur = 0;

        int imuPointerFront = 0;
        while (imuPointerFront < imuPointerCur) {
            if (pointTime < imuTime[imuPointerFront])
                break;
            ++imuPointerFront;
        }

        if (pointTime > imuTime[imuPointerFront] || imuPointerFront == 0) {
            *rotXCur = imuRotX[imuPointerFront];
            *rotYCur = imuRotY[imuPointerFront];
            *rotZCur = imuRotZ[imuPointerFront];
        } 
        else {
            int imuPointerBack = imuPointerFront - 1;
            double ratioFront = (pointTime - imuTime[imuPointerBack]) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            double ratioBack = (imuTime[imuPointerFront] - pointTime) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            *rotXCur = imuRotX[imuPointerFront] * ratioFront + imuRotX[imuPointerBack] * ratioBack;
            *rotYCur = imuRotY[imuPointerFront] * ratioFront + imuRotY[imuPointerBack] * ratioBack;
            *rotZCur = imuRotZ[imuPointerFront] * ratioFront + imuRotZ[imuPointerBack] * ratioBack;
        }
    }

    void findPosition(double relTime, float *posXCur, float *posYCur, float *posZCur) {
        *posXCur = 0; *posYCur = 0; *posZCur = 0;

        // If the sensor moves relatively slow, like walking speed, positional deskew seems to have little benefits. Thus code below is commented.

        // if (cloudInfo.odomAvailable == false || odomDeskewFlag == false)
        //     return;

        // float ratio = relTime / (timeScanEnd - timeScanCur);

        // *posXCur = ratio * odomIncreX;
        // *posYCur = ratio * odomIncreY;
        // *posZCur = ratio * odomIncreZ;
    }

    PointType deskewPoint(PointType *point, double relTime) {
        if (deskewFlag == -1 || cloudInfo.imuAvailable == false)
            return *point;

        double pointTime = timeScanCur + relTime;

        float rotXCur, rotYCur, rotZCur;
        findRotation(pointTime, &rotXCur, &rotYCur, &rotZCur);

        float posXCur, posYCur, posZCur;
        findPosition(relTime, &posXCur, &posYCur, &posZCur);

        if (firstPointFlag == true) {
            transStartInverse = (pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur)).inverse();
            firstPointFlag = false;
        }

        // transform points to start
        Eigen::Affine3f transFinal = pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur);
        Eigen::Affine3f transBt = transStartInverse * transFinal;

        PointType newPoint;
        newPoint.x = transBt(0,0) * point->x + transBt(0,1) * point->y + transBt(0,2) * point->z + transBt(0,3);
        newPoint.y = transBt(1,0) * point->x + transBt(1,1) * point->y + transBt(1,2) * point->z + transBt(1,3);
        newPoint.z = transBt(2,0) * point->x + transBt(2,1) * point->y + transBt(2,2) * point->z + transBt(2,3);
        newPoint.intensity = point->intensity;

        return newPoint;
    }

    void projectPointCloud() {
        int cloudSize = laserCloudIn->points.size();

        for (int i = 0; i < cloudSize; ++i) {
            PointType thisPoint;
            thisPoint.x = laserCloudIn->points[i].x;
            thisPoint.y = laserCloudIn->points[i].y;
            thisPoint.z = laserCloudIn->points[i].z;
            thisPoint.intensity = laserCloudIn->points[i].intensity;

            float range = pointDistance(thisPoint);
            if (range < lidarMinRange || range > lidarMaxRange)
                continue;

            int rowIdn = laserCloudIn->points[i].ring;
            if (rowIdn < 0 || rowIdn >= N_SCAN)
                continue;

            if (rowIdn % downsampleRate != 0)
                continue;

            if (i % point_filter_num != 0)
                continue;

            thisPoint = deskewPoint(&thisPoint, laserCloudIn->points[i].time);

            fullCloud->push_back(thisPoint);
        }
    }
    
    void publishClouds() {
        cloudInfo.header = cloudHeader;
        cloudInfo.cloud_deskewed  = publishCloud(pubExtractedCloud, fullCloud, cloudHeader.stamp, lidarFrame);
        pubLaserCloudInfo.publish(cloudInfo);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "liloc");

    common_lib_ = std::make_shared<CommonLib::common_lib>("LiLoc");

    ImageProjection IP;
    
    ROS_INFO("\033[1;32m----> Image Projection Started.\033[0m");

    ros::MultiThreadedSpinner spinner(3);
    spinner.spin();
    
    return 0;
}

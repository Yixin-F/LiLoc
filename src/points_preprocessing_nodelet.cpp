#include "utility.h"
#include "block_localization/cloud_info.h"

const int queueLength = 2000;

namespace block_localization {

class PointsPreprocessingNodelet : public ParamServer, public nodelet::Nodelet
{
private:
    std::mutex imu_mtx;
    std::mutex odom_mtx;

    ros::Subscriber subImu;
    std::deque<sensor_msgs::Imu> imuQueue;

    ros::Subscriber subOdom;
    std::deque<nav_msgs::Odometry> odomQueue;

    ros::Subscriber subLidarCloud;
    ros::Publisher pubLidarCloud;
    ros::Publisher pubLidarCloudInfo;

    std::deque<sensor_msgs::PointCloud2> cloudQueue;
    sensor_msgs::PointCloud2 currentCloudMsg;
    pcl::PointCloud<PointIRT>::Ptr lidarCloudIn;
    pcl::PointCloud<PointT>::Ptr deskewedCloud;

    double *imuTime = new double[queueLength];
    double *imuRotX = new double[queueLength];
    double *imuRotY = new double[queueLength];
    double *imuRotZ = new double[queueLength];

    block_localization::cloud_info cloudInfo;
    double timeScanCur;
    double timeScanEnd;
    std_msgs::Header cloudHeader;

    int deskewFlag;

    bool odomDeskewFlag;
    float odomIncreX;
    float odomIncreY;
    float odomIncreZ;

    int imuPointerCur;
    bool firstPointFlag;
    Eigen::Affine3f transStartInverse;


public:
    PointsPreprocessingNodelet() : deskewFlag(0) {
    }
    virtual ~PointsPreprocessingNodelet() {
    };

    void onInit() override {
        nh = getNodeHandle();

        subImu = nh.subscribe<sensor_msgs::Imu>(imuTopic, 2000, &PointsPreprocessingNodelet::imuHandler, this, ros::TransportHints().tcpNoDelay());
        subOdom = nh.subscribe<nav_msgs::Odometry>("/imu_incremental", 2000, &PointsPreprocessingNodelet::odometryHandler, this, ros::TransportHints().tcpNoDelay());
        subLidarCloud = nh.subscribe<sensor_msgs::PointCloud2>(lidarTopic, 5, &PointsPreprocessingNodelet::cloudHandler, this, ros::TransportHints().tcpNoDelay());
        
        pubLidarCloud = nh.advertise<sensor_msgs::PointCloud2>("/cloud_deskewed", 1);
        pubLidarCloudInfo = nh.advertise<block_localization::cloud_info>("/cloud_info", 1);

        allocateMemory();

        pcl::console::setVerbosityLevel(pcl::console::L_ERROR);    
    }


    void allocateMemory()
    {
        lidarCloudIn.reset(new pcl::PointCloud<PointIRT>());
        deskewedCloud.reset(new pcl::PointCloud<PointT>());

        resetParameters();
    }


    void resetParameters()
    {
        lidarCloudIn->clear();
        deskewedCloud->clear();

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


    void imuHandler(const sensor_msgs::Imu::ConstPtr& imuMsg) {
        sensor_msgs::Imu thisImu = imuConverter(*imuMsg);

        std::lock_guard<std::mutex> lock(imu_mtx);
        imuQueue.push_back(thisImu);        
    }

    
    void odometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg) {
        std::lock_guard<std::mutex> lock(odom_mtx);
        odomQueue.push_back(*odomMsg);
    }


    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& cloudMsg) {
        if (!cachePointCloud(cloudMsg))
            return;
        
        if (!deskewInfo())
            return;

        projectPointCloud();

        publishClouds();

        resetParameters();
    }


    bool cachePointCloud(const sensor_msgs::PointCloud2ConstPtr& cloudMsg) {
        cloudQueue.push_back(*cloudMsg);
        if (cloudQueue.size() <= 2)
            return false;

        currentCloudMsg = std::move(cloudQueue.front());
        cloudQueue.pop_front();
        pcl::moveFromROSMsg(currentCloudMsg, *lidarCloudIn);

        // for no ring and time channel pointcloud
        if (!have_ring_time_channel) 
        {
            bool halfPassed = false;
            int cloudNum = lidarCloudIn->points.size();

            cloudInfo.startOrientation = -atan2(lidarCloudIn->points[0].y, lidarCloudIn->points[0].x);
            cloudInfo.endOrientation = -atan2(lidarCloudIn->points[lidarCloudIn->points.size() - 1].y, lidarCloudIn->points[lidarCloudIn->points.size() - 1].x) + 2 * M_PI;
            if (cloudInfo.endOrientation - cloudInfo.startOrientation > 3 * M_PI) {
                cloudInfo.endOrientation -= 2*M_PI;
            }
            else if (cloudInfo.endOrientation - cloudInfo.startOrientation < M_PI) {
                cloudInfo.endOrientation += 2 * M_PI;
            }
            cloudInfo.orientationDiff = cloudInfo.endOrientation - cloudInfo.startOrientation;
            PointT point;
            for (int i = 0; i < cloudNum; ++i)
            {
                point.x = lidarCloudIn->points[i].x;
                point.y = lidarCloudIn->points[i].y;
                point.z = lidarCloudIn->points[i].z;
                float ori = -atan2(point.y, point.x);
                if (!halfPassed) {
                    if (ori < cloudInfo.startOrientation - M_PI / 2) {
                        ori += 2 * M_PI;
                    } else if (ori > cloudInfo.startOrientation + M_PI * 3 / 2) {
                        ori -= 2 * M_PI;
                    }
                    if (ori - cloudInfo.startOrientation > M_PI) {
                        halfPassed = true;
                    }
                } else {
                    ori += 2 * M_PI;
                    if (ori < cloudInfo.endOrientation - M_PI * 3 / 2) {
                        ori += 2 * M_PI;
                    } else if (ori > cloudInfo.endOrientation + M_PI / 2) {
                        ori -= 2 * M_PI;
                    }
                }
                float relTime = (ori - cloudInfo.startOrientation) / cloudInfo.orientationDiff;

                lidarCloudIn->points[i].time = 0.1 * relTime;
            }
            deskewFlag = 1;
        } 
        else 
        {
            // check ring channel
            static int ringFlag = 0;
            if (ringFlag == 0) {
                ringFlag = -1;
                for (auto &field : currentCloudMsg.fields) {
                    if (field.name == "ring") {
                        ringFlag = 1;
                        break;
                    }
                }
                if (ringFlag == -1) {
                    NODELET_ERROR("Point cloud ring channel not available, please configure your point cloud data!");
                    ros::shutdown();
                }
            }

            // check point time
            if (deskewFlag == 0)
            {
                deskewFlag = -1;
                for (auto &field : currentCloudMsg.fields)
                {
                    if (field.name == "time" || field.name == "t")
                    {
                        deskewFlag = 1;
                        break;
                    }
                }
                if (deskewFlag == -1)
                    NODELET_WARN("Point cloud timestamp not available, deskew function disabled, system will drift significantly!");
            }
        }
        cloudHeader = currentCloudMsg.header;
        timeScanCur = cloudHeader.stamp.toSec();
        timeScanEnd = timeScanCur + lidarCloudIn->points.back().time;
        
        return true;
    }


    bool deskewInfo() {
        std::lock_guard<std::mutex> lock1(imu_mtx);
        std::lock_guard<std::mutex> lock2(odom_mtx);

        // make sure Imu data available for the scan
        if (imuQueue.empty() || imuQueue.front().header.stamp.toSec() > timeScanCur || imuQueue.back().header.stamp.toSec() < timeScanEnd)
        {
            NODELET_DEBUG("Waiting for IMU data ...");
            return false;
        }

        imuDeskewInfo();

        odomDeskewInfo();

        return true;
    }


    void imuDeskewInfo() {
        cloudInfo.imuAvailable = false;

        // pop out imu before last cloud
        while (!imuQueue.empty())
        {
            if (imuQueue.front().header.stamp.toSec() < timeScanCur - 0.01)
                imuQueue.pop_front();
            else
                break;
        }

        if (imuQueue.empty())
            return;

        imuPointerCur = 0;

        for (auto &thisImuMsg : imuQueue) {
            double currentImuTime = thisImuMsg.header.stamp.toSec();

            // get roll, pitch, and yaw estimation for this scan
            if (currentImuTime <= timeScanCur)
                imuRPY2rosRPY(&thisImuMsg, &cloudInfo.imuRollInit, &cloudInfo.imuPitchInit, &cloudInfo.imuYawInit);
            
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

        while (!odomQueue.empty())
        {
            if (odomQueue.front().header.stamp.toSec() < timeScanCur - 0.01)
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

        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            startOdomMsg = odomQueue[i];

            if (getROSTime(&startOdomMsg) < timeScanCur)
                continue;
            else
                break;
        }

        tf::Quaternion orientation;
        tf::quaternionMsgToTF(startOdomMsg.pose.pose.orientation, orientation);

        double roll, pitch, yaw;
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

        // Initial guess
        cloudInfo.initialGuessX = startOdomMsg.pose.pose.position.x;
        cloudInfo.initialGuessY = startOdomMsg.pose.pose.position.y;
        cloudInfo.initialGuessZ = startOdomMsg.pose.pose.position.z;
        cloudInfo.initialGuessRoll  = roll;
        cloudInfo.initialGuessPitch = pitch;
        cloudInfo.initialGuessYaw   = yaw;

        cloudInfo.odomAvailable = true;

        // get end odometry at the end of the scan
        odomDeskewFlag = false;

        if (odomQueue.back().header.stamp.toSec() < timeScanEnd)
            return;

        nav_msgs::Odometry endOdomMsg;

        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            endOdomMsg = odomQueue[i];

            if (getROSTime(&endOdomMsg) < timeScanEnd)
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


    void projectPointCloud() {
        int cloudSize = lidarCloudIn->points.size();

        for (int i = 0; i < cloudSize; ++i) {
            PointT thisPoint;
            thisPoint.x = lidarCloudIn->points[i].x;
            thisPoint.y = lidarCloudIn->points[i].y;
            thisPoint.z = lidarCloudIn->points[i].z;
            thisPoint.intensity = lidarCloudIn->points[i].intensity;

            float range = pointDistance(thisPoint);
            if (range < lidarMinRange || range > lidarMaxRange)
                continue;

            thisPoint = deskewPoint(&thisPoint, lidarCloudIn->points[i].time);
            deskewedCloud->push_back(thisPoint);
        }
    }


    void findRotation(double pointTime, float *rotXCur, float *rotYCur, float *rotZCur) {
        *rotXCur = 0; *rotYCur = 0; *rotZCur = 0;

        int imuPointerFront = 0;
        while (imuPointerFront < imuPointerCur)
        {
            if (pointTime < imuTime[imuPointerFront])
                break;
            ++imuPointerFront;
        }

        if (pointTime > imuTime[imuPointerFront] || imuPointerFront == 0)
        {
            *rotXCur = imuRotX[imuPointerFront];
            *rotYCur = imuRotY[imuPointerFront];
            *rotZCur = imuRotZ[imuPointerFront];
        } else {
            int imuPointerBack = imuPointerFront - 1;
            double ratioFront = (pointTime - imuTime[imuPointerBack]) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            double ratioBack = (imuTime[imuPointerFront] - pointTime) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            *rotXCur = imuRotX[imuPointerFront] * ratioFront + imuRotX[imuPointerBack] * ratioBack;
            *rotYCur = imuRotY[imuPointerFront] * ratioFront + imuRotY[imuPointerBack] * ratioBack;
            *rotZCur = imuRotZ[imuPointerFront] * ratioFront + imuRotZ[imuPointerBack] * ratioBack;
        }
    }


    void findPosition(double relTime, float *posXCur, float *posYCur, float *posZCur)
    {
        *posXCur = 0; *posYCur = 0; *posZCur = 0;

        // If the sensor moves relatively slow, like walking speed, positional deskew seems to have little benefits. Thus code below is commented.

        if (cloudInfo.odomAvailable == false || odomDeskewFlag == false)
            return;

        float ratio = relTime / (timeScanEnd - timeScanCur);

        *posXCur = ratio * odomIncreX;
        *posYCur = ratio * odomIncreY;
        *posZCur = ratio * odomIncreZ;
    }


    PointT deskewPoint(PointT *point, double relTime) {
        if (deskewFlag == -1 || cloudInfo.imuAvailable == false)
            return *point;

        double pointTime = timeScanCur + relTime;

        float rotXCur, rotYCur, rotZCur;
        findRotation(pointTime, &rotXCur, &rotYCur, &rotZCur);

        float posXCur, posYCur, posZCur;
        findPosition(relTime, &posXCur, &posYCur, &posZCur);

        if (firstPointFlag == true)
        {
            transStartInverse = (pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur)).inverse();
            firstPointFlag = false;
        }

        // transform points to start
        Eigen::Affine3f transFinal = pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur);
        Eigen::Affine3f transBt = transStartInverse * transFinal;

        PointT newPoint;
        newPoint.x = transBt(0,0) * point->x + transBt(0,1) * point->y + transBt(0,2) * point->z + transBt(0,3);
        newPoint.y = transBt(1,0) * point->x + transBt(1,1) * point->y + transBt(1,2) * point->z + transBt(1,3);
        newPoint.z = transBt(2,0) * point->x + transBt(2,1) * point->y + transBt(2,2) * point->z + transBt(2,3);
        newPoint.intensity = point->intensity;

        return newPoint;
    }


    void publishClouds() {
        cloudInfo.header = cloudHeader;
        cloudInfo.cloud_deskewed = publishCloud(pubLidarCloud, deskewedCloud, cloudHeader.stamp, cloudHeader.frame_id);
        pubLidarCloudInfo.publish(cloudInfo);
    }
};

}

PLUGINLIB_EXPORT_CLASS(block_localization::PointsPreprocessingNodelet, nodelet::Nodelet)
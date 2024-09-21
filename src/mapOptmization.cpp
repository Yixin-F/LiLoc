#include "utility.h"
#include "tictoc.h"

#include "liloc/cloud_info.h"
#include "liloc/save_map.h"
#include "liloc/save_session.h"

#include "dataManager/dataSaver.hpp"
#include "dataManager/dataLoader.hpp"

#include "egoOptimization/anchorOptimize.hpp"

#include <pclomp/ndt_omp.h>
#include <pclomp/gicp_omp.h>

using namespace gtsam;

class mapOptimization : public ParamServer {
public:
    gtsam::NonlinearFactorGraph gtSAMgraph;
    gtsam::Values initialEstimate;
    gtsam::Values optimizedEstimate;
    gtsam::ISAM2 *isam;
    gtsam::Values isamCurrentEstimate;
    Eigen::MatrixXd poseCovariance;

    ros::Publisher pubLaserCloudSurround;
    ros::Publisher pubLaserOdometryGlobal;
    ros::Publisher pubLaserOdometryIncremental;
    ros::Publisher pubKeyPoses;
    ros::Publisher pubPath;

    ros::Publisher pubRecentKeyFrames;
    ros::Publisher pubRecentKeyFrame;
    ros::Publisher pubCloudRegisteredRaw;
    ros::Publisher pubLocalVertexAndEdge;

    ros::Subscriber subCloud;

    ros::Publisher pubPriorGlobalMap;
    ros::Publisher pubPriorGlobalTrajectory;
    ros::Publisher pubPriorLocalSubmap;
    ros::Publisher pubPriorLocalSubmapCenteriod;

    ros::Subscriber subPose;

    ros::ServiceServer srvSaveMap;
    ros::ServiceServer srvSaveSession;

    liloc::cloud_info cloudInfo;

    vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames;
    
    pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;
    pcl::PointCloud<PointType>::Ptr copy_cloudKeyPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr copy_cloudKeyPoses6D;

    pcl::PointCloud<PointType>::Ptr laserCloudSurfLast; // surf feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLastDS; // downsampled surf feature set from odoOptimization

    pcl::PointCloud<PointType>::Ptr laserCloudOri;
    pcl::PointCloud<PointType>::Ptr coeffSel;

    std::vector<PointType> laserCloudOriSurfVec; // surf point holder for parallel computation
    std::vector<PointType> coeffSelSurfVec;
    std::vector<bool> laserCloudOriSurfFlag;

    map<int, pair<pcl::PointCloud<PointType>, pcl::PointCloud<PointType>>> laserCloudMapContainer;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMapDS;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurroundingKeyPoses;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistoryKeyPoses;

    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    pcl::VoxelGrid<PointType> downSizeFilterLocalMapSurf;
    pcl::VoxelGrid<PointType> downSizeFilterSurroundingKeyPoses; // for surrounding key poses of scan-to-map optimization
    
    ros::Time timeLaserInfoStamp;
    double timeLaserInfoCur;

    float transformTobeMapped[6];
    float transformTobeMappedInit[6];

    std::mutex mtx;

    bool isDegenerate = false;
    cv::Mat matP;

    bool systemInitialized = false;
    bool poseInitialized = false;

    std::fstream pgSaveStream; // pg: pose-graph 
    std::vector<std::string> edges_str;
    std::vector<std::string> vertices_str;

    int laserCloudSurfFromMapDSNum = 0;
    int laserCloudSurfLastDSNum = 0;

    bool aLoopIsClosed = false;

    nav_msgs::Path globalPath;

    Eigen::Affine3f transPointAssociateToMap;
    Eigen::Affine3f incrementalOdometryAffineFront;
    Eigen::Affine3f incrementalOdometryAffineBack;

    // data manager
    std::shared_ptr<dataManager::Session> data_loader = nullptr;
    std::shared_ptr<dataManager::DataSaver> data_saver = nullptr;

    // optimization
    std::unique_ptr<optimization::AnchorOptimization> optimize = nullptr;

    pcl::Registration<PointType, PointType>::Ptr registration = nullptr;

    std::vector<double> total_time;
    std::vector<double> reg_time;
    std::vector<double> opt_time;

    std::vector<double> ros_time_tum;

public:

    ~mapOptimization() { }

    mapOptimization() {
        ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.1;
        parameters.relinearizeSkip = 1;
        isam = new ISAM2(parameters);

        pubKeyPoses                 = nh.advertise<sensor_msgs::PointCloud2>("liloc/mapping/trajectory", 1);
        pubLaserCloudSurround       = nh.advertise<sensor_msgs::PointCloud2>("liloc/mapping/map_global", 1);
        pubLaserOdometryGlobal      = nh.advertise<nav_msgs::Odometry> ("liloc/mapping/odometry", 1);
        pubLaserOdometryIncremental = nh.advertise<nav_msgs::Odometry> ("liloc/mapping/odometry_incremental", 1);
        pubPath                     = nh.advertise<nav_msgs::Path>("liloc/mapping/path", 1);

        subCloud = nh.subscribe<liloc::cloud_info>("liloc/deskew/cloud_info", 1, &mapOptimization::laserCloudInfoHandler, this, ros::TransportHints().tcpNoDelay());
        subPose  = nh.subscribe("/initialpose", 8, &mapOptimization::initialposeHandler, this, ros::TransportHints().tcpNoDelay());

        srvSaveMap  = nh.advertiseService("liloc/save_map", &mapOptimization::saveMapService, this);
        srvSaveSession = nh.advertiseService("liloc/save_session", &mapOptimization::saveSessionService, this);

        pubRecentKeyFrames    = nh.advertise<sensor_msgs::PointCloud2>("liloc/mapping/map_local", 1);
        pubRecentKeyFrame     = nh.advertise<sensor_msgs::PointCloud2>("liloc/mapping/cloud_registered", 1);
        pubCloudRegisteredRaw = nh.advertise<sensor_msgs::PointCloud2>("liloc/mapping/cloud_registered_raw", 1);

        pubPriorGlobalMap                 = nh.advertise<sensor_msgs::PointCloud2>("liloc/prior/map_prior", 1);
        pubPriorGlobalTrajectory          = nh.advertise<sensor_msgs::PointCloud2>("liloc/prior/traj_prior", 1);
        pubPriorLocalSubmap               = nh.advertise<sensor_msgs::PointCloud2>("liloc/prior/submap_prior", 1);
        pubPriorLocalSubmapCenteriod      = nh.advertise<sensor_msgs::PointCloud2>("liloc/prior/subcenter_prior", 1);
        pubLocalVertexAndEdge             = nh.advertise<visualization_msgs::MarkerArray>("/liloc/prior/local_constrains", 1);

        downSizeFilterSurf.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
        downSizeFilterLocalMapSurf.setLeafSize(surroundingKeyframeMapLeafSize, surroundingKeyframeMapLeafSize, surroundingKeyframeMapLeafSize);
        downSizeFilterSurroundingKeyPoses.setLeafSize(surroundingKeyframeDensity, surroundingKeyframeDensity, surroundingKeyframeDensity); // for surrounding key poses of scan-to-map optimization

        initialize();

        allocateMemory();
    }

    void initialize() {
        if (mode == ModeType::LIO) {
            // data_saver.reset(new dataManager::DataSaver(savePCDDirectory, mode));
        }
        else if (mode == ModeType::RELO) {
            // data_saver.reset(new dataManager::DataSaver(savePCDDirectory, mode));
            data_loader.reset(new dataManager::Session(1, "prior", savePCDDirectory, true));  // FIXME: must use "1"

            pclomp::NormalDistributionsTransform<PointType, PointType>::Ptr ndt(new pclomp::NormalDistributionsTransform<PointType, PointType>());
            pclomp::GeneralizedIterativeClosestPoint<PointType, PointType>::Ptr gicp(new pclomp::GeneralizedIterativeClosestPoint<PointType, PointType>());

            ndt->setTransformationEpsilon(ndtEpsilon);
            ndt->setResolution(ndtResolution);

            if (regMethod == "DIRECT1") {
                ROS_INFO("Using NDT_OMP with DIRECT1.");
                ndt->setNeighborhoodSearchMethod(pclomp::DIRECT1);
                registration = ndt;
            }
            else if (regMethod == "DIRECT7") {
                ROS_INFO("Using NDT_OMP with DIRECT7.");
                ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
                registration = ndt;
            }
            else if (regMethod == "GICP_OMP") {
                ROS_INFO("Using GICP_OMP.");
                registration = gicp;
            }
            else if (regMethod == "KDTREE") {
                ROS_INFO("Using NDT_OMP with KDTREE.");
                ndt->setNeighborhoodSearchMethod(pclomp::KDTREE);
                registration = ndt;
            }
            else {
                ROS_ERROR("Invaild Registration Method !");
                ros::shutdown();
            }

            optimize.reset(new optimization::AnchorOptimization(data_loader, registration));
        }
        else {
            ROS_ERROR(" Invaild Mode Type !");
            ros::shutdown();
        }
    }

    void allocateMemory() {
        cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
        copy_cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        copy_cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());

        kdtreeSurroundingKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());
        kdtreeHistoryKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());

        laserCloudSurfLast.reset(new pcl::PointCloud<PointType>()); // surf feature set from odoOptimization
        laserCloudSurfLastDS.reset(new pcl::PointCloud<PointType>()); // downsampled surf featuer set from odoOptimization

        laserCloudOri.reset(new pcl::PointCloud<PointType>());
        coeffSel.reset(new pcl::PointCloud<PointType>());

        laserCloudOriSurfVec.resize(N_SCAN * Horizon_SCAN);
        coeffSelSurfVec.resize(N_SCAN * Horizon_SCAN);
        laserCloudOriSurfFlag.resize(N_SCAN * Horizon_SCAN);

        std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(), false);

        laserCloudSurfFromMap.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfFromMapDS.reset(new pcl::PointCloud<PointType>());

        kdtreeSurfFromMap.reset(new pcl::KdTreeFLANN<PointType>());

        for (int i = 0; i < 6; ++i) {
            transformTobeMapped[i] = 0;
        }

        matP = cv::Mat(6, 6, CV_32F, cv::Scalar::all(0));
    }

    void laserCloudInfoHandler(const liloc::cloud_infoConstPtr& msgIn) {
        static double timeLastProcessing = -1;

        timeLaserInfoStamp = msgIn->header.stamp;
        timeLaserInfoCur = msgIn->header.stamp.toSec();

        cloudInfo = *msgIn;
        pcl::fromROSMsg(msgIn->cloud_deskewed, *laserCloudSurfLast);

        std::lock_guard<std::mutex> lock(mtx);

        if (timeLaserInfoCur - timeLastProcessing < timeInterval) {
            return ;
        }

        if (mode == ModeType::RELO && !poseInitialized) {
            if (!systemInitialized) {
                ROS_WARN("Wait for Initialized Pose ...");
                return ;
            }
            else {

                PointTypePose init_pose = trans2PointTypePose(transformTobeMappedInit);

                int submap_id;
                data_loader->searchNearestSubMapAndVertex(init_pose, submap_id);

                registration->setInputTarget(data_loader->usingSubMap_);
                registration->setInputSource(laserCloudSurfLast);

                Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();
                Eigen::Matrix3f rotation = eulerToRotation(transformTobeMappedInit[0], transformTobeMappedInit[1], transformTobeMappedInit[2]);
                Eigen::Vector3f translation(transformTobeMappedInit[3], transformTobeMappedInit[4], transformTobeMappedInit[5]);
                init_guess.block(0, 0, 3, 3) = rotation;
                init_guess.block(0, 3, 3, 1) = translation;

                for (int i = 0; i < 2; i++) {
                    CloudPtr aligned(new Cloud());
                    registration->align(*aligned, init_guess);

                    Eigen::Matrix4f transform;
                    transform = registration->getFinalTransformation();

                    init_guess = transform;
                }

                Eigen::Vector3f euler = RotMtoEuler(Eigen::Matrix3f(init_guess.block(0, 0, 3, 3)));
                Eigen::Vector3f xyz = init_guess.block(0, 3, 3, 1);

                transformTobeMappedInit[0] = euler(0);
                transformTobeMappedInit[1] = euler(1);
                transformTobeMappedInit[2] = euler(2);
                transformTobeMappedInit[3] = xyz(0);
                transformTobeMappedInit[4] = xyz(1);
                transformTobeMappedInit[5] = xyz(2);

                systemInitialized = true;
                poseInitialized = true;
            }
        }

        TicToc time;

        updateInitialGuess();

        extractSurroundingKeyFrames();

        downsampleCurrentScan();

        scan2MapOptimization();

        double t1 = time.toc("1");
        reg_time.push_back(t1);



        if (mode == ModeType::RELO) {
            
            saveRELOKeyFramesAndFactor();

        }
        else if (mode == ModeType::LIO) {
            
            saveLIOKeyFramesAndFactor();
        }
        else {
            ROS_ERROR("Invaild Mode Type. Please use 'LIO' or 'RELO' ... ");
            ros::shutdown();
        }



        double t2 = time.toc("2");

        opt_time.push_back(t2 - t1);

        total_time.push_back(t2);

        // std::cout << "total time: " << t2 << std::endl;

        // correctPoses();

        publishOdometry();

        publishFrames();

        timeLastProcessing = timeLaserInfoCur;


        // // XXX: without egocentric factor-graph optimization
        // if (timeLaserInfoCur - timeLastProcessing >= mappingProcessInterval) {

        //     if (mode == ModeType::RELO) {
        //         if (timeLaserInfoCur - timeLastProcessing < timeInterval) {
        //             return ;
        //         }
        //     }

        //     // TicToc time("Cloud Processing");

        //     updateInitialGuess();

        //     extractSurroundingKeyFrames();

        //     downsampleCurrentScan();

        //     scan2MapOptimization();

        //     saveLIOKeyFramesAndFactor();

        //     correctPoses();

        //     // time.toc("Cloud Processing");

        //     publishOdometry();

        //     publishFrames();

        //     timeLastProcessing = timeLaserInfoCur;
        // }
    }

    void pointAssociateToMap(PointType const * const pi, PointType * const po) {
        po->x = transPointAssociateToMap(0, 0) * pi->x + transPointAssociateToMap(0, 1) * pi->y + transPointAssociateToMap(0, 2) * pi->z + transPointAssociateToMap(0, 3);
        po->y = transPointAssociateToMap(1, 0) * pi->x + transPointAssociateToMap(1, 1) * pi->y + transPointAssociateToMap(1, 2) * pi->z + transPointAssociateToMap(1, 3);
        po->z = transPointAssociateToMap(2, 0) * pi->x + transPointAssociateToMap(2, 1) * pi->y + transPointAssociateToMap(2, 2) * pi->z + transPointAssociateToMap(2, 3);
        po->intensity = pi->intensity;
    }

    gtsam::Pose3 trans2gtsamPose(float transformIn[]) {
        return gtsam::Pose3(gtsam::Rot3::RzRyRx(transformIn[0], transformIn[1], transformIn[2]), 
                            gtsam::Point3(transformIn[3], transformIn[4], transformIn[5]));
    }

    Eigen::Affine3f pclPointToAffine3f(PointTypePose thisPoint) { 
        return pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z, thisPoint.roll, thisPoint.pitch, thisPoint.yaw);
    }

    Eigen::Affine3f trans2Affine3f(float transformIn[]) {
        return pcl::getTransformation(transformIn[3], transformIn[4], transformIn[5], transformIn[0], transformIn[1], transformIn[2]);
    }

    PointTypePose trans2PointTypePose(float transformIn[]) {
        PointTypePose thisPose6D;
        thisPose6D.x = transformIn[3];
        thisPose6D.y = transformIn[4];
        thisPose6D.z = transformIn[5];
        thisPose6D.roll  = transformIn[0];
        thisPose6D.pitch = transformIn[1];
        thisPose6D.yaw   = transformIn[2];
        return thisPose6D;
    }


    bool saveMapService(liloc::save_mapRequest& req, liloc::save_mapResponse& res) {
        string saveMapDirectory;

        cout << "****************************************************" << endl;
        cout << "Saving map to pcd files ..." << endl;

        saveMapDirectory = savePCDDirectory;

        std::cout << "Save destination: " << saveMapDirectory << endl;

        // create directory and remove old files;
        int unused = system((std::string("exec rm -r ") + saveMapDirectory).c_str());
        unused = system((std::string("mkdir -p ") + saveMapDirectory).c_str());

        if (req.savepath == 1) {
            pgSaveStream = std::fstream(saveMapDirectory + "/singlesession_posegraph.g2o", std::fstream::out);

            // save key frame transformations
            pcl::io::savePCDFileBinary(saveMapDirectory + "/trajectory.pcd", *cloudKeyPoses3D);
            pcl::io::savePCDFileBinary(saveMapDirectory + "/transformations.pcd", *cloudKeyPoses6D);
        
            for(auto& _line: vertices_str)
                pgSaveStream << _line << std::endl;

            for(auto& _line: edges_str)
                pgSaveStream << _line << std::endl;

            pgSaveStream.close();

            std::fstream kittiStream(saveMapDirectory + "/odom_kitti.txt", std::fstream::out);
            std::fstream tumStream(saveMapDirectory + "/odom_tum.txt", std::fstream::out);

            kittiStream.precision(15);
            tumStream.precision(15);

            for (size_t i = 0; i < cloudKeyPoses6D->size(); i++) {
                PointTypePose pose = cloudKeyPoses6D->points[i];

                Eigen::Matrix3f rot = eulerToRotation(pose.roll, pose.pitch, pose.yaw);

                kittiStream << rot(0, 0) << " " << rot(0, 1) << " " << rot(0, 2) << " " << pose.x << " "
                            << rot(1, 0) << " " << rot(1, 1) << " " << rot(1, 2) << " " << pose.y << " "
                            << rot(2, 0) << " " << rot(2, 1) << " " << rot(2, 2) << " " << pose.z << std::endl;
                
                Eigen::Matrix3d rot_d = rot.cast<double>();
                Eigen::Vector4d quat = rotationToQuaternion(rot_d);

                tumStream << ros_time_tum[i] << " " << pose.x << " " << pose.y << " " << pose.z << " "
                          << quat(0) << " " << quat(1) << " " << quat(2) << " " << quat(3) << std::endl;
            }

            std::fstream totalTime(saveMapDirectory + "/total_time.txt", std::fstream::out);
            totalTime.precision(5);
            for (auto t : total_time) {
                totalTime << t << std::endl;
            }
            totalTime.close();

            std::fstream regTime(saveMapDirectory + "/reg_time.txt", std::fstream::out);
            regTime.precision(5);
            for (auto t : reg_time) {
                regTime << t << std::endl;
            }
            regTime.close();

            std::fstream optTime(saveMapDirectory + "/opt_time.txt", std::fstream::out);
            optTime.precision(5);
            for (auto t : opt_time) {
                optTime << t << std::endl;
            }
            optTime.close();

        }

        if (req.savecloud == 1) {
            std::string savePcdDirectory = saveMapDirectory + "/PCDs";
            unused = system((std::string("mkdir -p ") + savePcdDirectory).c_str());

            pcl::PointCloud<PointType>::Ptr surfCloud(new pcl::PointCloud<PointType>());
            pcl::PointCloud<PointType>::Ptr surfCloudDS(new pcl::PointCloud<PointType>());
            pcl::PointCloud<PointType>::Ptr globalMapCloud(new pcl::PointCloud<PointType>());
            pcl::PointCloud<PointType>::Ptr globalMapCloudDS(new pcl::PointCloud<PointType>());

            if (req.resolution != 0) {
                cout << "\n\nSave resolution: " << req.resolution << endl;
                downSizeFilterSurf.setLeafSize(req.resolution, req.resolution, req.resolution);
            }

            for (int i = 0; i < (int)cloudKeyPoses3D->size(); i++) {
                surfCloud = surfCloudKeyFrames[i];

                if (req.resolution != 0) {
                    downSizeFilterSurf.setInputCloud(surfCloud);
                    downSizeFilterSurf.filter(*surfCloudDS);
                }

                std::string name_i = std::to_string(i) + ".pcd";
                pcl::io::savePCDFileBinary(savePcdDirectory + "/" + name_i, *surfCloudDS);

                *globalMapCloud   += *transformPointCloud(surfCloudKeyFrames[i],    &cloudKeyPoses6D->points[i]);
            }
            cout << "Processing feature cloud: " << cloudKeyPoses6D->size() << endl;
        
            if (req.resolution != 0) {
                downSizeFilterSurf.setInputCloud(globalMapCloud);
                downSizeFilterSurf.filter(*globalMapCloudDS);
            }
        
            pcl::io::savePCDFileBinary(saveMapDirectory + "/globalMap.pcd", *globalMapCloudDS);
        }

        int ret = 1;

        res.success = (ret == 1);

        downSizeFilterSurf.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);

        cout << "****************************************************" << endl;
        cout << "Saving map to pcd files completed\n" << endl;


        return true;
    }

    bool saveSessionService(liloc::save_sessionRequest& req, liloc::save_sessionResponse& res) {

        if (mode != ModeType::RELO) {
            ROS_ERROR("Not RELO Mode, Can't Save Session !");
            return false;
        }

        std::string saveSessionDir;

        cout << "****************************************************" << endl;
        cout << "Saving session to pcd files ..." << endl;

        saveSessionDir = saveSessionDirectory;

        std::cout << "Save destination: " << saveSessionDir << endl;

        int unused = system((std::string("exec rm -r ") + saveSessionDir).c_str());
        unused = system((std::string("mkdir -p ") + saveSessionDir).c_str());

        std::string savePcdDirectory = saveSessionDir + "/PCDs";
        unused = system((std::string("mkdir -p ") + savePcdDirectory).c_str());

        pgSaveStream = std::fstream(saveSessionDir + "/singlesession_posegraph.g2o", std::fstream::out);

        pcl::io::savePCDFileBinary(saveSessionDir + "/transformations.pcd", *data_loader->KeyPoses6D_);

        for (size_t i = 0; i < data_loader->KeyPoses6D_->size(); i++) {
            if (i == 0) {
                writeVertex(0, pclPointTogtsamPose3(data_loader->KeyPoses6D_->points[i]), vertices_str);
            }
            else {
                gtsam::Pose3 poseFrom = pclPointTogtsamPose3(data_loader->KeyPoses6D_->points[i - 1]);
                gtsam::Pose3 poseTo = pclPointTogtsamPose3(data_loader->KeyPoses6D_->points[i]);

                gtsam::Pose3 poseRel = poseFrom.between(poseTo);

                writeVertex(i, poseTo, vertices_str);
                writeEdge({i - 1, i}, poseRel, edges_str); 
            }
        }

        for(auto& _line: vertices_str)
            pgSaveStream << _line << std::endl;

        for(auto& _line: edges_str)
            pgSaveStream << _line << std::endl;

        pcl::PointCloud<PointType>::Ptr surfCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr surfCloudDS(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapCloudDS(new pcl::PointCloud<PointType>());

        if (req.resolution != 0) {
            cout << "\n\nSave resolution: " << req.resolution << endl;
            downSizeFilterSurf.setLeafSize(req.resolution, req.resolution, req.resolution);
        }

        for (size_t i = 0; i < data_loader->KeyPoses6D_->size(); i++) {
            surfCloud = data_loader->keyCloudVec_[i];

            if (req.resolution != 0) {
                downSizeFilterSurf.setInputCloud(surfCloud);
                downSizeFilterSurf.filter(*surfCloudDS);
            }

            std::string name_i = std::to_string(i) + ".pcd";
            pcl::io::savePCDFileBinary(savePcdDirectory + "/" + name_i, *surfCloudDS);

            *globalMapCloud   += *transformPointCloud(surfCloud, &data_loader->KeyPoses6D_->points[i]);
        }
        cout << "Processing feature cloud: " << data_loader->KeyPoses6D_->size() << endl;

        if (req.resolution != 0) {
            downSizeFilterSurf.setInputCloud(globalMapCloud);
            downSizeFilterSurf.filter(*globalMapCloudDS);
        }

        pcl::io::savePCDFileBinary(saveSessionDir + "/globalMap.pcd", *globalMapCloudDS);

        int ret = 1;

        res.success = (ret == 1);

        downSizeFilterSurf.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);

        cout << "****************************************************" << endl;
        cout << "Saving map to pcd files completed\n" << endl;

        pgSaveStream.close();

        return true;
    }

    void visualizeGlobalMapThread() {
        ros::Rate rate(1);
        while (ros::ok()){
            rate.sleep();
            publishGlobalMap();
        }
    }

    void drawLinePlot(const std::vector<double>& vec1, const std::vector<double>& vec2, const std::vector<double>& vec3, const std::string& windowName) {
        int maxLen = std::max({vec1.size(), vec2.size(), vec3.size()});

        double width = 800, height = 600;
        cv::Mat plotImg(height, width, CV_8UC3, cv::Scalar(255, 255, 255));

        double margin = 50;
        double maxVal = 200;

        cv::line(plotImg, cv::Point(margin, height - margin), cv::Point(width - margin, height - margin), cv::Scalar(0, 0, 0), 2);
        cv::line(plotImg, cv::Point(margin, margin), cv::Point(margin, height - margin), cv::Scalar(0, 0, 0), 2);

        cv::putText(plotImg, "Index", cv::Point(width / 2, height - 10), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 2);
        cv::putText(plotImg, "ms", cv::Point(10, margin / 2), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 2);

        for (int i = 0; i <= maxVal; i += 10) {
            double y = height - margin - (i * (height - 2 * margin) / maxVal);
            cv::line(plotImg, cv::Point(margin, y), cv::Point(width - margin, y), cv::Scalar(200, 200, 200), 1);
            cv::putText(plotImg, std::to_string(i), cv::Point(10, y), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1);
        }

        auto drawLine = [&](const std::vector<double>& vec, const cv::Scalar& color, const std::string& label, int labelPos) {
            for (size_t i = 1; i < vec.size(); ++i) {
                double x1 = margin + (i - 1) * (width - 2 * margin) / maxLen;
                double y1 = height - margin - (vec[i - 1] * (height - 2 * margin) / maxVal);
                double x2 = margin + i * (width - 2 * margin) / maxLen;
                double y2 = height - margin - (vec[i] * (height - 2 * margin) / maxVal);
                cv::line(plotImg, cv::Point(x1, y1), cv::Point(x2, y2), color, 2);
            }

            cv::putText(plotImg, label, cv::Point(width - margin + 10, labelPos), cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2);
        };

        drawLine(vec1, cv::Scalar(255, 0, 0), "Total Time", 100);
        drawLine(vec2, cv::Scalar(0, 255, 0), "LM Time", 130);
        drawLine(vec3, cv::Scalar(0, 0, 255), "EFGO Time", 160);

        cv::imshow(windowName, plotImg);            
        cv::waitKey(1);
    }

    void displayTime() {
        cv::namedWindow("Processing Times", cv::WINDOW_AUTOSIZE);

        ros::Rate rate(1);
        while (ros::ok()) {
            rate.sleep();

            drawLinePlot(total_time, reg_time, opt_time, "Processing Times");
        }

    }

    void publishGlobalMap() {
        pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyFrames;
        downSizeFilterGlobalMapKeyFrames.setLeafSize(globalMapVisualizationLeafSize, globalMapVisualizationLeafSize, globalMapVisualizationLeafSize); // for global map visualization

        if (mode == ModeType::RELO) {
            publishCloud(pubPriorGlobalMap, data_loader->globalMap_, timeLaserInfoStamp, mapFrame);
            publishCloud(pubPriorGlobalTrajectory, data_loader->KeyPoses6D_, timeLaserInfoStamp, mapFrame);
            publishCloud(pubPriorLocalSubmap, data_loader->usingSubMap_, timeLaserInfoStamp, mapFrame);
            publishCloud(pubPriorLocalSubmapCenteriod, data_loader->SubMapCenteriod_, timeLaserInfoStamp, mapFrame);
        
            visualizeLocalVertexAndEdge();
        }
        
        if (pubLaserCloudSurround.getNumSubscribers() == 0)
            return;

        if (cloudKeyPoses3D->points.empty() == true)
            return;

        pcl::KdTreeFLANN<PointType>::Ptr kdtreeGlobalMap(new pcl::KdTreeFLANN<PointType>());;
        pcl::PointCloud<PointType>::Ptr globalMapKeyPoses(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyPosesDS(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyFrames(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyFramesDS(new pcl::PointCloud<PointType>());

        std::vector<int> pointSearchIndGlobalMap;
        std::vector<float> pointSearchSqDisGlobalMap;

        mtx.lock();
        kdtreeGlobalMap->setInputCloud(cloudKeyPoses3D);
        kdtreeGlobalMap->radiusSearch(cloudKeyPoses3D->back(), globalMapVisualizationSearchRadius, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap, 0);
        mtx.unlock();

        for (int i = 0; i < (int)pointSearchIndGlobalMap.size(); ++i)
            globalMapKeyPoses->push_back(cloudKeyPoses3D->points[pointSearchIndGlobalMap[i]]);

        pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyPoses;
        downSizeFilterGlobalMapKeyPoses.setLeafSize(globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity); // for global map visualization
        downSizeFilterGlobalMapKeyPoses.setInputCloud(globalMapKeyPoses);
        downSizeFilterGlobalMapKeyPoses.filter(*globalMapKeyPosesDS);
        for(auto& pt : globalMapKeyPosesDS->points) {
            kdtreeGlobalMap->nearestKSearch(pt, 1, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap);
            pt.intensity = cloudKeyPoses3D->points[pointSearchIndGlobalMap[0]].intensity;
        }

        for (int i = 0; i < (int)globalMapKeyPosesDS->size(); ++i) {
            if (pointDistance(globalMapKeyPosesDS->points[i], cloudKeyPoses3D->back()) > globalMapVisualizationSearchRadius)
                continue;
            int thisKeyInd = (int)globalMapKeyPosesDS->points[i].intensity;
            *globalMapKeyFrames += *transformPointCloud(surfCloudKeyFrames[thisKeyInd],    &cloudKeyPoses6D->points[thisKeyInd]);
        }

        downSizeFilterGlobalMapKeyFrames.setInputCloud(globalMapKeyFrames);
        downSizeFilterGlobalMapKeyFrames.filter(*globalMapKeyFramesDS);
        publishCloud(pubLaserCloudSurround, globalMapKeyFramesDS, timeLaserInfoStamp, odometryFrame);
    }

    void initialposeHandler(const geometry_msgs::PoseWithCovarianceStampedConstPtr& poseMsg) {
        ROS_INFO("Initial pose received .");

        const auto& p = poseMsg->pose.pose.position;
        const auto& q = poseMsg->pose.pose.orientation;

        Eigen::Matrix3d rot = quaternionToRotation(Eigen::Vector4d(q.x, q.y, q.z, q.w));
        Eigen::Vector3d euler = RotMtoEuler(rot);

        transformTobeMappedInit[0] = 0.0;
        transformTobeMappedInit[1] = 0.0;
        transformTobeMappedInit[2] = euler(2);
        transformTobeMappedInit[3] = p.x;
        transformTobeMappedInit[4] = p.y;
        transformTobeMappedInit[5] = 0.0;

        std::cout << "position: " << " ( " << transformTobeMappedInit[3] << ", " << transformTobeMappedInit[4] << ", " << transformTobeMappedInit[5] << " )" << std::endl;

        systemInitialized = true;
    }

    void visualizeLocalVertexAndEdge() {
        if (data_loader->usingVertexes_->size() == 0) {
            return ;
        }

        visualization_msgs::MarkerArray markerArray;

        // loop nodes
        visualization_msgs::Marker markerNode;
        markerNode.header.frame_id = odometryFrame;
        markerNode.header.stamp = timeLaserInfoStamp;
        markerNode.action = visualization_msgs::Marker::ADD;
        markerNode.type = visualization_msgs::Marker::SPHERE_LIST;
        markerNode.ns = "vertex";
        markerNode.id = 0;
        markerNode.pose.orientation.w = 1;
        markerNode.scale.x = 0.3; markerNode.scale.y = 0.3; markerNode.scale.z = 0.3; 
        markerNode.color.r = 0; markerNode.color.g = 0.8; markerNode.color.b = 1;
        markerNode.color.a = 1;

        // loop edges
        visualization_msgs::Marker markerEdge;
        markerEdge.header.frame_id = odometryFrame;
        markerEdge.header.stamp = timeLaserInfoStamp;
        markerEdge.action = visualization_msgs::Marker::ADD;
        markerEdge.type = visualization_msgs::Marker::LINE_LIST;
        markerEdge.ns = "edge";
        markerEdge.id = 1;
        markerEdge.pose.orientation.w = 1;
        markerEdge.scale.x = 0.1;
        markerEdge.color.r = 0.9; markerEdge.color.g = 0.9; markerEdge.color.b = 0;
        markerEdge.color.a = 1;

        PointTypePose cur_pose = trans2PointTypePose(transformTobeMapped);

        for (int i = 0; i < data_loader->usingVertexes_->size(); i++) {
            geometry_msgs::Point p;
            p.x = cur_pose.x;
            p.y = cur_pose.y;
            p.z = cur_pose.z;
            markerNode.points.push_back(p);
            markerEdge.points.push_back(p);

            p.x = data_loader->usingVertexes_->points[i].x;
            p.y = data_loader->usingVertexes_->points[i].y;
            p.z = data_loader->usingVertexes_->points[i].z;
            markerNode.points.push_back(p);
            markerEdge.points.push_back(p);
        }

        markerArray.markers.push_back(markerNode);
        markerArray.markers.push_back(markerEdge);
        pubLocalVertexAndEdge.publish(markerArray);
    }

    void updateInitialGuess() {
        incrementalOdometryAffineFront = trans2Affine3f(transformTobeMapped);

        static Eigen::Affine3f lastImuTransformation;
        if (cloudKeyPoses3D->points.empty()) {
            if (mode == ModeType::LIO) {
                transformTobeMapped[0] = cloudInfo.imuRollInit;
                transformTobeMapped[1] = cloudInfo.imuPitchInit;
                transformTobeMapped[2] = cloudInfo.imuYawInit;

                lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit); // save imu before return;
            }
            else if (mode == ModeType::RELO) {
                Eigen::Affine3f transImuInit = pcl::getTransformation(0, 0, 0, cloudInfo.initialGuessRoll, cloudInfo.initialGuessPitch, cloudInfo.initialGuessYaw);
                Eigen::Affine3f transCbInit = pcl::getTransformation(transformTobeMappedInit[3], transformTobeMappedInit[4], transformTobeMappedInit[5], 
                                                                     transformTobeMappedInit[0], transformTobeMappedInit[1], transformTobeMappedInit[2]);
                
                Eigen::Affine3f transInit = transCbInit * transImuInit;
                pcl::getTranslationAndEulerAngles(transInit, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], 
                                                              transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);

                lastImuTransformation = pcl::getTransformation(0, 0, 0, transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]); 
            }
            else {
                ROS_ERROR("Ivaild mode type !");
                ros::shutdown();
            }
            
            return;
        }

        // use imu pre-integration estimation for pose guess
        static bool lastImuPreTransAvailable = false;
        static Eigen::Affine3f lastImuPreTransformation;
        if (cloudInfo.odomAvailable == true) {
            Eigen::Affine3f transBack = pcl::getTransformation(cloudInfo.initialGuessX,    cloudInfo.initialGuessY,     cloudInfo.initialGuessZ, 
                                                               cloudInfo.initialGuessRoll, cloudInfo.initialGuessPitch, cloudInfo.initialGuessYaw);
            if (lastImuPreTransAvailable == false) {
                lastImuPreTransformation = transBack;
                lastImuPreTransAvailable = true;
            } 
            else {
                Eigen::Affine3f transIncre = lastImuPreTransformation.inverse() * transBack;
                Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);
                Eigen::Affine3f transFinal = transTobe * transIncre;
                pcl::getTranslationAndEulerAngles(transFinal, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], 
                                                              transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);

                lastImuPreTransformation = transBack;

                lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit); // save imu before return;
                return;
            }
        }

        // use imu incremental estimation for pose guess (only rotation)
        if (cloudInfo.imuAvailable == true && imuType)
        {
            Eigen::Affine3f transBack = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit);
            Eigen::Affine3f transIncre = lastImuTransformation.inverse() * transBack;

            Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);
            Eigen::Affine3f transFinal = transTobe * transIncre;
            pcl::getTranslationAndEulerAngles(transFinal, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], 
                                                        transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);

            lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit); // save imu before return;
            return;
        }
    }

    void extractNearby() {
        pcl::PointCloud<PointType>::Ptr surroundingKeyPoses(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr surroundingKeyPosesDS(new pcl::PointCloud<PointType>());
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;

        kdtreeSurroundingKeyPoses->setInputCloud(cloudKeyPoses3D);
        kdtreeSurroundingKeyPoses->radiusSearch(cloudKeyPoses3D->back(), (double)surroundingKeyframeSearchRadius, pointSearchInd, pointSearchSqDis);
        for (int i = 0; i < (int)pointSearchInd.size(); ++i)
        {
            int id = pointSearchInd[i];
            surroundingKeyPoses->push_back(cloudKeyPoses3D->points[id]);
        }

        downSizeFilterSurroundingKeyPoses.setInputCloud(surroundingKeyPoses);
        downSizeFilterSurroundingKeyPoses.filter(*surroundingKeyPosesDS);
        for(auto& pt : surroundingKeyPosesDS->points)
        {
            kdtreeSurroundingKeyPoses->nearestKSearch(pt, 1, pointSearchInd, pointSearchSqDis);
            pt.intensity = cloudKeyPoses3D->points[pointSearchInd[0]].intensity;
        }

        int numPoses = cloudKeyPoses3D->size();
        for (int i = numPoses-1; i >= 0; --i)
        {
            if (timeLaserInfoCur - cloudKeyPoses6D->points[i].time < 5.0)
                surroundingKeyPosesDS->push_back(cloudKeyPoses3D->points[i]);
            else
                break;
        }

        extractCloud(surroundingKeyPosesDS);
    }

    void extractCloud(pcl::PointCloud<PointType>::Ptr cloudToExtract) {
        laserCloudSurfFromMap->clear(); 
        for (int i = 0; i < (int)cloudToExtract->size(); ++i) {
            if (pointDistance(cloudToExtract->points[i], cloudKeyPoses3D->back()) > surroundingKeyframeSearchRadius)
                continue;

            int thisKeyInd = (int)cloudToExtract->points[i].intensity;
            if (laserCloudMapContainer.find(thisKeyInd) != laserCloudMapContainer.end()) {
                *laserCloudSurfFromMap += laserCloudMapContainer[thisKeyInd].second;
            } 
            else {
                pcl::PointCloud<PointType> laserCloudCornerTemp;
                pcl::PointCloud<PointType> laserCloudSurfTemp = *transformPointCloud(surfCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]);
                *laserCloudSurfFromMap   += laserCloudSurfTemp;
                laserCloudMapContainer[thisKeyInd] = make_pair(laserCloudCornerTemp, laserCloudSurfTemp);
            }
        }

        downSizeFilterLocalMapSurf.setInputCloud(laserCloudSurfFromMap);
        downSizeFilterLocalMapSurf.filter(*laserCloudSurfFromMapDS);
        laserCloudSurfFromMapDSNum = laserCloudSurfFromMapDS->size();

        // clear map cache if too large
        if (laserCloudMapContainer.size() > 1000)
            laserCloudMapContainer.clear();
    }

    void extractSurroundingKeyFrames() {
        if (cloudKeyPoses3D->points.empty() == true)
            return; 

        extractNearby();
    }

    void downsampleCurrentScan() {
        laserCloudSurfLastDS->clear();
        downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
        downSizeFilterSurf.filter(*laserCloudSurfLastDS);
        laserCloudSurfLastDSNum = laserCloudSurfLastDS->size();
    }

    void updatePointAssociateToMap() {
        transPointAssociateToMap = trans2Affine3f(transformTobeMapped);
    }

    void surfOptimization() {
        updatePointAssociateToMap();

        #pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < laserCloudSurfLastDSNum; i = i + 2) {
            PointType pointOri, pointSel, coeff;
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;

            pointOri = laserCloudSurfLastDS->points[i];
            pointAssociateToMap(&pointOri, &pointSel); 
            kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

            Eigen::Matrix<float, 5, 3> matA0;
            Eigen::Matrix<float, 5, 1> matB0;
            Eigen::Vector3f matX0;

            matA0.setZero();
            matB0.fill(-1);
            matX0.setZero();

            if (pointSearchSqDis[4] < 1.0) {
                for (int j = 0; j < 5; j++) {
                    matA0(j, 0) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].x;
                    matA0(j, 1) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].y;
                    matA0(j, 2) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].z;
                }

                matX0 = matA0.colPivHouseholderQr().solve(matB0);

                float pa = matX0(0, 0);
                float pb = matX0(1, 0);
                float pc = matX0(2, 0);
                float pd = 1;

                float ps = sqrt(pa * pa + pb * pb + pc * pc);
                pa /= ps; pb /= ps; pc /= ps; pd /= ps;

                bool planeValid = true;
                for (int j = 0; j < 5; j++) {
                    if (fabs(pa * laserCloudSurfFromMapDS->points[pointSearchInd[j]].x +
                             pb * laserCloudSurfFromMapDS->points[pointSearchInd[j]].y +
                             pc * laserCloudSurfFromMapDS->points[pointSearchInd[j]].z + pd) > 0.2) {
                        planeValid = false;
                        break;
                    }
                }

                if (planeValid) {
                    float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

                    float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointOri.x * pointOri.x
                            + pointOri.y * pointOri.y + pointOri.z * pointOri.z));

                    coeff.x = s * pa;
                    coeff.y = s * pb;
                    coeff.z = s * pc;
                    coeff.intensity = s * pd2;

                    if (s > 0.1) {
                        laserCloudOriSurfVec[i] = pointOri;
                        coeffSelSurfVec[i] = coeff;
                        laserCloudOriSurfFlag[i] = true;
                    }
                }
            }
        }
    }

    void combineOptimizationCoeffs() {
        for (int i = 0; i < laserCloudSurfLastDSNum; ++i){
            if (laserCloudOriSurfFlag[i] == true){
                laserCloudOri->push_back(laserCloudOriSurfVec[i]);
                coeffSel->push_back(coeffSelSurfVec[i]);
            }
        }

        std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(), false);
    }

    bool LMOptimization(int iterCount) {
        // This optimization is from the original loam_velodyne by Ji Zhang, need to cope with coordinate transformation
        // lidar <- camera      ---     camera <- lidar
        // x = z                ---     x = y
        // y = x                ---     y = z
        // z = y                ---     z = x
        // roll = yaw           ---     roll = pitch
        // pitch = roll         ---     pitch = yaw
        // yaw = pitch          ---     yaw = roll

        // lidar -> camera
        float srx = sin(transformTobeMapped[2]);
        float crx = cos(transformTobeMapped[2]);
        float sry = sin(transformTobeMapped[1]);
        float cry = cos(transformTobeMapped[1]);
        float srz = sin(transformTobeMapped[0]);
        float crz = cos(transformTobeMapped[0]);

        int laserCloudSelNum = laserCloudOri->size();
        if (laserCloudSelNum < 50) {
            return false;
        }

        cv::Mat matA(laserCloudSelNum, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matAt(6, laserCloudSelNum, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matB(laserCloudSelNum, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));

        PointType pointOri, coeff;

        for (int i = 0; i < laserCloudSelNum; i++) {
            // lidar -> camera
            pointOri.x = laserCloudOri->points[i].x;
            pointOri.y = laserCloudOri->points[i].y;
            pointOri.z = laserCloudOri->points[i].z;

            // lidar -> camera
            coeff.x = coeffSel->points[i].x;
            coeff.y = coeffSel->points[i].y;
            coeff.z = coeffSel->points[i].z;
            coeff.intensity = coeffSel->points[i].intensity;

            // in camera
            float arx = (-srx * cry * pointOri.x - (srx * sry * srz + crx * crz) * pointOri.y + (crx * srz - srx * sry * crz) * pointOri.z) * coeff.x
                      + (crx * cry * pointOri.x - (srx * crz - crx * sry * srz) * pointOri.y + (crx * sry * crz + srx * srz) * pointOri.z) * coeff.y;

            float ary = (-crx * sry * pointOri.x + crx * cry * srz * pointOri.y + crx * cry * crz * pointOri.z) * coeff.x
                      + (-srx * sry * pointOri.x + srx * sry * srz * pointOri.y + srx * cry * crz * pointOri.z) * coeff.y
                      + (-cry * pointOri.x - sry * srz * pointOri.y - sry * crz * pointOri.z) * coeff.z;

            float arz = ((crx * sry * crz + srx * srz) * pointOri.y + (srx * crz - crx * sry * srz) * pointOri.z) * coeff.x
                      + ((-crx * srz + srx * sry * crz) * pointOri.y + (-srx * sry * srz - crx * crz) * pointOri.z) * coeff.y
                      + (cry * crz * pointOri.y - cry * srz * pointOri.z) * coeff.z;
              
            // camera -> lidar
            matA.at<float>(i, 0) = arz;
            matA.at<float>(i, 1) = ary;
            matA.at<float>(i, 2) = arx;
            matA.at<float>(i, 3) = coeff.x;
            matA.at<float>(i, 4) = coeff.y;
            matA.at<float>(i, 5) = coeff.z;
            matB.at<float>(i, 0) = -coeff.intensity;
        }

        cv::transpose(matA, matAt);
        matAtA = matAt * matA;
        matAtB = matAt * matB;
        cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

        if (iterCount == 0) {

            cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

            cv::eigen(matAtA, matE, matV);
            matV.copyTo(matV2);

            isDegenerate = false;
            float eignThre[6] = {100, 100, 100, 100, 100, 100};
            for (int i = 5; i >= 0; i--) {
                if (matE.at<float>(0, i) < eignThre[i]) {
                    for (int j = 0; j < 6; j++) {
                        matV2.at<float>(i, j) = 0;
                    }
                    isDegenerate = true;
                } else {
                    break;
                }
            }
            matP = matV.inv() * matV2;
        }

        if (isDegenerate) {
            cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
            matX.copyTo(matX2);
            matX = matP * matX2;
        }

        transformTobeMapped[0] += matX.at<float>(0, 0);
        transformTobeMapped[1] += matX.at<float>(1, 0);
        transformTobeMapped[2] += matX.at<float>(2, 0);
        transformTobeMapped[3] += matX.at<float>(3, 0);
        transformTobeMapped[4] += matX.at<float>(4, 0);
        transformTobeMapped[5] += matX.at<float>(5, 0);

        float deltaR = sqrt(pow(pcl::rad2deg(matX.at<float>(0, 0)), 2) +
                            pow(pcl::rad2deg(matX.at<float>(1, 0)), 2) +
                            pow(pcl::rad2deg(matX.at<float>(2, 0)), 2));

        float deltaT = sqrt(pow(matX.at<float>(3, 0) * 100, 2) +
                            pow(matX.at<float>(4, 0) * 100, 2) +
                            pow(matX.at<float>(5, 0) * 100, 2));

        if (deltaR < 0.05 && deltaT < 0.05) {
            return true;
        }
        return false;
    }

    void scan2MapOptimization() {
        if (cloudKeyPoses3D->points.empty())
            return;

        if (laserCloudSurfLastDSNum > 30) {
            kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMapDS);

            for (int iterCount = 0; iterCount < 20; iterCount++) {
                laserCloudOri->clear();
                coeffSel->clear();

                surfOptimization();

                combineOptimizationCoeffs();

                if (LMOptimization(iterCount) == true)
                    break;              
            }

            transformUpdate();
        } 
        else {
            ROS_WARN("Not enough features! Only %d planar features available.", laserCloudSurfLastDSNum);
        }
    }

    void transformUpdate() {
        if (cloudInfo.imuAvailable == true && imuType) {
            if (std::abs(cloudInfo.imuPitchInit) < 1.4) {
                double imuWeight = imuRPYWeight;
                tf::Quaternion imuQuaternion;
                tf::Quaternion transformQuaternion;
                double rollMid, pitchMid, yawMid;

                transformQuaternion.setRPY(transformTobeMapped[0], 0, 0);
                imuQuaternion.setRPY(cloudInfo.imuRollInit, 0, 0);
                tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                transformTobeMapped[0] = rollMid;

                transformQuaternion.setRPY(0, transformTobeMapped[1], 0);
                imuQuaternion.setRPY(0, cloudInfo.imuPitchInit, 0);
                tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                transformTobeMapped[1] = pitchMid;
            }
        }

        transformTobeMapped[0] = constraintTransformation(transformTobeMapped[0], rotation_tollerance);
        transformTobeMapped[1] = constraintTransformation(transformTobeMapped[1], rotation_tollerance);
        transformTobeMapped[5] = constraintTransformation(transformTobeMapped[5], z_tollerance);

        incrementalOdometryAffineBack = trans2Affine3f(transformTobeMapped);
    }

    float constraintTransformation(float value, float limit) {
        if (value < -limit)
            value = -limit;
        if (value > limit)
            value = limit;

        return value;
    }

    bool saveFrame() {
        if (cloudKeyPoses3D->points.empty())
            return true;

        Eigen::Affine3f transStart = pclPointToAffine3f(cloudKeyPoses6D->back());
        Eigen::Affine3f transFinal = pcl::getTransformation(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], 
                                                            transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        Eigen::Affine3f transBetween = transStart.inverse() * transFinal;

        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(transBetween, x, y, z, roll, pitch, yaw);

        if (abs(roll)  < surroundingkeyframeAddingAngleThreshold &&
            abs(pitch) < surroundingkeyframeAddingAngleThreshold && 
            abs(yaw)   < surroundingkeyframeAddingAngleThreshold &&
            sqrt(x * x + y * y + z * z) < surroundingkeyframeAddingDistThreshold)
            return false;

        return true;
    }

    void addOdomFactor() {
        if (cloudKeyPoses3D->points.empty()) {
            noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-2, 1e-2, M_PI * M_PI, 1e8, 1e8, 1e8).finished()); // rad*rad, meter*meter
            gtSAMgraph.add(PriorFactor<Pose3>(0, trans2gtsamPose(transformTobeMapped), priorNoise));
            initialEstimate.insert(0, trans2gtsamPose(transformTobeMapped));

            writeVertex(0, trans2gtsamPose(transformTobeMapped), vertices_str);
        }
        else {
            noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
            gtsam::Pose3 poseFrom = pclPointTogtsamPose3(cloudKeyPoses6D->points.back());
            gtsam::Pose3 poseTo   = trans2gtsamPose(transformTobeMapped);
            gtsam::Pose3 poseRel = poseFrom.between(poseTo);

            initialEstimate.insert(cloudKeyPoses3D->size(), poseTo);
            gtSAMgraph.add(BetweenFactor<Pose3>(cloudKeyPoses3D->size() - 1, cloudKeyPoses3D->size(), poseRel, odometryNoise));

            writeVertex(cloudKeyPoses3D->size(), poseTo, vertices_str);
            writeEdge({cloudKeyPoses3D->size() - 1, cloudKeyPoses3D->size()}, poseRel, edges_str); 
        }
    }

    void addScanMatchingFactor() {
        if (cloudKeyPoses3D->points.empty()) {
            return ;
        }

        PointTypePose cur_pose =  trans2PointTypePose(transformTobeMapped);

        int submap_id;
        data_loader->searchNearestSubMapAndVertex(cur_pose, submap_id);

        if (data_loader->usingVertexes_->size() < 2) {
            return ;
        }

        registration->setInputTarget(data_loader->usingSubMap_);

        registration->setInputSource(laserCloudSurfLast);

        Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();
        Eigen::Matrix3f rotation = eulerToRotation(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        Eigen::Vector3f translation(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5]);
        init_guess.block(0, 0, 3, 3) = rotation;
        init_guess.block(0, 3, 3, 1) = translation;

        CloudPtr aligned(new Cloud());
        registration->align(*aligned, init_guess);

        Eigen::Matrix4f transform;
        transform = registration->getFinalTransformation();

        std::cout << std::endl;
        std::cout << init_guess << std::endl;

        std::cout << transform << std::endl;
        std::cout << std::endl;

        Eigen::Vector3f euler = RotMtoEuler(Eigen::Matrix3f(transform.block(0, 0, 3, 3)));
        Eigen::Vector3f xyz = transform.block(0, 3, 3, 1);

        noiseModel::Diagonal::shared_ptr matchNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
        gtsam::Pose3 poseFrom = pclPointTogtsamPose3(cloudKeyPoses6D->points.back());
        gtsam::Pose3 poseTo(gtsam::Rot3::RzRyRx(euler(0), euler(1), euler(2)), gtsam::Point3(xyz(0), xyz(1), xyz(2)));

        gtsam::Pose3 poseRel = poseFrom.between(poseTo);

        gtSAMgraph.add(BetweenFactor<Pose3>(cloudKeyPoses3D->size() - 1, cloudKeyPoses3D->size(), poseFrom.between(poseTo), matchNoise));
    }

    void saveRELOKeyFramesAndFactor() {

        // if (saveFrame() == false)
        //     return;


        optimize->getCurrentPose(cloudKeyPoses3D->points.size(), trans2PointTypePose(transformTobeMapped), laserCloudSurfLast);
        optimize->addOdomFactors();

        ros_time_tum.push_back(timeLaserInfoCur);

        PointType thisPose3D;
        PointTypePose thisPose6D;
        Pose3 latestEstimate;

        int latestId = genGlobalNodeIdx(optimize->session_id, cloudKeyPoses3D->points.size());
        latestEstimate = optimize->isamCurrentEstimate_.at<Pose3>(latestId);

        thisPose3D.x = latestEstimate.translation().x();
        thisPose3D.y = latestEstimate.translation().y();
        thisPose3D.z = latestEstimate.translation().z();
        thisPose3D.intensity = cloudKeyPoses3D->size(); // this can be used as index
        cloudKeyPoses3D->push_back(thisPose3D);

        thisPose6D.x = thisPose3D.x;
        thisPose6D.y = thisPose3D.y;
        thisPose6D.z = thisPose3D.z;
        thisPose6D.intensity = thisPose3D.intensity ; // this can be used as index
        thisPose6D.roll  = latestEstimate.rotation().roll();
        thisPose6D.pitch = latestEstimate.rotation().pitch();
        thisPose6D.yaw   = latestEstimate.rotation().yaw();
        thisPose6D.time = timeLaserInfoCur;
        cloudKeyPoses6D->push_back(thisPose6D);

        poseCovariance = optimize->isam_->marginalCovariance(latestId);

        // save updated transform
        transformTobeMapped[0] = latestEstimate.rotation().roll();
        transformTobeMapped[1] = latestEstimate.rotation().pitch();
        transformTobeMapped[2] = latestEstimate.rotation().yaw();
        transformTobeMapped[3] = latestEstimate.translation().x();
        transformTobeMapped[4] = latestEstimate.translation().y();
        transformTobeMapped[5] = latestEstimate.translation().z();

        // save all the received edge and surf points
        pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame(new pcl::PointCloud<PointType>());
        pcl::copyPointCloud(*laserCloudSurfLastDS, *thisSurfKeyFrame);

        // save key frame cloud
        surfCloudKeyFrames.push_back(thisSurfKeyFrame);

        // save path for visualization
        updatePath(thisPose6D);
    }

    void saveLIOKeyFramesAndFactor() {
        if (saveFrame() == false)
            return;

        ros_time_tum.push_back(timeLaserInfoCur);

        // odom factor
        addOdomFactor();


        // cout << "****************************************************" << endl;
        // gtSAMgraph.print("GTSAM Graph:\n");

        // update iSAM
        isam->update(gtSAMgraph, initialEstimate);
        isam->update();

        gtSAMgraph.resize(0);
        initialEstimate.clear();

        //save key poses
        PointType thisPose3D;
        PointTypePose thisPose6D;
        Pose3 latestEstimate;

        isamCurrentEstimate = isam->calculateEstimate();
        latestEstimate = isamCurrentEstimate.at<Pose3>(isamCurrentEstimate.size()-1);

        
        // cout << "****************************************************" << endl;
        // isamCurrentEstimate.print("Current estimate: ");

        thisPose3D.x = latestEstimate.translation().x();
        thisPose3D.y = latestEstimate.translation().y();
        thisPose3D.z = latestEstimate.translation().z();
        thisPose3D.intensity = cloudKeyPoses3D->size(); // this can be used as index
        cloudKeyPoses3D->push_back(thisPose3D);

        thisPose6D.x = thisPose3D.x;
        thisPose6D.y = thisPose3D.y;
        thisPose6D.z = thisPose3D.z;
        thisPose6D.intensity = thisPose3D.intensity ; // this can be used as index
        thisPose6D.roll  = latestEstimate.rotation().roll();
        thisPose6D.pitch = latestEstimate.rotation().pitch();
        thisPose6D.yaw   = latestEstimate.rotation().yaw();
        thisPose6D.time = timeLaserInfoCur;
        cloudKeyPoses6D->push_back(thisPose6D);

        // cout << "****************************************************" << endl;
        // cout << "Pose covariance:" << endl;
        // cout << isam->marginalCovariance(isamCurrentEstimate.size()-1) << endl << endl;
        poseCovariance = isam->marginalCovariance(isamCurrentEstimate.size() - 1);

        // save updated transform
        transformTobeMapped[0] = latestEstimate.rotation().roll();
        transformTobeMapped[1] = latestEstimate.rotation().pitch();
        transformTobeMapped[2] = latestEstimate.rotation().yaw();
        transformTobeMapped[3] = latestEstimate.translation().x();
        transformTobeMapped[4] = latestEstimate.translation().y();
        transformTobeMapped[5] = latestEstimate.translation().z();

        // save all the received edge and surf points
        pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame(new pcl::PointCloud<PointType>());
        pcl::copyPointCloud(*laserCloudSurfLastDS, *thisSurfKeyFrame);

        // save key frame cloud
        surfCloudKeyFrames.push_back(thisSurfKeyFrame);

        // save path for visualization
        updatePath(thisPose6D);
    }

    void correctPoses() {
        if (cloudKeyPoses3D->points.empty())
            return;

        if (aLoopIsClosed == true) {
            // clear map cache
            laserCloudMapContainer.clear();
            // clear path
            globalPath.poses.clear();
            // update key poses
            int numPoses = isamCurrentEstimate.size();
            for (int i = 0; i < numPoses; ++i) {
                cloudKeyPoses3D->points[i].x = isamCurrentEstimate.at<Pose3>(i).translation().x();
                cloudKeyPoses3D->points[i].y = isamCurrentEstimate.at<Pose3>(i).translation().y();
                cloudKeyPoses3D->points[i].z = isamCurrentEstimate.at<Pose3>(i).translation().z();

                cloudKeyPoses6D->points[i].x = cloudKeyPoses3D->points[i].x;
                cloudKeyPoses6D->points[i].y = cloudKeyPoses3D->points[i].y;
                cloudKeyPoses6D->points[i].z = cloudKeyPoses3D->points[i].z;
                cloudKeyPoses6D->points[i].roll  = isamCurrentEstimate.at<Pose3>(i).rotation().roll();
                cloudKeyPoses6D->points[i].pitch = isamCurrentEstimate.at<Pose3>(i).rotation().pitch();
                cloudKeyPoses6D->points[i].yaw   = isamCurrentEstimate.at<Pose3>(i).rotation().yaw();

                updatePath(cloudKeyPoses6D->points[i]);
            }

            aLoopIsClosed = false;
        }
    }

    void updatePath(const PointTypePose& pose_in) {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time().fromSec(pose_in.time);
        pose_stamped.header.frame_id = odometryFrame;
        pose_stamped.pose.position.x = pose_in.x;
        pose_stamped.pose.position.y = pose_in.y;
        pose_stamped.pose.position.z = pose_in.z;
        tf::Quaternion q = tf::createQuaternionFromRPY(pose_in.roll, pose_in.pitch, pose_in.yaw);
        pose_stamped.pose.orientation.x = q.x();
        pose_stamped.pose.orientation.y = q.y();
        pose_stamped.pose.orientation.z = q.z();
        pose_stamped.pose.orientation.w = q.w();

        globalPath.poses.push_back(pose_stamped);
    }

    void publishOdometry() {
        // Publish odometry for ROS (global)
        nav_msgs::Odometry laserOdometryROS;
        laserOdometryROS.header.stamp = timeLaserInfoStamp;
        laserOdometryROS.header.frame_id = odometryFrame;
        laserOdometryROS.child_frame_id = "odom_mapping";
        laserOdometryROS.pose.pose.position.x = transformTobeMapped[3];
        laserOdometryROS.pose.pose.position.y = transformTobeMapped[4];
        laserOdometryROS.pose.pose.position.z = transformTobeMapped[5];
        laserOdometryROS.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        pubLaserOdometryGlobal.publish(laserOdometryROS);
        
        // Publish TF
        static tf::TransformBroadcaster br;
        tf::Transform t_odom_to_lidar = tf::Transform(tf::createQuaternionFromRPY(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]),
                                                      tf::Vector3(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5]));
        tf::StampedTransform trans_odom_to_lidar = tf::StampedTransform(t_odom_to_lidar, timeLaserInfoStamp, odometryFrame, "lidar_link");
        br.sendTransform(trans_odom_to_lidar);

        // Publish odometry for ROS (incremental)
        static bool lastIncreOdomPubFlag = false;
        static nav_msgs::Odometry laserOdomIncremental; // incremental odometry msg
        static Eigen::Affine3f increOdomAffine; // incremental odometry in affine
        if (lastIncreOdomPubFlag == false) {
            lastIncreOdomPubFlag = true;
            laserOdomIncremental = laserOdometryROS;
            increOdomAffine = trans2Affine3f(transformTobeMapped);
        } 
        else {
            Eigen::Affine3f affineIncre = incrementalOdometryAffineFront.inverse() * incrementalOdometryAffineBack;
            increOdomAffine = increOdomAffine * affineIncre;

            float x, y, z, roll, pitch, yaw;
            pcl::getTranslationAndEulerAngles (increOdomAffine, x, y, z, roll, pitch, yaw);

            if (cloudInfo.imuAvailable == true && imuType) {
                if (std::abs(cloudInfo.imuPitchInit) < 1.4) {
                    double imuWeight = 0.1;
                    tf::Quaternion imuQuaternion;
                    tf::Quaternion transformQuaternion;
                    double rollMid, pitchMid, yawMid;

                    // slerp roll
                    transformQuaternion.setRPY(roll, 0, 0);
                    imuQuaternion.setRPY(cloudInfo.imuRollInit, 0, 0);
                    tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                    roll = rollMid;

                    // slerp pitch
                    transformQuaternion.setRPY(0, pitch, 0);
                    imuQuaternion.setRPY(0, cloudInfo.imuPitchInit, 0);
                    tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                    pitch = pitchMid;
                }
            }

            laserOdomIncremental.header.stamp = timeLaserInfoStamp;
            laserOdomIncremental.header.frame_id = odometryFrame;
            laserOdomIncremental.child_frame_id = "odom_mapping";
            laserOdomIncremental.pose.pose.position.x = x;
            laserOdomIncremental.pose.pose.position.y = y;
            laserOdomIncremental.pose.pose.position.z = z;
            laserOdomIncremental.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);

            if (isDegenerate)
                laserOdomIncremental.pose.covariance[0] = 1;
            else
                laserOdomIncremental.pose.covariance[0] = 0;
        }

        pubLaserOdometryIncremental.publish(laserOdomIncremental);
    }

    void publishFrames() {
        if (cloudKeyPoses3D->points.empty())
            return;

        // publish key poses
        publishCloud(pubKeyPoses, cloudKeyPoses3D, timeLaserInfoStamp, odometryFrame);

        // Publish surrounding key frames
        publishCloud(pubRecentKeyFrames, laserCloudSurfFromMapDS, timeLaserInfoStamp, odometryFrame);

        // publish registered key frame
        if (pubRecentKeyFrame.getNumSubscribers() != 0) {
            pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
            PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
            *cloudOut += *transformPointCloud(laserCloudSurfLastDS,    &thisPose6D);
            publishCloud(pubRecentKeyFrame, cloudOut, timeLaserInfoStamp, odometryFrame);
        }

        // publish registered high-res raw cloud
        if (pubCloudRegisteredRaw.getNumSubscribers() != 0) {
            pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
            pcl::fromROSMsg(cloudInfo.cloud_deskewed, *cloudOut);
            PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
            *cloudOut = *transformPointCloud(cloudOut,  &thisPose6D);
            publishCloud(pubCloudRegisteredRaw, cloudOut, timeLaserInfoStamp, odometryFrame);
        }

        // publish path
        if (pubPath.getNumSubscribers() != 0) {
            globalPath.header.stamp = timeLaserInfoStamp;
            globalPath.header.frame_id = odometryFrame;
            pubPath.publish(globalPath);
        }
    }
};


int main(int argc, char** argv) {
    ROS_INFO("\033[1;32m----> Map Optimization Started.\033[0m");
    ros::init(argc, argv, "liloc");

    mapOptimization MO;
    
    std::thread visualizeMapThread(&mapOptimization::visualizeGlobalMapThread, &MO);
    // std::thread visualizeTimeThread(&mapOptimization::displayTime, &MO);

    ros::spin();

    visualizeMapThread.join();
    // visualizeTimeThread.join();

    return 0;
}

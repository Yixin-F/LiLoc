#pragma once

#ifndef _ANCHOR_OPTIMIZE_HPP_
#define _ANCHOR_OPTIMIZE_HPP_

#include "../utility.h"
#include "../dataManager/dataLoader.hpp"

#include "../tictoc.h"

#include "BetweenFactorWithAnchoring.h"

#include <pclomp/ndt_omp.h>
#include <pclomp/gicp_omp.h>

namespace optimization {

class AnchorOptimization : public ParamServer {
public:
    std::mutex mtx;

    std::shared_ptr<dataManager::Session> priorSession_ = nullptr;
    pcl::Registration<PointType, PointType>::Ptr registration_ = nullptr;


    gtsam::NonlinearFactorGraph gtSAMgraph_;
    gtsam::ISAM2 *isam_;
    gtsam::Values initialEstimate_;
    gtsam::Values isamCurrentEstimate_;

    gtsam::Pose3 poseOrigin_;

    gtsam::noiseModel::Diagonal::shared_ptr priorNoise_;
    gtsam::noiseModel::Diagonal::shared_ptr odomNoise_;
    gtsam::noiseModel::Diagonal::shared_ptr matchNoise_;
    gtsam::noiseModel::Diagonal::shared_ptr largeNoise_;
    gtsam::noiseModel::Base::shared_ptr robustNoise_;

    std::vector<int> priorNodePtIds_;
    std::vector<int> increNodePtIds_;
    std::vector<int> currOdomNodeIds_;

    int key_;
    PointTypePose curPose_;
    CloudPtr curCloud_;

    const int session_id = 0;

    const int marg_size = 10;

    bool first_add = true;

    pcl::VoxelGrid<PointType> downSizeFilterSurf;

public:
    ~AnchorOptimization() { }
    AnchorOptimization(const std::shared_ptr<dataManager::Session>& session, const pcl::Registration<PointType, PointType>::Ptr &reg) 
    : priorSession_(session), registration_(reg)
    { 
        for (size_t i = 0; i < priorSession_->KeyPoses6D_->size(); i++) {
            priorNodePtIds_.push_back(i);
        }

        poseOrigin_ = gtsam::Pose3(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.0), gtsam::Point3(0.0, 0.0, 0.0));

        initOptimizer();

        initNoiseConstants();

        addPriorSessionToGraph();

        optimizeGraph(1);

        allocateMemory();

        ROS_INFO_STREAM("\033[1;32m Anchor Optimization is initialized successfully \033[0m");
    }

    void allocateMemory() {
        curCloud_.reset(new Cloud());
    }

    void initOptimizer(){
        gtsam::ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.1;
        parameters.relinearizeSkip = 1;
        isam_ = new gtsam::ISAM2(parameters);
    }

    void initNoiseConstants() {
        // Variances Vector6 order means : rad*rad, rad*rad, rad*rad, meter*meter, meter*meter, meter*meter
        {
            gtsam::Vector Vector6(6);
            Vector6 << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12;
            priorNoise_ = gtsam::noiseModel::Diagonal::Variances(Vector6);
        }

        {
            gtsam::Vector Vector6(6);
            Vector6 << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6;
            odomNoise_ = gtsam::noiseModel::Diagonal::Variances(Vector6);
        }

        {
            gtsam::Vector Vector6(6);
            Vector6 << 1e-5, 1e-5, 1e-5, 1e-3, 1e-3, 1e-3;
            matchNoise_ = gtsam::noiseModel::Diagonal::Variances(Vector6);
        }

        {
            gtsam::Vector Vector6(6);
            Vector6 << M_PI * M_PI, M_PI * M_PI, M_PI * M_PI, 1e8, 1e8, 1e8;
            largeNoise_ = gtsam::noiseModel::Diagonal::Variances(Vector6);
        }

        float robustNoiseScore = 0.5; // constant is ok...
        gtsam::Vector robustNoiseVector6(6); 
        robustNoiseVector6 << robustNoiseScore, robustNoiseScore, robustNoiseScore, robustNoiseScore, robustNoiseScore, robustNoiseScore;
        robustNoise_ = gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Cauchy::Create(1), 
            gtsam::noiseModel::Diagonal::Variances(robustNoiseVector6)
        );
    }

    void addPriorSessionToGraph() {
        int this_session_anchor_node_idx = genAnchorNodeIdx(priorSession_->index_);

        if (priorSession_->is_base_session_) {
            gtSAMgraph_.add(gtsam::PriorFactor<gtsam::Pose3>(this_session_anchor_node_idx, poseOrigin_, priorNoise_));
        }
        else {
            gtSAMgraph_.add(gtsam::PriorFactor<gtsam::Pose3>(this_session_anchor_node_idx, poseOrigin_, largeNoise_));
        }

        initialEstimate_.insert(this_session_anchor_node_idx, poseOrigin_);

        // add nodes 
        for (auto& _node: priorSession_->nodes_) {
            int node_idx = _node.second.idx;
            auto& curr_pose = _node.second.initial;

            int curr_node_global_idx = genGlobalNodeIdx(priorSession_->index_, node_idx);

            if (node_idx == 0) {
                // prior node 
                gtSAMgraph_.add(gtsam::PriorFactor<gtsam::Pose3>(curr_node_global_idx, curr_pose, priorNoise_));
                initialEstimate_.insert(curr_node_global_idx, curr_pose);
            }
            else {
                // odom nodes 
                gtSAMgraph_.add(gtsam::PriorFactor<gtsam::Pose3>(curr_node_global_idx, curr_pose, priorNoise_));
                initialEstimate_.insert(curr_node_global_idx, curr_pose);
            }
        }

        // add edges 
        for (auto& _edge: priorSession_->edges_) {
            int from_node_idx = _edge.second.from_idx;
            int to_node_idx = _edge.second.to_idx;

            int from_node_global_idx = genGlobalNodeIdx(priorSession_->index_, from_node_idx);
            int to_node_global_idx = genGlobalNodeIdx(priorSession_->index_, to_node_idx);

            gtsam::Pose3 relative_pose = _edge.second.relative;

            if (std::abs(to_node_idx - from_node_idx) == 1) {
                gtSAMgraph_.add(gtsam::BetweenFactor<gtsam::Pose3>(from_node_global_idx, to_node_global_idx, relative_pose, priorNoise_));
            }
        }
    }

    void optimizeGraph(int _inter) {
        isam_->update(gtSAMgraph_, initialEstimate_);

        while (--_inter) {
            isam_->update();
        }

        isamCurrentEstimate_ = isam_->calculateEstimate();

        gtSAMgraph_.resize(0);
        initialEstimate_.clear();

        updateSessionPoses();

        ROS_INFO_STREAM("Optimize ... " << " Have prior nodes: " << priorNodePtIds_.size() << ", current odometry nodes: " << currOdomNodeIds_.size() << " and incremental nodes: " << increNodePtIds_.size());
    }

    void getCurrentPose(const int &key, const PointTypePose & cur_pose, const CloudPtr & cur_cloud) {
        curPose_ = cur_pose;

        key_ = key;

        curCloud_ = cur_cloud;
    }

    void margilization() {
        if ((key_ != 0 && key_ % 10 == 0 || priorSession_->margFlag)) {
            int currentId = genGlobalNodeIdx(session_id, key_);

            gtsam::noiseModel::Gaussian::shared_ptr updatedPoseNoise = gtsam::noiseModel::Gaussian::Covariance(isam_->marginalCovariance(currentId));
            gtsam::noiseModel::Gaussian::shared_ptr updatedVelNoise  = gtsam::noiseModel::Gaussian::Covariance(isam_->marginalCovariance(currentId));
            gtsam::noiseModel::Gaussian::shared_ptr updatedBiasNoise = gtsam::noiseModel::Gaussian::Covariance(isam_->marginalCovariance(currentId));
        }
    }

    void addOdomFactors() {

        TicToc time("1");

        addLidarFactor();

        double t1 = time.toc("lidar factor");

        // std::cout << " lidar factor :" << t1 << std::endl;

        addImuFactor();


        addScanMatchingFactor();

        double t2 = time.toc("scan factor");

        // std::cout << " scan factor :" << t2 - t1 << std::endl;


        optimizeGraph(1);

        double t3 = time.toc("opt");

        // std::cout << " opt :" << t3 - t2 << std::endl;
    

        margilization();

        double t4 = time.toc("marg");

        // std::cout << " marg :" << t4 - t3 << std::endl;
    }

    void addLidarFactor() {
        if (key_ == 0) {
            int this_session_anchor_node_idx = genAnchorNodeIdx(session_id);
            gtSAMgraph_.add(gtsam::PriorFactor<gtsam::Pose3>(this_session_anchor_node_idx, poseOrigin_, largeNoise_));
            initialEstimate_.insert(this_session_anchor_node_idx, poseOrigin_);

            int this_node_id = genGlobalNodeIdx(session_id, key_);
            gtSAMgraph_.add(gtsam::PriorFactor<gtsam::Pose3>(this_node_id, pclPointTogtsamPose3(curPose_), priorNoise_));
            initialEstimate_.insert(this_node_id, pclPointTogtsamPose3(curPose_));

            currOdomNodeIds_.push_back(key_);
        }
        else {
            int this_node_id = genGlobalNodeIdx(session_id, key_);

            gtsam::Pose3 poseFrom = isamCurrentEstimate_.at<gtsam::Pose3>(this_node_id - 1);
            gtsam::Pose3 poseTo = pclPointTogtsamPose3(curPose_);

            gtsam::Pose3 poseRel = poseFrom.between(poseTo);
            initialEstimate_.insert(this_node_id, poseTo);

            gtSAMgraph_.add(gtsam::BetweenFactor<gtsam::Pose3>(this_node_id - 1, this_node_id, poseRel, odomNoise_));

            currOdomNodeIds_.push_back(key_);
        }
    }

    void addImuFactor() {

    }

    void addNewPrior() {
        static PointTypePose lastPose;
        static int end = priorSession_->KeyPoses6D_->size();
        static int count = 0;
        if (first_add) {
            curPose_.intensity = priorSession_->KeyPoses6D_->size();
            priorSession_->KeyPoses6D_->push_back(curPose_);

            CloudPtr copy(new Cloud());
            pcl::copyPointCloud(*curCloud_, *copy);
            priorSession_->keyCloudVec_.push_back(copy);

            int this_node_id = genGlobalNodeIdx(priorSession_->index_, (int)curPose_.intensity);

            gtsam::Pose3 poseFrom = isamCurrentEstimate_.at<gtsam::Pose3>(this_node_id - 1);
            gtsam::Pose3 poseTo = pclPointTogtsamPose3(curPose_);
            initialEstimate_.insert(this_node_id, poseTo);

            gtsam::Pose3 poseRel = poseFrom.between(poseTo);

            gtSAMgraph_.add(gtsam::BetweenFactor<gtsam::Pose3>(this_node_id - 1, this_node_id, poseRel, odomNoise_));

            increNodePtIds_.push_back(curPose_.intensity);
            count ++;
            first_add = false;
            lastPose = curPose_;
        }
        else {
            Eigen::Affine3f transStart = pclPointToAffine3f(lastPose);
            Eigen::Affine3f transFinal = pclPointToAffine3f(curPose_);

            Eigen::Affine3f transBetween = transStart.inverse() * transFinal;

            float x, y, z, roll, pitch, yaw;
            pcl::getTranslationAndEulerAngles(transBetween, x, y, z, roll, pitch, yaw);

            if (abs(roll)  < surroundingkeyframeAddingAngleThreshold &&
                abs(pitch) < surroundingkeyframeAddingAngleThreshold && 
                abs(yaw)   < surroundingkeyframeAddingAngleThreshold &&
                sqrt(x * x + y * y + z * z) < surroundingkeyframeAddingDistThreshold) 
            {
                return ;
            }

            curPose_.intensity = priorSession_->KeyPoses6D_->size();
            priorSession_->KeyPoses6D_->push_back(curPose_);

            CloudPtr copy(new Cloud());
            pcl::copyPointCloud(*curCloud_, *copy);
            priorSession_->keyCloudVec_.push_back(copy);

            int this_node_id = genGlobalNodeIdx(priorSession_->index_, (int)curPose_.intensity);
            gtsam::Pose3 poseFrom = isamCurrentEstimate_.at<gtsam::Pose3>(this_node_id - 1);
            gtsam::Pose3 poseTo = pclPointTogtsamPose3(curPose_);
            initialEstimate_.insert(this_node_id, poseTo);

            gtsam::Pose3 poseRel = poseFrom.between(poseTo);

            // PointTypePose pp = gtsamPose3ToPclPoint(poseRel);
            // std::cout << pp.x << " " << pp.y << std::endl;

            gtSAMgraph_.add(gtsam::BetweenFactor<gtsam::Pose3>(this_node_id - 1, this_node_id, poseRel, odomNoise_));


            increNodePtIds_.push_back(curPose_.intensity);
            count ++;
            first_add = false;
            lastPose = curPose_;

            if (count % priorSession_->submap_size == 0) {
                CloudPtr submap(new Cloud());
                TrajectoryPtr vertexCloud(new Trajectory());
                float xx = 0.0, xy = 0.0, xz = 0.0;

                for (int i = end; i < priorSession_->keyCloudVec_.size(); i++) {
                    *submap += *transformPointCloud(priorSession_->keyCloudVec_[i], &priorSession_->KeyPoses6D_->points[i]);
                    xx += priorSession_->KeyPoses6D_->points[i].x;
                    xy += priorSession_->KeyPoses6D_->points[i].y;
                    xz += priorSession_->KeyPoses6D_->points[i].z;

                    vertexCloud->push_back(priorSession_->KeyPoses6D_->points[i]);
                }

                PointTypePose centeriod;
                centeriod.x = xx / (float)priorSession_->submap_size;
                centeriod.y = xy / (float)priorSession_->submap_size;
                centeriod.z = xz / (float)priorSession_->submap_size;
                priorSession_->SubMapCenteriod_->push_back(centeriod);

                priorSession_->subMapVertexCloudVec_.push_back(vertexCloud);

                CloudPtr submap_copy(new Cloud());      

                downSizeFilterSurf.setLeafSize(0.2, 0.2, 0.2);
                downSizeFilterSurf.setInputCloud(submap);
                downSizeFilterSurf.filter(*submap);

                pcl::copyPointCloud(*submap, *submap_copy);
                priorSession_->subMapCloudVec_.push_back(submap_copy);

                // *priorSession_->globalMap_ += *submap_copy;

                end = priorSession_->KeyPoses6D_->size();
                count = 0;
                submap->clear();
            }

        }
    }

    void addScanMatchingFactor() {

        int submap_id;
        priorSession_->searchNearestSubMapAndVertex(curPose_, submap_id);

        if (priorSession_->usingVertexes_->size() <= 1) {
            addNewPrior();
            return ;
        }

        first_add = true;

        registration_->setInputTarget(priorSession_->usingSubMap_);
        registration_->setInputSource(curCloud_);

        Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();
        Eigen::Matrix3f rotation = eulerToRotation(curPose_.roll, curPose_.pitch, curPose_.yaw);
        Eigen::Vector3f translation(curPose_.x, curPose_.y, curPose_.z);
        init_guess.block(0, 0, 3, 3) = rotation;
        init_guess.block(0, 3, 3, 1) = translation;

        CloudPtr aligned(new Cloud());
        registration_->align(*aligned, init_guess);

        Eigen::Matrix4f transform;
        transform = registration_->getFinalTransformation();

        Eigen::Vector3f euler = RotMtoEuler(Eigen::Matrix3f(transform.block(0, 0, 3, 3)));
        Eigen::Vector3f xyz = transform.block(0, 3, 3, 1);

        int this_node_id = genGlobalNodeIdx(session_id, key_);

        gtsam::Pose3 poseTo(gtsam::Rot3::RzRyRx(euler(0), euler(1), euler(2)), gtsam::Point3(xyz(0), xyz(1), xyz(2)));

        if (key_ != 0) {
            gtsam::Pose3 poseFrom = isamCurrentEstimate_.at<gtsam::Pose3>(this_node_id - 1);
            gtsam::Pose3 poseRel = poseFrom.between(poseTo);

            gtSAMgraph_.add(BetweenFactor<Pose3>(this_node_id - 1, this_node_id, poseFrom.between(poseTo), matchNoise_));
        }

        CloudPtr curCloud_trans(new Cloud());
        PointTypePose pose_match = gtsamPose3ToPclPoint(poseTo);
        *curCloud_trans += *transformPointCloud(curCloud_, &pose_match);

        for (auto & id : priorSession_->localGoup_) {
            PointTypePose pose_i = priorSession_->subMapVertexCloudVec_[submap_id]->points[id];
            int prior_id = priorSession_->subMapVertexCloudVec_[submap_id]->points[id].intensity;

            auto it = std::find(increNodePtIds_.begin(), increNodePtIds_.end(), prior_id);
            if (it != increNodePtIds_.end()) {
                continue ;
            }

            CloudPtr cloud_i(new Cloud());
            *cloud_i += *priorSession_->keyCloudVec_[prior_id];

            CloudPtr cloud_i_trans(new Cloud());
            *cloud_i_trans += *transformPointCloud(cloud_i, &pose_i);

            float score = getICPFitnessScore(cloud_i_trans, curCloud_trans);

            if (score > 2) {
                score = 2;
            }

            gtsam::Vector Vector6(6);
            Vector6 << 1e-6 * score, 1e-6 * score, 1e-6 * score, 1e-5 * score, 1e-5 * score, 1e-5 * score;

            int from_id_i = genGlobalNodeIdx(priorSession_->index_, prior_id);
            gtsam::Pose3 poseFrom_i = isamCurrentEstimate_.at<gtsam::Pose3>(from_id_i);

            int to_id = genGlobalNodeIdx(session_id, key_);
            gtsam::Pose3 poseRel_i = poseFrom_i.between(poseTo);

            gtSAMgraph_.add(BetweenFactor<Pose3>(from_id_i, to_id, poseRel_i, matchNoise_));
        }
    }

    void updateSessionPoses() {

    }

    float getICPFitnessScore(const CloudPtr& cloud1_, const CloudPtr& cloud2_) {
        pcl::registration::CorrespondenceEstimation<PointType, PointType> est;
        est.setInputSource(cloud1_);
        est.setInputTarget(cloud2_);

        pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);
        est.determineCorrespondences(*correspondences);

        float fitness_score = 0.0;
        for (const auto& corr : *correspondences) {
            fitness_score += corr.distance;
        }

        fitness_score /= correspondences->size();

        return fitness_score;
    }

};

}

#endif
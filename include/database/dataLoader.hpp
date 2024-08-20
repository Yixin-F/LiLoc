#ifndef _DATA_LOADER_HPP_
#define _DATA_LOADER_HPP_

#include "../utility.h"

class DataLoader : public ParamServer {
public:
    ros::Publisher pub_map;
    ros::Publisher pub_pose;

    pcl::PointCloud<PointType>::Ptr prior_map;
    pcl::PointCloud<PointTypePose>::Ptr prior_pose;
    std::vector<pcl::PointCloud<PointType>::Ptr> prior_surf;
    std::vector<pcl::PointCloud<PointType>::Ptr> prior_corn;

    std::string map_name;
    std::string pose_name;
    std::string surf_dir;
    std::string corn_dir;

    SessionNodes nodes_;
    SessionEdges edges_;

    ~DataLoader() { }
    DataLoader(std::string path_) {
        pub_map = nh.advertise<sensor_msgs::PointCloud2>("/prior_map", 5);
        pub_pose = nh.advertise<sensor_msgs::PointCloud2>("/prior_pose", 5);
        
        map_name = path_ + "prior/prior_map.pcd";
        pose_name = path_ + "prior/prior_posegraph.g2o";
        surf_dir = path_ + "prior/prior_surf/";
        corn_dir = path_ + "prior/prior_corn/";

        allocateMemory();

        // if (!fs::exists(map_name) || !fs::exists(pose_name) || !fs::exists(surf_dir) || !fs::exists(corn_dir)) {
        //     ROS_ERROR_STREAM("\033[1;31m Prior directories do not exist ! \033[0m");
        //     ros::shutdown();
        // }

        // loadMap();
        // loadPose();
        // loadKeyFrame();
    }
    
    void allocateMemory() {
        prior_map.reset(new pcl::PointCloud<PointType>());
        prior_pose.reset(new pcl::PointCloud<PointTypePose>());
    }

    void loadMap() {
        if (pcl::io::loadPCDFile<PointType>(map_name, *prior_map) < 0) {
            ROS_WARN_STREAM("error in prior map loading !");
            return ;
        }

        pcl::VoxelGrid<PointType> down_sample;
        down_sample.setLeafSize(globalMapLeafSize, globalMapLeafSize, globalMapLeafSize);
        down_sample.setInputCloud(prior_map);
        down_sample.filter(*prior_map);

        ROS_INFO_STREAM("\033[1;32m Prior map loaded: " << map_name << "\033[0m");
    }

    void loadPose() {
        std::ifstream posefile_handle (pose_name);
        std::string strOneLine;
        
        while (getline(posefile_handle, strOneLine)) {
            G2oLineInfo line_info = splitG2oFileLine(strOneLine);

            if( isTwoStringSame(line_info.type, G2oLineInfo::kVertexTypeName) ) {
                Node this_node { line_info.curr_idx, gtsam::Pose3( 
                    gtsam::Rot3(gtsam::Quaternion(line_info.quat[3], line_info.quat[0], line_info.quat[1], line_info.quat[2])), // xyzw to wxyz
                    gtsam::Point3(line_info.trans[0], line_info.trans[1], line_info.trans[2])) }; 
                nodes_.insert(std::pair<int, Node>(line_info.curr_idx, this_node)); 
            }

            if( isTwoStringSame(line_info.type, G2oLineInfo::kEdgeTypeName) ) {
                Edge this_edge { line_info.prev_idx, line_info.curr_idx, gtsam::Pose3( 
                    gtsam::Rot3(gtsam::Quaternion(line_info.quat[3], line_info.quat[0], line_info.quat[1], line_info.quat[2])), // xyzw to wxyz
                    gtsam::Point3(line_info.trans[0], line_info.trans[1], line_info.trans[2])) }; 
                edges_.insert(std::pair<int, Edge>(line_info.prev_idx, this_edge)); 
            }
        }

        for(auto & _node_info: nodes_){
            PointTypePose thisPose6D;

            int node_idx = _node_info.first;
            Node node = _node_info.second; 
            gtsam::Pose3 pose = node.initial;

            thisPose6D.x = pose.translation().x();
            thisPose6D.y = pose.translation().y();
            thisPose6D.z = pose.translation().z();
            thisPose6D.intensity = node_idx; // TODO: remark
            thisPose6D.roll  = pose.rotation().roll();
            thisPose6D.pitch = pose.rotation().pitch();
            thisPose6D.yaw   = pose.rotation().yaw();
            thisPose6D.time = 0.0; // no-use

            prior_pose->push_back(thisPose6D);   
        }

        ROS_INFO_STREAM("\033[1;32m Prior pose loaded: " << pose_name << " with num: " << prior_pose->size() << "\033[0m");
    }

    void loadKeyFrame() {
        std::vector<std::string> surf_names;
        for(auto& surf : fs::directory_iterator(surf_dir)) {
            std::string surf_filepath = surf.path();
            surf_names.emplace_back(surf_filepath);
        }

        std::sort(surf_names.begin(), surf_names.end(), fileNameSort);

        int surf_count = 0;
        for (auto const& surf_name: surf_names){
            pcl::PointCloud<PointType>::Ptr this_surf(new pcl::PointCloud<PointType>());   
            pcl::io::loadPCDFile<PointType> (surf_name, *this_surf);
            ROS_INFO_STREAM("Prior " << surf_count << " -th surf keyframe loaded. ");

            prior_surf.push_back(this_surf);
            surf_count ++;
        }

        std::vector<std::string> corn_names;
        for(auto& corn : fs::directory_iterator(corn_dir)) {
            std::string corn_filepath = corn.path();
            corn_names.emplace_back(corn_filepath);
        }

        std::sort(corn_names.begin(), corn_names.end(), fileNameSort);

        int corn_count = 0;
        for (auto const& corn_name: corn_names){
            pcl::PointCloud<PointType>::Ptr this_corn(new pcl::PointCloud<PointType>());   
            pcl::io::loadPCDFile<PointType> (corn_name, *this_corn);
            ROS_INFO_STREAM("Prior " << corn_count << " -th corn keyframe loaded. ");

            prior_corn.push_back(this_corn);
            corn_count ++;
        }

        if (prior_surf.size() != prior_pose->size() || prior_corn.size() != prior_pose->size()) {
            ROS_ERROR_STREAM("\033[1;31m Prior surf keyframe num ( " << prior_surf.size() << " ) and corn keyframe num (" 
                             << prior_corn.size() << " ) is not equal to pose num ( " << prior_pose->size() << " ) !" << "\033[0m");
            ros::shutdown();
        }
    }


};


#endif
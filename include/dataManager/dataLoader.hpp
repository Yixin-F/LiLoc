#pragma once

#ifndef _DATA_LOADER_
#define _DATA_LOADER_

#include "../utility.h"

#include <experimental/filesystem> // file gcc>=8
#include <experimental/optional>

namespace fs = std::experimental::filesystem;

namespace dataManager {

// edge
struct Edge {
    int from_idx;
    int to_idx;
    gtsam::Pose3 relative;
};

// node
struct Node {
    int idx;
    gtsam::Pose3 initial;
};

// pose
struct Pose
{
    Eigen::Vector3d t;
    Eigen::Matrix3d R;
};

// g2o format
struct G2oLineInfo {
    std::string type;

    int prev_idx = -1; // for vertex, this member is null
    int curr_idx;

    std::vector<double> trans;
    std::vector<double> quat;

    inline static const std::string kVertexTypeName = "VERTEX_SE3:QUAT";
    inline static const std::string kEdgeTypeName = "EDGE_SE3:QUAT";
}; 

using SessionNodes = std::multimap<int, Node>; // from_idx, Node
using SessionEdges = std::multimap<int, Edge>; // from_idx, Edge

bool isTwoStringSame(std::string _str1, std::string _str2) {
	return !(_str1.compare(_str2));
}

// read g2o
// example：VERTEX_SE3:QUAT 99 -61.332581 -9.253125 0.131973 -0.004256 -0.005810 -0.625732 0.780005
G2oLineInfo splitG2oFileLine(std::string _str_line) {

    std::stringstream ss(_str_line);

	std::vector<std::string> parsed_elms ;
    std::string elm;
	char delimiter = ' ';
    while (getline(ss, elm, delimiter)) {
        parsed_elms.push_back(elm); // convert string to "double"
    }

	G2oLineInfo parsed;
    // determine whether edge or node
	if (isTwoStringSame(parsed_elms.at(0), G2oLineInfo::kVertexTypeName))
	{
		parsed.type = parsed_elms.at(0);
		parsed.curr_idx = std::stoi(parsed_elms.at(1));
		parsed.trans.push_back(std::stod(parsed_elms.at(2)));
		parsed.trans.push_back(std::stod(parsed_elms.at(3)));
		parsed.trans.push_back(std::stod(parsed_elms.at(4)));
		parsed.quat.push_back(std::stod(parsed_elms.at(5)));
		parsed.quat.push_back(std::stod(parsed_elms.at(6)));
		parsed.quat.push_back(std::stod(parsed_elms.at(7)));
		parsed.quat.push_back(std::stod(parsed_elms.at(8)));
	}
	if (isTwoStringSame(parsed_elms.at(0), G2oLineInfo::kEdgeTypeName))
	{
		parsed.type = parsed_elms.at(0);
		parsed.prev_idx = std::stoi(parsed_elms.at(1));
		parsed.curr_idx = std::stoi(parsed_elms.at(2));
		parsed.trans.push_back(std::stod(parsed_elms.at(3)));
		parsed.trans.push_back(std::stod(parsed_elms.at(4)));
		parsed.trans.push_back(std::stod(parsed_elms.at(5)));
		parsed.quat.push_back(std::stod(parsed_elms.at(6)));
		parsed.quat.push_back(std::stod(parsed_elms.at(7)));
		parsed.quat.push_back(std::stod(parsed_elms.at(8)));
		parsed.quat.push_back(std::stod(parsed_elms.at(9)));
	}

	return parsed;
}

void showProgressBar(int progress, int total) {
    int barWidth = 70; 
    float progressRatio = static_cast<float>(progress) / total;

    std::cout << "[";
    int pos = barWidth * progressRatio;
    for (int i = 0; i < barWidth; ++i) {
        if (i < pos) std::cout << "=";
        else if (i == pos) std::cout << ">";
        else std::cout << " ";
    }
    std::cout << "] " << int(progressRatio * 100.0) << " %\r";
    std::cout.flush();
}

// filesort by name
bool fileNameSort(std::string name1_, std::string name2_){
    std::string::size_type iPos1 = name1_.find_last_of('/') + 1;
	std::string filename1 = name1_.substr(iPos1, name1_.length() - iPos1);
	std::string name1 = filename1.substr(0, filename1.rfind("."));

    std::string::size_type iPos2 = name2_.find_last_of('/') + 1;
    std::string filename2 = name2_.substr(iPos2, name2_.length() - iPos2);
	std::string name2 = filename2.substr(0, filename2.rfind(".")); 

    return std::stoi(name1) < std::stoi(name2);
}

class Session : public ParamServer {
public:
    int index_;

    std::string name_;
    std::string session_dir_path_;

    bool is_base_session_;

    SessionNodes nodes_;
    SessionEdges edges_;

    int anchor_node_idx_;

    TrajectoryPtr KeyPoses6D_;  // parsing
    TrajectoryPtr originPoses6D_;

    CloudPtr globalMap_;

    std::vector<CloudPtr> keyCloudVec_;

    int prior_size;

    std::vector<TrajectoryPtr> subMapVertexCloudVec_;
    std::vector<CloudPtr> subMapCloudVec_;
    TrajectoryPtr SubMapCenteriod_;

    CloudPtr usingSubMap_;
    TrajectoryPtr usingVertexes_;

    pcl::KdTreeFLANN<PointTypePose>::Ptr kdtreeSearchVertex_;
    pcl::KdTreeFLANN<PointTypePose>::Ptr kdtreeSearchSubMap_;

    int submap_id = -1;
    int submap_size = 10;
    int search_gap = submap_size;

    bool margFlag = false;

    std::vector<int> localGoup_;

    pcl::VoxelGrid<PointType> downSizeFilterSurf;

public:
    ~Session() { }
    Session() { }

    Session(int _idx, std::string _name, std::string _session_dir_path, bool _is_base_session)
           : index_(_idx), name_(_name), session_dir_path_(_session_dir_path), is_base_session_(_is_base_session){

        allocateMemory();

        loadSessionGraph();

        loadGlobalMap();

        loadKeyCloud();

        prior_size = KeyPoses6D_->size();

        generateSubMaps();

        ROS_INFO_STREAM("\033[1;32m Session " << index_ << " (" << name_ << ") is loaded successfully \033[0m");
    }

    void allocateMemory() {
        KeyPoses6D_.reset(new Trajectory());
        originPoses6D_.reset(new Trajectory());
        globalMap_.reset(new Cloud());

        usingSubMap_.reset(new Cloud());
        usingVertexes_.reset(new Trajectory());

        SubMapCenteriod_.reset(new Trajectory());

        kdtreeSearchSubMap_.reset(new pcl::KdTreeFLANN<PointTypePose>());
        kdtreeSearchVertex_.reset(new pcl::KdTreeFLANN<PointTypePose>());
    }

    void loadSessionGraph() {
        std::string posefile_path = session_dir_path_ + "/singlesession_posegraph.g2o";

        std::ifstream posefile_handle (posefile_path);
        std::string strOneLine;

        while (getline(posefile_handle, strOneLine)) {
            G2oLineInfo line_info = splitG2oFileLine(strOneLine);

            // save variables (nodes)
            if (isTwoStringSame(line_info.type, G2oLineInfo::kVertexTypeName)) {
                Node this_node { line_info.curr_idx, gtsam::Pose3( 
                    gtsam::Rot3(gtsam::Quaternion(line_info.quat[3], line_info.quat[0], line_info.quat[1], line_info.quat[2])), // xyzw to wxyz
                    gtsam::Point3(line_info.trans[0], line_info.trans[1], line_info.trans[2])) }; 
                nodes_.insert(std::pair<int, Node>(line_info.curr_idx, this_node)); 
            }
 
            // save edges 
            if(isTwoStringSame(line_info.type, G2oLineInfo::kEdgeTypeName)) {
                Edge this_edge { line_info.prev_idx, line_info.curr_idx, gtsam::Pose3( 
                    gtsam::Rot3(gtsam::Quaternion(line_info.quat[3], line_info.quat[0], line_info.quat[1], line_info.quat[2])), // xyzw to wxyz
                    gtsam::Point3(line_info.trans[0], line_info.trans[1], line_info.trans[2])) }; 
                edges_.insert(std::pair<int, Edge>(line_info.prev_idx, this_edge)); 
            }
        }

        initKeyPoses();

        ROS_INFO_STREAM("\033[1;32m Graph loaded: " << posefile_path << " - num nodes: " << nodes_.size() << "\033[0m");
    }

    void initKeyPoses() {
        for (auto & _node_info: nodes_) {
            PointTypePose thisPose6D;

            int node_idx = _node_info.first;
            Node node = _node_info.second; 
            gtsam::Pose3 pose = node.initial;

            thisPose6D.x = pose.translation().x();
            thisPose6D.y = pose.translation().y();
            thisPose6D.z = pose.translation().z();
            thisPose6D.intensity = node_idx; // TODO
            thisPose6D.roll  = pose.rotation().roll();
            thisPose6D.pitch = pose.rotation().pitch();
            thisPose6D.yaw   = pose.rotation().yaw();
            thisPose6D.time = 0.0; // TODO: no-use

            KeyPoses6D_->push_back(thisPose6D);   
        }
        // std::cout << "final prior intensity: " << KeyPoses6D_->points.back().intensity << std::endl;

        PointTypePose thisPose6D;
        thisPose6D.x = 0.0;
        thisPose6D.y = 0.0;
        thisPose6D.z = 0.0;
        thisPose6D.intensity = 0.0;
        thisPose6D.roll = 0.0;
        thisPose6D.pitch = 0.0;
        thisPose6D.yaw = 0.0;
        thisPose6D.time = 0.0;
        originPoses6D_->push_back(thisPose6D);
    }

    void loadGlobalMap() {
        std::string mapfile_path = session_dir_path_ + "/globalMap.pcd";  
        pcl::io::loadPCDFile<PointType>(mapfile_path, *globalMap_);
        ROS_INFO_STREAM("\033[1;32m Map loaded: " << mapfile_path << " - size: " << globalMap_->points.size() << "\033[0m");
    }

    void loadKeyCloud() {
        std::string pcd_dir = session_dir_path_ + "/PCDs/";

        std::vector<std::string> pcd_names;
        for(auto& pcd : fs::directory_iterator(pcd_dir)) {
            std::string pcd_filepath = pcd.path();
            pcd_names.emplace_back(pcd_filepath);
        }

        std::sort(pcd_names.begin(), pcd_names.end(), fileNameSort);

        int pcd_num = pcd_names.size();

        int pcd_count = 0; 
        for (auto const& pcd_name: pcd_names){
            pcl::PointCloud<PointType>::Ptr this_pcd(new pcl::PointCloud<PointType>());   
            pcl::io::loadPCDFile<PointType> (pcd_name, *this_pcd);

            showProgressBar(pcd_count, pcd_num);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));

            keyCloudVec_.push_back(this_pcd);

            pcd_count ++;
        }

        ROS_INFO_STREAM("\033[1;32m Key Cloud loaded: " << pcd_dir << " - num pcds: " << keyCloudVec_.size() << "\033[0m");
    }

    void generateSubMaps() {
        CloudPtr submap(new Cloud());
        float x = 0.0, y = 0.0, z = 0.0;
        int count = 0;

        TrajectoryPtr vertexCloud(new Trajectory());

        for (int i = 0; i < keyCloudVec_.size(); i++) {
            count ++;
            *submap += *transformPointCloud(keyCloudVec_[i], &KeyPoses6D_->points[i]);
            x += KeyPoses6D_->points[i].x;
            y += KeyPoses6D_->points[i].y;
            z += KeyPoses6D_->points[i].z;

            vertexCloud->push_back(KeyPoses6D_->points[i]);

            if (count % submap_size == 0 || i == keyCloudVec_.size() - 1) {
                PointTypePose centeriod;
                centeriod.x = x / (float)count;
                centeriod.y = y / (float)count;
                centeriod.z = z / (float)count;
                SubMapCenteriod_->push_back(centeriod);

                subMapVertexCloudVec_.push_back(vertexCloud);
                CloudPtr submap_copy(new Cloud());

                downSizeFilterSurf.setLeafSize(0.2, 0.2, 0.2);
                downSizeFilterSurf.setInputCloud(submap);
                downSizeFilterSurf.filter(*submap_copy);

                pcl::copyPointCloud(*submap, *submap_copy);
                subMapCloudVec_.push_back(submap_copy);

                // std::cout << submap_copy->size() << std::endl;

                count = 0;
                x = 0.0; y = 0.0; z = 0.0;
                submap->clear();
            }
        }

        ROS_INFO_STREAM("\033[1;32m Submap Generated - num: " << subMapCloudVec_.size() << " with pcd num: " << submap_size << "\033[0m");
    }

    void searchNearestSubMapAndVertex(const PointTypePose& pose, int& map_id) {
        std::vector<int> ids;
        std::vector<float> dis;
        kdtreeSearchSubMap_->setInputCloud(SubMapCenteriod_);
        kdtreeSearchSubMap_->nearestKSearchT(pose, 1, ids, dis);

        map_id = ids[0];
        if (submap_id != map_id) {
            submap_id = map_id;
            margFlag = true;
        }
        
        usingSubMap_ = subMapCloudVec_[map_id];

        std::cout << usingSubMap_->size() << std::endl;

        ids.clear(); dis.clear();
        kdtreeSearchVertex_->setInputCloud(subMapVertexCloudVec_[map_id]);
        kdtreeSearchVertex_->radiusSearch(pose, 5.0, ids, dis);

        usingVertexes_->clear();
        localGoup_.clear();
        int count = 0;
        for (auto id : ids) {
            int curNew = KeyPoses6D_->size();

            if (subMapVertexCloudVec_[map_id]->points[id].intensity > prior_size && 
                std::abs(curNew - subMapVertexCloudVec_[map_id]->points[id].intensity) <= 10) 
            {
                continue;
            }

            count ++;

            if (count > 3) {
                break;
            }
            usingVertexes_->push_back(subMapVertexCloudVec_[map_id]->points[id]);
            localGoup_.push_back(id);
        }
    }
};

}

#endif
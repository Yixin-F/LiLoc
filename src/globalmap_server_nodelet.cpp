#include "utility.h"
#include "block_localization/queryMap.h"

#include <map_server/image_loader.h>
#include <yaml-cpp/yaml.h>

#ifdef HAVE_NEW_YAMLCPP
template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
    i = node.as<T>();
}
#endif


namespace block_localization {

class GlobalmapServerNodelet : public ParamServer, public nodelet::Nodelet
{
public:
    GlobalmapServerNodelet() {
    }
    virtual ~GlobalmapServerNodelet() {
    }

    void onInit() override {
        nh = getNodeHandle();

        loadAllMap();

        loadMapCentroids();

        // load initial BM (using the pose from global localization)
        globalmap = *loadMapFromIdx(0);
        // service for changing BM
        mapQueryServer = nh.advertiseService("/mapQuery", &GlobalmapServerNodelet::mapQueryCB, this);
        // publish globalmap with "latched" publisher
        globalmap_pub = nh.advertise<sensor_msgs::PointCloud2>("/globalmap", 5, true);
        globalmap_pub_timer = nh.createWallTimer(ros::WallDuration(mapqry_interval), &GlobalmapServerNodelet::pubMapOnce, this, false, true);
    }

private:
    void pubMapOnce(const ros::WallTimerEvent& event) {
        sensor_msgs::PointCloud2 ros_cloud;
        if (!globalmap.empty()) {
            pcl::toROSMsg(globalmap, ros_cloud);
            globalmap_pub.publish(ros_cloud);
        }
    }


    bool mapQueryCB(block_localization::queryMap::Request &req,
                    block_localization::queryMap::Response &res) {
        PointT searchPoint;
        searchPoint.x = req.position.x;
        searchPoint.y = req.position.y;
        searchPoint.z = req.position.z;
        // NODELET_INFO("K-nearest neighbor search at (%f, %f, %f).", searchPoint.x, searchPoint.y, searchPoint.z);
        
        // find nearest block map
        pointIdxNKNSearch.clear();
        pointNKNSquareDistance.clear();
        int k_nearest = centroid_kdtree.nearestKSearch(searchPoint, 2, pointIdxNKNSearch, pointNKNSquareDistance);
        if (k_nearest == 2) {
            globalmap.clear();
            globalmap = (*loadMapFromIdx(pointIdxNKNSearch[0])) + (*loadMapFromIdx(pointIdxNKNSearch[1]));
        } else if (k_nearest == 1) {
            globalmap.clear();
            globalmap = *loadMapFromIdx(pointIdxNKNSearch[0]);
        } // else BM doesn't need to be changed

        res.success = true;
        return true;
    }


    void loadAllMap() {
        auto t1 = ros::WallTime::now();
        boost::format load_format("%03d.pcd");

        for (int map_id = 0; ; map_id++)
        {
            pcl::PointCloud<PointT>::Ptr tmp_cloud(new pcl::PointCloud<PointT>());
            std::string map_name = globalmap_dir + (load_format % map_id).str();

            if (pcl::io::loadPCDFile(map_name, *tmp_cloud) == -1) {
                map_id -= 1;
                map_name = globalmap_dir + (load_format % map_id).str();
                NODELET_WARN("The last map is: %s", map_name.c_str());
                break;
            }

            tmp_cloud->header.frame_id = "globalmap_link";

            // downsampling
            boost::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(new pcl::VoxelGrid<PointT>());
            voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
            voxelgrid->setInputCloud(tmp_cloud);
            pcl::PointCloud<PointT>::Ptr filtered_cloud(new pcl::PointCloud<PointT>());
            voxelgrid->filter(*filtered_cloud);
            tmp_cloud = filtered_cloud;
            
            globalmap_vec.push_back(tmp_cloud);
        }
        auto t2 = ros::WallTime::now();
        NODELET_INFO("Globalmap server has already loaded %ld maps. Time cost: %f [msec]", globalmap_vec.size(), (t2 - t1).toSec() * 1000.0);
    }


    inline pcl::PointCloud<PointT>::Ptr loadMapFromIdx(int map_idx) {
        // NODELET_INFO("Globalmap NO.%03d is already loaded!", map_idx);
        return globalmap_vec[map_idx];
    }


    void loadMapCentroids() {
        std::string centroid_filename = globalmap_dir + "CentroidCloud.pcd";
        
        centroid_cloud.reset(new pcl::PointCloud<PointT>());
        if (pcl::io::loadPCDFile(centroid_filename, *centroid_cloud) == -1) {
            NODELET_ERROR("Fail to load the centroid cloud! Please check your source!");
            ros::shutdown();
        }

        NODELET_INFO("Already loaded %ld centroids of block maps.", centroid_cloud->points.size());

        if (centroid_cloud->empty()) {
            NODELET_ERROR("Fail to build a KD-Tree! Please check your centroid pointcloud!");
            ros::shutdown();
        }

        // build KD-Tree
        centroid_kdtree.setInputCloud(centroid_cloud);
    }


    void pubGridmap() {
        YAML::Node doc = YAML::LoadFile(yaml_path_);
        try
        {
            doc["resolution"] >> res_;
            doc["origin"][0] >> origin_[0];
            doc["origin"][1] >> origin_[1];
            doc["origin"][2] >> origin_[2];
            doc["negate"] >> negate_;
            doc["occupied_thresh"] >> occ_th_;
            doc["free_thresh"] >> free_th_;
        }
        catch(YAML::InvalidScalar)
        {
            ROS_ERROR("The .yaml does not contain tags required or they are invalid.");
            ros::shutdown();
        }
        mode_ = TRINARY;

        std::cout << "	       map name: " << map_name_
                    << "\n	     resolution: " << res_
                    << "\n	         origin: " << origin_[0] << ", " << origin_[1] << ", " << origin_[2]
                    << "\n	         negate: " << negate_
                    << "\n	occupied thresh: " << occ_th_
                    << "\n	    free thresh: " << free_th_ << std::endl;
        map_server::loadMapFromFile(&map_resp_, pgm_path_.c_str(), res_, negate_, occ_th_, free_th_, origin_, mode_);
        map_resp_.map.header.frame_id = "gridmap_link";
        map_publisher_ = nh.advertise<nav_msgs::OccupancyGrid> ("/gridmap", 1, true);
        map_publisher_.publish(map_resp_.map);
    }


private:
    // map settings
    pcl::PointCloud<PointT> globalmap;
    std::vector<pcl::PointCloud<PointT>::Ptr> globalmap_vec;

    ros::ServiceServer mapQueryServer;
    ros::Publisher globalmap_pub;
    ros::WallTimer globalmap_pub_timer;

    // block map centroids
    pcl::PointCloud<PointT>::Ptr centroid_cloud;

    // KD-Tree for block maps
    pcl::KdTreeFLANN<PointT> centroid_kdtree;
    std::vector<int> pointIdxNKNSearch;
    std::vector<float> pointNKNSquareDistance;

    MapMode mode_;
    std::string map_name_;
    double res_;
    double origin_[3];
    int negate_;
    double occ_th_, free_th_;
    nav_msgs::GetMap::Response map_resp_;
    ros::Publisher map_publisher_;
};

}


PLUGINLIB_EXPORT_CLASS(block_localization::GlobalmapServerNodelet, nodelet::Nodelet)
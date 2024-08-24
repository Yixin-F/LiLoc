#ifndef _PREPROCESS_HPP_
#define _PREPROCESS_HPP_

#include "../utility.h"
#include <livox_ros_driver/CustomMsg.h>

class Preprocess : public ParamServer {
public:
    ros::Subscriber sub_points;
    ros::Subscriber sub_imu;
    ros::Publisher pub_full;
    ros::Publisher pub_surf;
    ros::Publisher pub_corn;
.
    std::mutex buff_mutex;

    std::deque<std::pair<double, pcl::PointCloud<PointType>::Ptr>> cloud_buff;
    std::deque<sensor_msgs::Imu> imu_buff;

    double lidar_timestamp = 0.0;
    double imu_timestamp = 0.0;

    const double rad2deg = 180 * M_1_PI;

    double vx, vy, vz;  // tmp

    bool has_time_ = false;

    SensorMeasurement sensor_measurement;

    ~Preprocess() { }

    Preprocess() {
        sub_imu = nh.subscribe(imuTopic, 10000, &Preprocess::imuHandler, this, ros::TransportHints().tcpNoDelay());

        switch(sensor) {
            case SensorType::VELODYNE:
                sub_points = nh.subscribe(pointCloudTopic, 1000, &Preprocess::veloHandler, this, ros::TransportHints().tcpNoDelay());
                break;
            
            case SensorType::OUSTER:
                sub_points = nh.subscribe(pointCloudTopic, 1000, &Preprocess::ousterHandler, this, ros::TransportHints().tcpNoDelay());
                break;

            default:
                sub_points = nh.subscribe(pointCloudTopic, 1000, &Preprocess::livoxHandler, this, ros::TransportHints().tcpNoDelay());
                break;
        }

        pub_full = nh.advertise<sensor_msgs::PointCloud2>("/pc2_fullN", 20);
        pub_surf = nh.advertise<sensor_msgs::PointCloud2>("/pc2_surfN", 20);
        pub_corn = nh.advertise<sensor_msgs::PointCloud2>("/pc2_cornN", 20);
    }

    void imuHandler(const sensor_msgs::Imu::ConstPtr &msg) {
        std::lock_guard<std::mutex> lock(buff_mutex);

        // sensor_msgs::Imu msg_aligned = imuConverter(*msg);
        // msg = std::move(&msg_aligned);

        static double last_imu_timestamp = -1.0;
        static sensor_msgs::Imu last_imu = *msg;

        // parameters for EMA filter
        static double a = 0.8;
        static double b = 1.0 - a;

        sensor_msgs::Imu imu_msg = *msg;
        imu_timestamp = imu_msg.header.stamp.toSec();
            
        if (imu_timestamp < last_imu_timestamp) {
            ROS_WARN_STREAM("imu loop back, clear buffer");
            imu_buff.clear();
        }

        // EMA filter for accelerometer
        imu_msg.linear_acceleration.x = msg->linear_acceleration.x * a + last_imu.linear_acceleration.x * b;
        imu_msg.linear_acceleration.y = msg->linear_acceleration.y * a + last_imu.linear_acceleration.y * b;
        imu_msg.linear_acceleration.z = msg->linear_acceleration.z * a + last_imu.linear_acceleration.z * b;

        last_imu = *msg;
        last_imu_timestamp = imu_timestamp;

        imu_buff.push_back(imu_msg);
    }

    void veloHandler(const sensor_msgs::PointCloud2::ConstPtr &msg) {
        std::lock_guard<std::mutex> lock(buff_mutex);

        static double last_lidar_timestamp = 0.0;
        lidar_timestamp = msg->header.stamp.toSec();

        if (lidar_timestamp < last_lidar_timestamp) {
            ROS_WARN_STREAM("lidar loop back, clear buffer");
            cloud_buff.clear();
        }
        last_lidar_timestamp = lidar_timestamp;

        pcl::PointCloud<PointType>::Ptr cloud_ptr(new pcl::PointCloud<PointType>());
        processVelodyne(msg, cloud_ptr);

        cloud_buff.push_back(std::make_pair(msg->header.stamp.toSec(), cloud_ptr));
    }

    void processVelodyne(const sensor_msgs::PointCloud2::ConstPtr &msg, pcl::PointCloud<PointType>::Ptr& cloud_out) {
        pcl::PointCloud<VelodynePointXYZIRT> cloud_origin;
        pcl::fromROSMsg(*msg, cloud_origin);

        // These variables only works when no point timestamps given
        int plsize = cloud_origin.size();
        double omega_l = 3.61;  // scan angular velocity
        std::vector<bool> is_first(N_SCAN, true);
        std::vector<double> yaw_fp(N_SCAN, 0.0);    // yaw of first scan point
        std::vector<float> yaw_last(N_SCAN, 0.0);   // yaw of last scan point
        std::vector<float> time_last(N_SCAN, 0.0);  // last offset time

        if (cloud_origin.back().time > 0) {
            has_time_ = true;
        } 
        else {
            has_time_ = false;
            double yaw_first = atan2(cloud_origin.points[0].y, cloud_origin.points[0].x) * 57.29578;
            double yaw_end = yaw_first;
            // FIXME: there is no 'ring' information sometimes !!, we have to calculate it by ourselves
            int layer_first = cloud_origin.points[0].ring;
            for (uint i = plsize - 1; i > 0; i--) {
                if (cloud_origin.points[i].ring == layer_first) {
                    yaw_end = atan2(cloud_origin.points[i].y, cloud_origin.points[i].x) * 57.29578;
                    break;
                }
            }
        }

        for (size_t i = 0; i < cloud_origin.size(); ++i) {
            if ((i % point_filter_num == 0) && !HasInf(cloud_origin.at(i)) && !HasNan(cloud_origin.at(i))) {
                PointType point;
                point.normal_x = 0;
                point.normal_y = 0;
                point.normal_z = 0;
                point.x = cloud_origin.at(i).x;
                point.y = cloud_origin.at(i).y;
                point.z = cloud_origin.at(i).z;
                point.intensity = cloud_origin.at(i).intensity;
                if (has_time_) {
                    point.curvature = cloud_origin.at(i).time * timeScale;  // ms
                } 
                else {
                    int layer = cloud_origin.points[i].ring;
                    double yaw_angle = atan2(point.y, point.x) * 57.2957;

                    if (is_first[layer]) {
                        yaw_fp[layer] = yaw_angle;
                        is_first[layer] = false;
                        point.curvature = 0.0;
                        yaw_last[layer] = yaw_angle;
                        time_last[layer] = point.curvature;
                        continue;
                    }

                    // compute offset time
                    if (yaw_angle <= yaw_fp[layer]) {
                        point.curvature = (yaw_fp[layer] - yaw_angle) / omega_l;
                    } 
                    else {
                        point.curvature = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l;
                    }

                    if (point.curvature < time_last[layer])
                        point.curvature += 360.0 / omega_l;

                    if (!std::isfinite(point.curvature)) {
                        continue;
                    }

                    yaw_last[layer] = yaw_angle;
                    time_last[layer] = point.curvature;
                }

                cloud_out->push_back(point);
            }
        }

        std::sort(cloud_out->points.begin(), cloud_out->points.end(), [](const PointType& x, const PointType& y) 
                        -> bool { return (x.curvature < y.curvature); });
    }

    void getVelodyneFeature(const std::pair<double, pcl::PointCloud<PointType>::Ptr> &cloud_info, 
                            pcl::PointCloud<PointType>::Ptr &surf, pcl::PointCloud<PointType>::Ptr &corn)
    {
        auto lidar_timestamp_ros = ros::Time().fromSec(cloud_info.first);

        int orders[N_SCAN];
        for (int i = 0; i < N_SCAN / 2; i++) {
            orders[i] = i * 2;
            orders[i + N_SCAN / 2] = i * 2 + 1;
        }

        pcl::PointCloud<PointType> pl_orig = *cloud_info.second;
        uint plsize = pl_orig.size();

        std::vector<pcl::PointCloud<PointType>> pl_buff(N_SCAN);
        std::vector<std::vector<orgtype>> typess(N_SCAN);
        pcl::PointCloud<PointType> pl_corn, pl_surf;

        int scanID;
        int last_stat = -1;
        int idx = 0;

        for (int i = 0; i < N_SCAN; i++) {
            pl_buff[i].resize(plsize);
            typess[i].resize(plsize);
        }

        for (uint i = 0; i < plsize; i++) {
            PointType &ap = pl_orig[i];
            double leng = sqrt(ap.x * ap.x + ap.y * ap.y);
            if (leng < blind) {
                continue;
            }

            double ang = atan(ap.z / leng) * rad2deg;
            scanID = int((ang + 15) / 2 + 0.5);

            if (scanID >= N_SCAN || scanID < 0) {
                continue;
            }

            if(orders[scanID] <= last_stat) {
                idx ++;
            }

            pl_buff[scanID][idx].x = ap.x;
            pl_buff[scanID][idx].y = ap.y;
            pl_buff[scanID][idx].z = ap.z;
            pl_buff[scanID][idx].intensity = ap.intensity;
            typess[scanID][idx].range = leng;
            last_stat = orders[scanID];
        }

        idx ++;

        for (int j = 0; j < N_SCAN; j++) {
            pcl::PointCloud<PointType> &pl = pl_buff[j];
            std::vector<orgtype> &types = typess[j];
            pl.erase(pl.begin() + idx, pl.end());
            types.erase(types.begin() + idx, types.end());
            plsize = idx - 1;
            for (uint i = 0; i < plsize; i++) {
                vx = pl[i].x - pl[i + 1].x;
                vy = pl[i].y - pl[i + 1].y;
                vz = pl[i].z - pl[i + 1].z;
                types[i].dista = vx * vx + vy * vy + vz * vz;
            }
            types[plsize].range = sqrt(pl[plsize].x * pl[plsize].x + pl[plsize].y * pl[plsize].y);
    
            give_feature(pl, types, pl_corn, pl_surf);
        }

        *surf += pl_surf;
        *corn += pl_corn;

        publishCloud(pub_full, &pl_orig, lidar_timestamp_ros, "map");
        publishCloud(pub_surf, &pl_surf, lidar_timestamp_ros, "map");
        publishCloud(pub_corn, &pl_corn, lidar_timestamp_ros, "map");
    }

    void ousterHandler(const sensor_msgs::PointCloud2::ConstPtr &msg) {
        std::lock_guard<std::mutex> lock(buff_mutex);

        static double last_lidar_timestamp = 0.0;
        lidar_timestamp = msg->header.stamp.toSec();

        if (lidar_timestamp < last_lidar_timestamp) {
            ROS_WARN_STREAM("lidar loop back, clear buffer");
            cloud_buff.clear();
        }
        last_lidar_timestamp = lidar_timestamp;

        pcl::PointCloud<PointType>::Ptr cloud_ptr(new pcl::PointCloud<PointType>());
        processOuster(msg, cloud_ptr);

        cloud_buff.push_back(std::make_pair(msg->header.stamp.toSec(), cloud_ptr));
    }

    void processOuster(const sensor_msgs::PointCloud2::ConstPtr &msg, pcl::PointCloud<PointType>::Ptr& cloud_out) {
        pcl::PointCloud<OusterPointXYZIRT> cloud_origin;
        pcl::fromROSMsg(*msg, cloud_origin);

        for (size_t i = 0; i < cloud_origin.size(); ++i) {
            if ((i % point_filter_num == 0) && !HasInf(cloud_origin.at(i)) && !HasNan(cloud_origin.at(i))) {
                PointType point;
                point.normal_x = 0;
                point.normal_y = 0;
                point.normal_z = 0;
                point.x = cloud_origin.at(i).x;
                point.y = cloud_origin.at(i).y;
                point.z = cloud_origin.at(i).z;
                point.intensity = cloud_origin.at(i).intensity;
                point.curvature = cloud_origin.at(i).t * 1e-6;  // ms
                cloud_out->push_back(point);
            }
        }

        std::sort(cloud_out->points.begin(), cloud_out->points.end(), [](const PointType& x, const PointType& y) 
                        -> bool { return (x.curvature < y.curvature); });
    }

    void getOusterFeature(const std::pair<double, pcl::PointCloud<PointType>::Ptr> &cloud_info, 
                          pcl::PointCloud<PointType>::Ptr &surf, pcl::PointCloud<PointType>::Ptr &corn)
    {
        auto lidar_timestamp_ros = ros::Time().fromSec(cloud_info.first);

        pcl::PointCloud<PointType> pl_orig = *cloud_info.second;
        uint plsize = pl_orig.size();

        std::vector<pcl::PointCloud<PointType>> pl_buff(N_SCAN);
        std::vector<std::vector<orgtype>> typess(N_SCAN);
        pcl::PointCloud<PointType> pl_corn, pl_surf;

        for (int i = 0; i < N_SCAN; i++) {
            pl_buff[i].reserve(plsize);
        }

        for (uint i = 0; i < plsize; i += N_SCAN) {
            for (int j = 0; j < N_SCAN; j++) {
                pl_buff[j].push_back(pl_orig[i+j]);
            }
        }

        for (int j = 0; j < N_SCAN; j++) {
            pcl::PointCloud<PointType> &pl = pl_buff[j];
            std::vector<orgtype> &types = typess[j];
            plsize = pl.size() - 1;
            types.resize(plsize + 1);
            for (uint i = 0; i < plsize; i++) {
                types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
                vx = pl[i].x - pl[i + 1].x;
                vy = pl[i].y - pl[i + 1].y;
                vz = pl[i].z - pl[i + 1].z;
                types[i].dista = vx * vx + vy * vy + vz * vz;
            }
            types[plsize].range = sqrt(pl[plsize].x * pl[plsize].x + pl[plsize].y * pl[plsize].y);
            give_feature(pl, types, pl_corn, pl_surf);
        }

        *surf += pl_surf;
        *corn += pl_corn;

        publishCloud(pub_full, &pl_orig, lidar_timestamp_ros, "map");
        publishCloud(pub_surf, &pl_surf, lidar_timestamp_ros, "map");
        publishCloud(pub_corn, &pl_corn, lidar_timestamp_ros, "map");
    }

    void livoxHandler(const livox_ros_driver::CustomMsg::ConstPtr &msg) {
        std::lock_guard<std::mutex> lock(buff_mutex);

        static double last_lidar_timestamp = 0.0;
        lidar_timestamp = msg->header.stamp.toSec();

        if (lidar_timestamp < last_lidar_timestamp) {
            ROS_WARN_STREAM("lidar loop back, clear buffer");
            cloud_buff.clear();
        }
        last_lidar_timestamp = lidar_timestamp;

        pcl::PointCloud<PointType>::Ptr cloud_ptr(new pcl::PointCloud<PointType>());
        processLivox(msg, cloud_ptr, lidar_timestamp);

        cloud_buff.push_back(std::make_pair(msg->header.stamp.toSec(), cloud_ptr));
    }

    void processLivox(const livox_ros_driver::CustomMsg::ConstPtr& msg, pcl::PointCloud<PointType>::Ptr& cloud_out, const double last_start_time) {
        double time_offset = (msg->header.stamp.toSec() - last_start_time) * 1000.0;  // ms

        for (size_t i = 1; i < msg->point_num; ++i) {
            if ((msg->points[i].line < N_SCAN) &&
                ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00) &&
                !HasInf(msg->points[i]) && !HasNan(msg->points[i]) &&
                !IsNear(msg->points[i], msg->points[i - 1]) &&
                (i % point_filter_num == 0)) {
                    PointType point;
                    point.normal_x = 0;
                    point.normal_y = 0;
                    point.normal_z = 0;
                    point.x = msg->points[i].x;
                    point.y = msg->points[i].y;
                    point.z = msg->points[i].z;
                    // point.intensity = msg->points[i].reflectivity;
                    point.intensity = msg->points[i].line;  // FIXME: replaced by scan line for feature extraction
                    point.curvature = time_offset + msg->points[i].offset_time * 1e-6;  // ms
                    cloud_out->push_back(point);
            }
        }

        std::sort(cloud_out->points.begin(), cloud_out->points.end(), [](const PointType& x, const PointType& y) 
                        -> bool { return (x.curvature < y.curvature); });
    }

    void getLivoxFeature(const std::pair<double, pcl::PointCloud<PointType>::Ptr> &cloud_info, 
                          pcl::PointCloud<PointType>::Ptr &surf, pcl::PointCloud<PointType>::Ptr &corn)
    {
        auto lidar_timestamp_ros = ros::Time().fromSec(cloud_info.first);

        pcl::PointCloud<PointType> pl_orig = *cloud_info.second;
        uint plsize = pl_orig.size();

        std::vector<pcl::PointCloud<PointType>> pl_buff(N_SCAN);
        std::vector<std::vector<orgtype>> typess(N_SCAN);
        pcl::PointCloud<PointType> pl_full, pl_corn, pl_surf;

        for (int i = 0; i < N_SCAN; i++) {
            pl_buff[i].reserve(plsize);
        }
  
        for (uint i = 0; i < plsize; i++) {
            if (pl_orig.points[i].intensity < N_SCAN) {
                pl_full[i].x = pl_orig.points[i].x;
                pl_full[i].y = pl_orig.points[i].y;
                pl_full[i].z = pl_orig.points[i].z;
                pl_full[i].intensity =  pl_orig.points[i].intensity;  // scan line
                pl_buff[pl_orig.points[i].intensity].push_back(pl_full[i]);
            }
        }

        for (int j = 0; j < N_SCAN; j++) {
            pcl::PointCloud<PointType> &pl = pl_buff[j];
            std::vector<orgtype> &types = typess[j];
            plsize = pl.size();
            types.resize(plsize);
            plsize--;
            for (uint i = 0; i < plsize; i++) {
                types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
                vx = pl[i].x - pl[i + 1].x;
                vy = pl[i].y - pl[i + 1].y;
                vz = pl[i].z - pl[i + 1].z;
                types[i].dista = vx * vx + vy * vy + vz * vz;
            }
            types[plsize].range = sqrt(pl[plsize].x * pl[plsize].x + pl[plsize].y * pl[plsize].y);

            give_feature(pl, types, pl_corn, pl_surf);
        }

        *surf += pl_surf;
        *corn += pl_corn;

        publishCloud(pub_full, &pl_orig, lidar_timestamp_ros, "map");
        publishCloud(pub_surf, &pl_surf, lidar_timestamp_ros, "map");
        publishCloud(pub_corn, &pl_corn, lidar_timestamp_ros, "map");
    }

    void give_feature(pcl::PointCloud<PointType> &pl, std::vector<orgtype> &types, pcl::PointCloud<PointType> &pl_corn, pcl::PointCloud<PointType> &pl_surf) {
        uint plsize = pl.size();
        uint plsize2;
        if (plsize == 0) {
            printf("something wrong\n");
            return;
        }
        uint head = 0;

        while (types[head].range < blind) {
            head++;
        }

        plsize2 = plsize - group_size;

        Eigen::Vector3d curr_direct(Eigen::Vector3d::Zero());
        Eigen::Vector3d last_direct(Eigen::Vector3d::Zero());

        uint i_nex, i2;
        uint last_i = 0; uint last_i_nex = 0;
        int last_state = 0;
        int plane_type;

        for (uint i = head; i < plsize2; i++) {
            if (types[i].range < blind) {
                continue;
            }

            i2 = i;
            plane_type = plane_judge(pl, types, i, i_nex, curr_direct);
    
            if (plane_type == 1) {
                for (uint j = i; j <= i_nex; j++) { 
                    if (j != i && j != i_nex) {
                        types[j].ftype = Real_Plane;
                    }
                    else {
                        types[j].ftype = Poss_Plane;
                    }
                }
      
                if (last_state==1 && last_direct.norm()>0.1) {
                    double mod = last_direct.transpose() * curr_direct;
        
                    if (mod>-0.707 && mod<0.707) {
                        types[i].ftype = Edge_Plane;
                    }
                    else {
                        types[i].ftype = Real_Plane;
                    }
                }
      
                i = i_nex - 1;
                last_state = 1;
            }
            else if (plane_type == 2) {
                i = i_nex;
                last_state = 0;
            }
            else if (plane_type == 0) {
                if (last_state == 1) {
                    uint i_nex_tem;
                    uint j;
                    for(j = last_i + 1; j <= last_i_nex; j++) {
                        uint i_nex_tem2 = i_nex_tem;
                        Eigen::Vector3d curr_direct2;

                        uint ttem = plane_judge(pl, types, j, i_nex_tem, curr_direct2);

                        if (ttem != 1) {
                            i_nex_tem = i_nex_tem2;
                            break;
                        }
                        curr_direct = curr_direct2;
                    }

                    if (j == last_i + 1) {
                        last_state = 0;
                    }
                    else {
                        for (uint k = last_i_nex; k <= i_nex_tem; k++) {
                            if(k != i_nex_tem) {
                                types[k].ftype = Real_Plane;
                            }
                            else {
                                types[k].ftype = Poss_Plane;
                            }
                        }
                        i = i_nex_tem-1;
                        i_nex = i_nex_tem;
                        i2 = j-1;
                        last_state = 1;
                    }

                }
            }

            last_i = i2;
            last_i_nex = i_nex;
            last_direct = curr_direct;
        }

        plsize2 = plsize - 3;
        for (uint i = head + 3; i < plsize2; i++) {
            if (types[i].range < blind || types[i].ftype >= Real_Plane) {
                continue;
            }

            if (types[i-1].dista < 1e-16 || types[i].dista < 1e-16) {
                continue;
            }

            Eigen::Vector3d vec_a(pl[i].x, pl[i].y, pl[i].z);
            Eigen::Vector3d vecs[2];

            for (int j = 0; j < 2; j++) {
                int m = -1;
                if (j == 1) {
                    m = 1;
                }

                if (types[i + m].range < blind) {
                    if (types[i].range > inf_bound) {
                        types[i].edj[j] = Nr_inf;
                    }
                    else {
                        types[i].edj[j] = Nr_blind;
                    }
                    continue;
                }

                vecs[j] = Eigen::Vector3d(pl[i + m].x, pl[i + m].y, pl[i + m].z);
                vecs[j] = vecs[j] - vec_a;
      
                types[i].angle[j] = vec_a.dot(vecs[j]) / vec_a.norm() / vecs[j].norm();
                if (types[i].angle[j] < jump_up_limit) {
                    types[i].edj[j] = Nr_180;
                }
                else if (types[i].angle[j] > jump_down_limit) {
                    types[i].edj[j] = Nr_zero;
                }
            }

            types[i].intersect = vecs[Prev].dot(vecs[Next]) / vecs[Prev].norm() / vecs[Next].norm();
            if (types[i].edj[Prev] == Nr_nor && types[i].edj[Next] == Nr_zero && types[i].dista > 0.0225 && types[i].dista > 4 * types[i - 1].dista) {
                if (types[i].intersect > cos160) {
                    if (edge_jump_judge(pl, types, i, Prev)) {
                        types[i].ftype = Edge_Jump;
                    }
                }
            }
            else if (types[i].edj[Prev] == Nr_zero && types[i].edj[Next] == Nr_nor && types[i - 1].dista > 0.0225 && types[i-1].dista > 4 * types[i].dista) {
                if (types[i].intersect > cos160) {
                    if (edge_jump_judge(pl, types, i, Next)) {
                        types[i].ftype = Edge_Jump;
                    }
                }
            }
            else if (types[i].edj[Prev] == Nr_nor && types[i].edj[Next] == Nr_inf) {
                if (edge_jump_judge(pl, types, i, Prev)) {
                    types[i].ftype = Edge_Jump;
                }
            }
            else if (types[i].edj[Prev] == Nr_inf && types[i].edj[Next] == Nr_nor) {
                if (edge_jump_judge(pl, types, i, Next)) {
                    types[i].ftype = Edge_Jump;
                } 
            }
            else if (types[i].edj[Prev] > Nr_nor && types[i].edj[Next] > Nr_nor) {
                if (types[i].ftype == Nor) {
                    types[i].ftype = Wire;
                }
            }
        }

        plsize2 = plsize - 1;
        double ratio;
        for (uint i = head + 1; i < plsize2; i++) {
            if (types[i].range < blind || types[i - 1].range < blind || types[i + 1].range < blind) {
                continue;
            }
    
            if (types[i - 1].dista < 1e-8 || types[i].dista < 1e-8) {
                continue;
            }

            if (types[i].ftype == Nor) {
                if (types[i-1].dista > types[i].dista) {
                    ratio = types[i-1].dista / types[i].dista;
                }
                else {
                    ratio = types[i].dista / types[i-1].dista;
                }

                if (types[i].intersect < smallp_intersect && ratio < smallp_ratio) {
                    if (types[i - 1].ftype == Nor) {
                        types[i - 1].ftype = Real_Plane;
                    }
                    if (types[i + 1].ftype == Nor) {
                        types[i + 1].ftype = Real_Plane;
                    }
                    types[i].ftype = Real_Plane;
                }
            }
        }

        int last_surface = -1;
        for (uint j = head; j < plsize; j++) {
            if (types[j].ftype == Poss_Plane || types[j].ftype == Real_Plane) {
                if (last_surface == -1) {
                    last_surface = j;
                }
      
                if (j == uint(last_surface + point_filter_num - 1)) {
                    PointType ap;
                    for (uint k = last_surface; k <= j; k++) {
                        ap.x += pl[k].x;
                        ap.y += pl[k].y;
                        ap.z += pl[k].z;
                    }
                    ap.x /= point_filter_num;
                    ap.y /= point_filter_num;
                    ap.z /= point_filter_num;
                    pl_surf.push_back(ap);
                    last_surface = -1;
                }
            }
            else {
                if (types[j].ftype == Edge_Jump || types[j].ftype == Edge_Plane) {
                    pl_corn.push_back(pl[j]);
                }
                if (last_surface != -1) {
                    PointType ap;
                    for (uint k = last_surface; k < j; k++) {
                        ap.x += pl[k].x;
                        ap.y += pl[k].y;
                        ap.z += pl[k].z;
                    }
                    ap.x /= (j - last_surface);
                    ap.y /= (j - last_surface);
                    ap.z /= (j - last_surface);
                    pl_surf.push_back(ap);
                }
                last_surface = -1;
            }
        }
    }

    int plane_judge(const pcl::PointCloud<PointType> &pl, std::vector<orgtype> &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct) {
        double group_dis = disA*types[i_cur].range + disB;
        group_dis = group_dis * group_dis;

        double two_dis;
        std::vector<double> disarr;
        disarr.reserve(20);

        for (i_nex = i_cur; i_nex < i_cur+group_size; i_nex++) {
            if (types[i_nex].range < blind) {
                curr_direct.setZero();
                return 2;
            }
            disarr.push_back(types[i_nex].dista);
        }
  
        for ( ; ; ) {
            if (types[i_nex].range < blind) {
                curr_direct.setZero();
                return 2;
            }
            vx = pl[i_nex].x - pl[i_cur].x;
            vy = pl[i_nex].y - pl[i_cur].y;
            vz = pl[i_nex].z - pl[i_cur].z;
            two_dis = vx * vx + vy * vy + vz * vz;
            if (two_dis >= group_dis) {
                break;
            }
            disarr.push_back(types[i_nex].dista);
            i_nex++;
        }

        double leng_wid = 0;
        double v1[3], v2[3];
        for (uint j = i_cur + 1; j < i_nex; j++) {
            v1[0] = pl[j].x - pl[i_cur].x;
            v1[1] = pl[j].y - pl[i_cur].y;
            v1[2] = pl[j].z - pl[i_cur].z;

            v2[0] = v1[1] * vz - vy * v1[2];
            v2[1] = v1[2] * vx - v1[0] * vz;
            v2[2] = v1[0] * vy - vx * v1[1];

            double lw = v2[0] * v2[0] + v2[1] * v2[1] + v2[2] * v2[2];
            if (lw > leng_wid) {
                leng_wid = lw;
            }
        }


        if ((two_dis*two_dis/leng_wid) < p2l_ratio) {
            curr_direct.setZero();
            return 0;
        }

        uint disarrsize = disarr.size();
        for (uint j = 0; j < disarrsize-1; j++) {
            for (uint k = j + 1; k < disarrsize; k++) {
                if (disarr[j] < disarr[k]) {
                    leng_wid = disarr[j];
                    disarr[j] = disarr[k];
                    disarr[k] = leng_wid;
                }
            }
        }

        if (disarr[disarr.size() - 2] < 1e-16) {
            curr_direct.setZero();
            return 0;
        }

        if (sensor != SensorType::VELODYNE && sensor != SensorType::OUSTER) {
            double dismax_mid = disarr[0] / disarr[disarrsize / 2];
            double dismid_min = disarr[disarrsize / 2]/disarr[disarrsize - 2];

            if (dismax_mid >= limit_maxmid || dismid_min >= limit_midmin) {
                curr_direct.setZero();
                return 0;
            }
        }
        else {
            double dismax_min = disarr[0] / disarr[disarrsize - 2];
            if (dismax_min >= limit_maxmin) {
                curr_direct.setZero();
                return 0;
            }
        }
  
        curr_direct << vx, vy, vz;
        curr_direct.normalize();
        return 1;
    }


    bool edge_jump_judge(const pcl::PointCloud<PointType> &pl, std::vector<orgtype> &types, uint i, Surround nor_dir) {
        if (nor_dir == 0) {
            if (types[i - 1].range<blind || types[i - 2].range < blind) {
                return false;
            }
        }
        else if (nor_dir == 1) {
            if (types[i + 1].range < blind || types[i + 2].range < blind) {
                return false;
            }
        }

        double d1 = types[i + nor_dir - 1].dista;
        double d2 = types[i + 3 * nor_dir - 2].dista;
        double d;

        if (d1 < d2) {
            d = d1;
            d1 = d2;
            d2 = d;
        }

        d1 = sqrt(d1);
        d2 = sqrt(d2);

        if (d1 > edgea * d2 || (d1 - d2) > edgeb) {
            return false;
        }
  
        return true;
    }

    bool SyncMeasurements() {
        static bool measurement_pushed = false;
        static bool process_lidar = false;
        static double lidar_mean_scantime = 0.0;
        static int lidar_scan_num = 0;
        static SensorMeasurement local_sensor_measurement;

        if (cloud_buff.empty() || imu_buff.empty()) {
            return false;
        }

        std::lock_guard<std::mutex> lock(buff_mutex);

        double lidar_end_time = 0.0;
        if (!measurement_pushed) {
            if (!process_lidar) {
                local_sensor_measurement.cloud_ptr_ = cloud_buff.front().second;
                local_sensor_measurement.bag_time_ = cloud_buff.front().first;
                if (!local_sensor_measurement.cloud_ptr_->points.empty()) {
                    local_sensor_measurement.lidar_start_time_ = cloud_buff.front().first + local_sensor_measurement.cloud_ptr_->points.front().curvature / (double)(1000);
                } 
                else {
                    local_sensor_measurement.lidar_start_time_ = cloud_buff.front().first;
                }

                if (local_sensor_measurement.cloud_ptr_->size() <= 1) {
                    ROS_WARN_STREAM("Too Few Points in Cloud !");
                    lidar_end_time = local_sensor_measurement.lidar_start_time_ + lidar_mean_scantime;
                } 
                else if (local_sensor_measurement.cloud_ptr_->points.back().curvature / (double)(1000) < 0.5 * lidar_mean_scantime) {
                    lidar_end_time = local_sensor_measurement.lidar_start_time_ + lidar_mean_scantime;
                } 
                else {
                    lidar_scan_num++;
                    lidar_end_time = local_sensor_measurement.bag_time_ + local_sensor_measurement.cloud_ptr_->points.back().curvature / (double)(1000);
                    lidar_mean_scantime += ((local_sensor_measurement.cloud_ptr_->points.back().curvature - local_sensor_measurement.cloud_ptr_->points.front().curvature) 
                                            / (double)(1000) - lidar_mean_scantime) / (double)(lidar_scan_num);
                }

                local_sensor_measurement.lidar_end_time_ = lidar_end_time;
                    
                process_lidar = true;
            }

            measurement_pushed = true;

            sensor_measurement = local_sensor_measurement;

            cloud_buff.pop_front();
            process_lidar = false;
        }

        if (imu_buff.back().header.stamp.toSec() < sensor_measurement.lidar_end_time_) {
            return false;
        }

        sensor_measurement.imu_buff_.clear();
        while (!imu_buff.empty()) {
            double imu_time = imu_buff.front().header.stamp.toSec();
            if (imu_time < sensor_measurement.lidar_end_time_) {
                sensor_measurement.imu_buff_.push_back(imu_buff.front());
                imu_buff.pop_front();
            } 
            else {
                break;
            }
        }
        sensor_measurement.imu_buff_.push_back(imu_buff.front());

        measurement_pushed = false;

        undistortPointCloud();

        return true;
    }

    void resetMeasurement() {
        SensorMeasurement new_sensor_measurement;
        sensor_measurement = new_sensor_measurement;
    }

    bool undistortPointCloud() {
        if (sensor_measurement.cloud_ptr_->size() < 1) {
            return false;
        }

        // TODO: undistort ??

        // TODO: get feature
        sensor_measurement.plane_ptr_.reset(new pcl::PointCloud<PointType>());
        sensor_measurement.edge_ptr_.reset(new pcl::PointCloud<PointType>());

        switch(sensor) {
            case SensorType::VELODYNE:
                getVelodyneFeature(std::make_pair(sensor_measurement.bag_time_, sensor_measurement.cloud_ptr_), sensor_measurement.plane_ptr_, sensor_measurement.edge_ptr_);
                break;
            
            case SensorType::OUSTER:
                getOusterFeature(std::make_pair(sensor_measurement.bag_time_, sensor_measurement.cloud_ptr_), sensor_measurement.plane_ptr_, sensor_measurement.edge_ptr_);
                break;

            default:
                getLivoxFeature(std::make_pair(sensor_measurement.bag_time_, sensor_measurement.cloud_ptr_), sensor_measurement.plane_ptr_, sensor_measurement.edge_ptr_);
                break;
        }
        
        return true;
    }

};

#endif 
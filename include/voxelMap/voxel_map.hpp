#pragma once

#ifndef _VOXEL_MAP_HPP
#define _VOXEL_MAP_HPP

#include <list>
#include <vector>
#include <memory>
#include <Eigen/SVD>
#include <Eigen/Core>
#include <unordered_map>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace voxelMap
{

enum MODE {
    NEARBY_1,
    NEARBY_7,
    NEARBY_19,
    NEARBY_26
};

bool compare(std::pair<Eigen::Vector3d, double> &p1, std::pair<Eigen::Vector3d, double> &p2) {
    return p1.second < p2.second;
}

struct PointCov {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PointCov(Eigen::Vector3d &p, Eigen::Matrix3d &c) : point(p), cov(c) {}
    Eigen::Vector3d point;
    Eigen::Matrix3d cov;
};

class HashUtil {
private:
    size_t hash_p_ = 116101;
    size_t max_n_ = 10000000000;
    double resolution_inv_ = 1.0;

public:
    HashUtil() = default;

    HashUtil(double resolution) : resolution_inv_(1.0 / resolution) {}

    HashUtil(double resolution, size_t hash_p, size_t max_n) : hash_p_(hash_p), max_n_(max_n), resolution_inv_(1.0 / resolution) {}

    size_t operator()(const Eigen::Vector3d &point) {
        double loc_xyz[3];
        for (size_t i = 0; i < 3; ++i) {
            loc_xyz[i] = point[i] * resolution_inv_;
            if (loc_xyz[i] < 0) {
                loc_xyz[i] -= 1.0;
            }
        }
        size_t x = static_cast<size_t>(loc_xyz[0]);
        size_t y = static_cast<size_t>(loc_xyz[1]);
        size_t z = static_cast<size_t>(loc_xyz[2]);

        return ((((z)*hash_p_) % max_n_ + (y)) * hash_p_) % max_n_ + (x);
    }
};

struct Grid {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    size_t hash = 0;
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    Eigen::Matrix3d conv = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d conv_inv = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d conv_sum = Eigen::Matrix3d::Zero();
    Eigen::Vector3d points_sum = Eigen::Vector3d::Zero();
    size_t points_num = 0;
    size_t min_num = 6;
    size_t max_num = 20;
    bool is_valid = false;
    std::vector<Eigen::Vector3d> points;
    bool is_updated = false;

    Grid(size_t num) : max_num(num), is_updated(false) { points.reserve(2 * max_num); }

    void setMinMax(size_t min_n, size_t max_n) {
        min_num = min_n;
        max_num = max_n;
    }

    void updateConv() {
        if (is_updated)
            return;
        is_updated = true;
        if (points_num < min_num) {
            is_valid = false;
            return;
        }
        conv = (conv_sum - points_sum * centroid.transpose()) / (static_cast<double>(points_num) - 1.0);
        is_valid = true;
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(conv, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Vector3d val(1, 1, 1e-3);
        conv = svd.matrixU() * val.asDiagonal() * svd.matrixV().transpose();
        conv_inv = conv.inverse();
    }

    void addPoint(Eigen::Vector3d &point, bool insert) {
        is_updated = false;
        points_num++;
        points_sum += point;
        centroid = points_sum / static_cast<double>(points_num);
        conv_sum += (point * point.transpose());
        if (points.size() >= max_num * 2 || !insert)
            return;
        points.push_back(point);
    }
};

class VoxelMap {
private:
    MODE mode_ = MODE::NEARBY_7;
    double resolution_ = 1.0;
    double resolution_inv_ = 1.0;
    size_t capacity_ = 5000000;
    size_t grid_capacity_ = 20;
    HashUtil hash_util_;
    std::vector<Eigen::Vector3d> delta_;

    std::vector<size_t> current_idx_;
    std::list<size_t> cache_;
    std::unordered_map<size_t, std::pair<std::list<size_t>::iterator, std::shared_ptr<Grid>>> storage_;

    void initializeDelta() {
        delta_.clear();
        switch (mode_) {
        case MODE::NEARBY_1:
            delta_.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            break;
        case MODE::NEARBY_7:
            delta_.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            delta_.push_back(Eigen::Vector3d(resolution_, 0.0, 0.0));
            delta_.push_back(Eigen::Vector3d(-resolution_, 0.0, 0.0));
            delta_.push_back(Eigen::Vector3d(0.0, resolution_, 0.0));
            delta_.push_back(Eigen::Vector3d(0.0, -resolution_, 0.0));
            delta_.push_back(Eigen::Vector3d(0.0, 0.0, resolution_));
            delta_.push_back(Eigen::Vector3d(0.0, 0.0, -resolution_));
            break;
        case MODE::NEARBY_19:
            delta_.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            delta_.push_back(Eigen::Vector3d(resolution_, 0.0, 0.0));
            delta_.push_back(Eigen::Vector3d(-resolution_, 0.0, 0.0));
            delta_.push_back(Eigen::Vector3d(0.0, resolution_, 0.0));
            delta_.push_back(Eigen::Vector3d(0.0, -resolution_, 0.0));
            delta_.push_back(Eigen::Vector3d(0.0, 0.0, resolution_));
            delta_.push_back(Eigen::Vector3d(0.0, 0.0, -resolution_));
            delta_.push_back(Eigen::Vector3d(resolution_, resolution_, 0.0));
            delta_.push_back(Eigen::Vector3d(resolution_, -resolution_, 0.0));
            delta_.push_back(Eigen::Vector3d(resolution_, 0.0, resolution_));
            delta_.push_back(Eigen::Vector3d(resolution_, 0.0, -resolution_));
            delta_.push_back(Eigen::Vector3d(0.0, resolution_, resolution_));
            delta_.push_back(Eigen::Vector3d(0.0, resolution_, -resolution_));
            delta_.push_back(Eigen::Vector3d(-resolution_, resolution_, 0.0));
            delta_.push_back(Eigen::Vector3d(-resolution_, -resolution_, 0.0));
            delta_.push_back(Eigen::Vector3d(-resolution_, 0.0, resolution_));
            delta_.push_back(Eigen::Vector3d(-resolution_, 0.0, -resolution_));
            delta_.push_back(Eigen::Vector3d(0.0, -resolution_, resolution_));
            delta_.push_back(Eigen::Vector3d(0.0, -resolution_, -resolution_));
            break;
        default:
            delta_.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            delta_.push_back(Eigen::Vector3d(resolution_, 0.0, 0.0));
            delta_.push_back(Eigen::Vector3d(-resolution_, 0.0, 0.0));
            delta_.push_back(Eigen::Vector3d(0.0, resolution_, 0.0));
            delta_.push_back(Eigen::Vector3d(0.0, -resolution_, 0.0));
            delta_.push_back(Eigen::Vector3d(0.0, 0.0, resolution_));
            delta_.push_back(Eigen::Vector3d(0.0, 0.0, -resolution_));
            delta_.push_back(Eigen::Vector3d(resolution_, resolution_, 0.0));
            delta_.push_back(Eigen::Vector3d(resolution_, -resolution_, 0.0));
            delta_.push_back(Eigen::Vector3d(resolution_, 0.0, resolution_));
            delta_.push_back(Eigen::Vector3d(resolution_, 0.0, -resolution_));
            delta_.push_back(Eigen::Vector3d(0.0, resolution_, resolution_));
            delta_.push_back(Eigen::Vector3d(0.0, resolution_, -resolution_));
            delta_.push_back(Eigen::Vector3d(-resolution_, resolution_, 0.0));
            delta_.push_back(Eigen::Vector3d(-resolution_, -resolution_, 0.0));
            delta_.push_back(Eigen::Vector3d(-resolution_, 0.0, resolution_));
            delta_.push_back(Eigen::Vector3d(-resolution_, 0.0, -resolution_));
            delta_.push_back(Eigen::Vector3d(0.0, -resolution_, resolution_));
            delta_.push_back(Eigen::Vector3d(0.0, -resolution_, -resolution_));
            delta_.push_back(Eigen::Vector3d(resolution_, resolution_, resolution_));
            delta_.push_back(Eigen::Vector3d(resolution_, resolution_, -resolution_));
            delta_.push_back(Eigen::Vector3d(resolution_, -resolution_, resolution_));
            delta_.push_back(Eigen::Vector3d(resolution_, -resolution_, -resolution_));
            delta_.push_back(Eigen::Vector3d(-resolution_, resolution_, resolution_));
            delta_.push_back(Eigen::Vector3d(-resolution_, resolution_, -resolution_));
            delta_.push_back(Eigen::Vector3d(-resolution_, -resolution_, resolution_));
            delta_.push_back(Eigen::Vector3d(-resolution_, -resolution_, -resolution_));
            break;
        }
    }

public:
    VoxelMap(double resolution, size_t capacity, size_t grid_capacity, MODE mode)
        : resolution_(resolution),
          capacity_(capacity),
          grid_capacity_(grid_capacity),
          mode_(mode),
          hash_util_(resolution)
    {
        initializeDelta();
    }

    bool isSameGrid(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2) {
        int hx_1 = floor(p1.x() * resolution_inv_);
        int hy_1 = floor(p1.y() * resolution_inv_);
        int hz_1 = floor(p1.z() * resolution_inv_);
        int hx_2 = floor(p2.x() * resolution_inv_);
        int hy_2 = floor(p2.y() * resolution_inv_);
        int hz_2 = floor(p2.z() * resolution_inv_);

        return ((hx_1 == hx_2) && (hy_1 == hy_2) && (hz_1 == hz_2));
    }

    size_t addCloud(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud) {
        size_t error_count = 0;
        if (cloud->empty())
            return 0;
        current_idx_.clear();
        for (pcl::PointXYZINormal &point : cloud->points) {
            Eigen::Vector3d point_vec(point.x, point.y, point.z);
            size_t idx = hash_util_(point_vec);
            current_idx_.push_back(idx);
            auto iter = storage_.find(idx);
            if (iter == storage_.end()) {
                std::shared_ptr<Grid> grid = std::make_shared<Grid>(grid_capacity_);
                grid->addPoint(point_vec, true);
                cache_.push_front(idx);
                storage_.insert({idx, {cache_.begin(), grid}});
                if (storage_.size() >= capacity_) {
                    storage_.erase(cache_.back());
                    cache_.pop_back();
                }
            }
            else {
                if (!isSameGrid(iter->second.second->centroid, point_vec)) {
                    error_count++;
                    cache_.erase(iter->second.first);
                    std::shared_ptr<Grid> grid = std::make_shared<Grid>(grid_capacity_);
                    grid->addPoint(point_vec, true);
                    cache_.push_front(idx);
                    storage_[idx].first = cache_.begin();
                    storage_[idx].second = grid;
                }
                else {
                    size_t point_in_grid = iter->second.second->points_num;
                    if (point_in_grid < 50)
                        iter->second.second->addPoint(point_vec, point_in_grid < grid_capacity_ ? true : false);
                    cache_.splice(cache_.begin(), cache_, iter->second.first);
                    storage_[idx].first = cache_.begin();
                }
            }
        }
        for (size_t &idx : current_idx_) {
            storage_[idx].second->updateConv();
        }
        return error_count;
    }

    void reset() {
        cache_.clear();
        storage_.clear();
    }

    bool searchKNN(const Eigen::Vector3d &point, size_t K, double range, std::vector<Eigen::Vector3d> &results) {
        std::vector<std::pair<Eigen::Vector3d, double>> candidates;
        double range2 = range * range;
        for (const Eigen::Vector3d &delta : delta_) {
            Eigen::Vector3d near_by = point + delta;
            size_t hash_idx = hash_util_(near_by);
            auto iter = storage_.find(hash_idx);
            if (iter != storage_.end() && isSameGrid(near_by, iter->second.second->centroid)) {
                for (Eigen::Vector3d &p : iter->second.second->points)
                {
                    double dist = (point - p).squaredNorm();
                    if (dist < range2)
                        candidates.emplace_back(p, dist);
                }
            }
        }

        if (candidates.empty())
            return false;

        if (candidates.size() > K) {
            std::nth_element(
                candidates.begin(),
                candidates.begin() + K - 1,
                candidates.end(),
                compare);
            candidates.resize(K);
        }
        std::nth_element(candidates.begin(), candidates.begin(), candidates.end(), compare);

        results.clear();

        for (const auto &it : candidates) {
            results.emplace_back(it.first);
        }
        return true;
    }

    bool getCentroidAndCovariance(size_t hash_idx, Eigen::Vector3d &centroid, Eigen::Matrix3d &cov) {
        auto iter = storage_.find(hash_idx);
        if (iter != storage_.end() && iter->second.second->is_valid) {
            centroid = iter->second.second->centroid;
            cov = iter->second.second->conv;
            return true;
        }
        return false;
    }

    bool getCentroidAndCovariance(const Eigen::Vector3d &point, Eigen::Vector3d &centroid, Eigen::Matrix3d &cov) {
        size_t idx = hash_util_(point);
        return getCentroidAndCovariance(idx, centroid, cov);
    }
};

struct FastGrid {
public:
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    size_t num = 0;

    void addPoint(const Eigen::Vector3d &point) {
        num += 1;
        centroid += ((point - centroid) / static_cast<double>(num));
    }
};


class FastVoxelMap {
private:
    HashUtil hash_util_;
    double resolution_;
    std::vector<Eigen::Vector3d> search_range_;
    std::vector<std::shared_ptr<FastGrid>> grid_array_;
    std::unordered_map<size_t, std::shared_ptr<FastGrid>> grid_map_;  

public:
    FastVoxelMap(double resolution) : resolution_(resolution), hash_util_(resolution) {
        search_range_.clear();
        for (int x_gain = -1; x_gain <= 1; ++x_gain) {
            for (int y_gain = -1; y_gain <= 1; ++y_gain) {
                for (int z_gain = -1; z_gain <= 1; ++z_gain) {
                    search_range_.emplace_back(Eigen::Vector3d(x_gain * resolution_,
                                                               y_gain * resolution_,
                                                               z_gain * resolution_));
                }
            }
        }
    }

    void filter(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud, std::vector<PointCov> &outs) {
        if (cloud->empty())
            return;
        outs.clear();
        grid_array_.clear();
        grid_map_.clear();
        grid_array_.reserve(cloud->size());
        for (pcl::PointXYZINormal &p : cloud->points) {
            Eigen::Vector3d p_vec(p.x, p.y, p.z);
            size_t idx = hash_util_(p_vec);
            auto iter = grid_map_.find(idx);
            if (iter == grid_map_.end()) {
                std::shared_ptr<FastGrid> grid = std::make_shared<FastGrid>();
                grid->addPoint(p_vec);
                grid_map_.insert({idx, grid});
                grid_array_.push_back(grid);
            }
            else {
                iter->second->addPoint(p_vec);
            }
        }

        for (std::shared_ptr<FastGrid> &g : grid_array_) {
            Eigen::Vector3d points_sum = Eigen::Vector3d::Zero();
            Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
            size_t points_num = 0;
            for (Eigen::Vector3d &r : search_range_) {
                Eigen::Vector3d near = g->centroid + r;
                auto iter = grid_map_.find(hash_util_(near));
                if (iter != grid_map_.end()) {
                    points_sum += iter->second->centroid;
                    cov += iter->second->centroid * iter->second->centroid.transpose();
                    points_num += 1;
                }
            }
            if (points_num >= 6) {
                Eigen::Vector3d centriod = points_sum / static_cast<double>(points_num);
                cov = (cov - points_sum * centriod.transpose()) / (static_cast<double>(points_num) - 1.0);
                Eigen::JacobiSVD<Eigen::Matrix3d> svd(cov, Eigen::ComputeFullU | Eigen::ComputeFullV);
                Eigen::Vector3d values(1, 1, 1e-3);
                cov = svd.matrixU() * values.asDiagonal() * svd.matrixV().transpose();
            }
            outs.emplace_back(g->centroid, cov);
        }
    }  
};   
    
}


#endif
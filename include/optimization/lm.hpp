#ifndef _LM_HPP_
#define _LM_HPP_

#include "../utility.h"

class LM : public ParamServer {
public:
    bool point_selected_edge[100000] = {false};
    bool point_selected_surf[100000] = {false};
    pcl::PointCloud<PointType>::Ptr normvec;
    pcl::PointCloud<PointType>::Ptr dirvec;
    std::vector<KD_TREE<PointType>::PointVector> near_edge;
    std::vector<KD_TREE<PointType>::PointVector> near_surf; 

    pcl::PointCloud<PointType>::Ptr effect_surf;
    pcl::PointCloud<PointType>::Ptr coeff_surf;
    pcl::PointCloud<PointType>::Ptr effect_edge;
    pcl::PointCloud<PointType>::Ptr coeff_edge;

    pcl::PointCloud<PointType>::Ptr effect_all;
    pcl::PointCloud<PointType>::Ptr coeff_all;

    int edge_cloud_size = 0;
    int surf_cloud_size = 0;

    ~LM() { }
    LM() { }

    void allocateMemory() {
        normvec.reset(new pcl::PointCloud<PointType>(100000, 1));
        dirvec.reset(new pcl::PointCloud<PointType>(100000, 1));
        effect_surf.reset(new pcl::PointCloud<PointType>(100000, 1));
        coeff_surf.reset(new pcl::PointCloud<PointType>(100000, 1));
        effect_edge.reset(new pcl::PointCloud<PointType>(100000, 1));
        coeff_edge.reset(new pcl::PointCloud<PointType>(100000, 1));

        effect_all.reset(new pcl::PointCloud<PointType>());
        coeff_all.reset(new pcl::PointCloud<PointType>());
    }

    void runptimization(KD_TREE<PointType> *edge_tree_lio, const pcl::PointCloud<PointType>::Ptr &edge_cloud,
                        KD_TREE<PointType> *surf_tree_lio, const pcl::PointCloud<PointType>::Ptr &surf_cloud,
                        State& cur_state) 
    {
        for (int iter = 0; iter < LM_ITERATION; iter++) {
            reset();

            edgeOptimization(edge_tree_lio, edge_cloud, cur_state);
            surfOptimization(surf_tree_lio, surf_cloud, cur_state);

            combineOptimizationCoeffs();

            if (LMOptimization(iter, cur_state)) {
                break;
            }
        }
    }

    void edgeOptimization(KD_TREE<PointType> *edge_tree_lio, const pcl::PointCloud<PointType>::Ptr &edge_cloud, const State& cur_state) {
        edge_cloud_size = edge_cloud->size();

        #pragma omp parallel for num_threads(numberOfCores)
        for (size_t i = 0; i < edge_cloud_size; i++) {
            PointType pointOri, pointWorld, coeff;
            pointOri = edge_cloud->points[i];
            pointAssociateToMap(&pointOri, &pointWorld, cur_state);

            std::vector<float> pointSearchSqDis(NUM_MATCH_POINTS);
            auto &points_near = near_edge[i];

            edge_tree_lio->Nearest_Search(pointWorld, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
            point_selected_edge[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > MATCH_DIS ? false : true;

            if(!point_selected_edge[i]) 
                continue;
            
            point_selected_edge[i] = false;

            Eigen::Matrix<float, 4, 1> pabcd;
            if(esti_line(pabcd, points_near, float(LINE_TH), pointWorld)) {
                float s = 1 - 0.9 * fabs(pabcd(3));

                if (s > 0.9) {  // FIXME: the value of 's'
                    point_selected_edge[i] = true;
                    effect_edge->points[i] = pointOri;

                    dirvec->points[i].x = pabcd(0);
                    dirvec->points[i].y = pabcd(1);
                    dirvec->points[i].z = pabcd(2);
                    dirvec->points[i].intensity = pabcd(3);

                    coeff.x = s * pabcd(0);
                    coeff.y = s * pabcd(1);
                    coeff.z = s * pabcd(2);
                    coeff.intensity = s * pabcd(3);
                    coeff_edge->points[i] = coeff;
                }
            }
        }
    }

    void surfOptimization(KD_TREE<PointType> *surf_tree_lio, const pcl::PointCloud<PointType>::Ptr &surf_cloud, const State& cur_state) {
        surf_cloud_size = surf_cloud->size();

        #pragma omp parallel for num_threads(numberOfCores)
        for (size_t j = 0; j < surf_cloud->size(); j++) {
            PointType pointOri, pointWorld, coeff;
            pointOri = surf_cloud->points[j];
            pointAssociateToMap(&pointOri, &pointWorld, cur_state);

            std::vector<float> pointSearchSqDis(NUM_MATCH_POINTS);
            auto &points_near = near_surf[j];

            surf_tree_lio->Nearest_Search(pointWorld, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
            point_selected_surf[j] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > MATCH_DIS ? false : true;

            if (!point_selected_surf[j])
                continue;
            
            Eigen::Matrix<float, 4, 1> pabcd;
            point_selected_surf[j] = false;
            if (esti_plane(pabcd, points_near, float(PLANE_TH))) {
                float pd2 = pabcd(0) * pointWorld.x + pabcd(1) * pointWorld.y + pabcd(2) * pointWorld.z + pabcd(3);
                float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointOri.x * pointOri.x + pointOri.y * pointOri.y + pointOri.z * pointOri.z));

                if (s > 0.1) {  // FIXME: the value of 's'
                    point_selected_surf[j] = true;
                    effect_surf->points[j] = pointOri;

                    normvec->points[j].x = pabcd(0);
                    normvec->points[j].y = pabcd(1);
                    normvec->points[j].z = pabcd(2);
                    normvec->points[j].intensity = pd2;

                    coeff.x = s * pabcd(0);
                    coeff.y = s * pabcd(1);
                    coeff.z = s * pabcd(2);
                    coeff.intensity = s * pd2;
                    coeff_surf->points[j] = coeff;
                }
            }
        }
    }

    void combineOptimizationCoeffs() {
        for (size_t i = 0; i < edge_cloud_size; i++) {
            if (point_selected_edge[i]) {
                effect_all->push_back(effect_edge->points[i]);
                coeff_all->push_back(coeff_edge->points[i]);
            }
        }

        for (size_t j = 0; j < surf_cloud_size; j++) {
            if (point_selected_surf[j]) {
                effect_all->push_back(effect_surf->points[j]);
                coeff_all->push_back(coeff_surf->points[j]);
            }
        }
    }

    bool LMOptimization(const int& iterCount, State& cur_state) {
        // This optimization is from the original loam_velodyne by Ji Zhang, need to cope with coordinate transformation
        // lidar <- camera      ---       camera <- lidar
        // x = z                ---       x = y
        // y = x                ---       y = z
        // z = y                ---       z = x
        // roll = yaw           ---       roll = pitch
        // pitch = roll         ---       pitch = yaw
        // yaw = pitch          ---       yaw = roll
        // XXX: we use lidar -> camera

        float rpyxyz[6];
        state2RPYXYZ(rpyxyz, cur_state);

        float srx = sin(rpyxyz[1]);
        float crx = cos(rpyxyz[1]);
        float sry = sin(rpyxyz[2]);
        float cry = cos(rpyxyz[2]);
        float srz = sin(rpyxyz[0]);
        float crz = cos(rpyxyz[0]);

        const int effect_count = effect_all->size();
        if (effect_count < 50) {
            return false;
        }

        Eigen::Matrix<float, Eigen::Dynamic, 6> matA(effect_count, 6);
        Eigen::Matrix<float, 6, Eigen::Dynamic> matAt(6, effect_count);
        Eigen::Matrix<float, 6, 6> matAtA(6, 6);
        Eigen::Matrix<float, Eigen::Dynamic, 1> matB(effect_count, 1);
        Eigen::Matrix<float, 6, 1> matAtB(6, 1);
        Eigen::Matrix<float, 6, 1> matX(6, 1);

        PointType pointOri, coeff;

        for (size_t i = 0; i < effect_count; i++) {
            pointOri.x = effect_all->points[i].y;
            pointOri.y = effect_all->points[i].z;
            pointOri.z = effect_all->points[i].x;

            coeff.x = coeff_all->points[i].y;
            coeff.y = coeff_all->points[i].z;
            coeff.z = coeff_all->points[i].x;
            coeff.intensity = coeff_all->points[i].intensity;

            float arx = (crx*sry*srz*pointOri.x + crx*crz*sry*pointOri.y - srx*sry*pointOri.z) * coeff.x
                        + (-srx*srz*pointOri.x - crz*srx*pointOri.y - crx*pointOri.z) * coeff.y
                        + (crx*cry*srz*pointOri.x + crx*cry*crz*pointOri.y - cry*srx*pointOri.z) * coeff.z;

            float ary = ((cry*srx*srz - crz*sry)*pointOri.x
                        + (sry*srz + cry*crz*srx)*pointOri.y + crx*cry*pointOri.z) * coeff.x
                        + ((-cry*crz - srx*sry*srz)*pointOri.x
                        + (cry*srz - crz*srx*sry)*pointOri.y - crx*sry*pointOri.z) * coeff.z;

            float arz = ((crz*srx*sry - cry*srz)*pointOri.x + (-cry*crz-srx*sry*srz)*pointOri.y)*coeff.x
                        + (crx*crz*pointOri.x - crx*srz*pointOri.y) * coeff.y
                        + ((sry*srz + cry*crz*srx)*pointOri.x + (crz*sry-cry*srx*srz)*pointOri.y)*coeff.z;

      
            matA(i, 0) = arz;
            matA(i, 1) = arx;
            matA(i, 2) = ary;
            matA(i, 3) = coeff.z;
            matA(i, 4) = coeff.x;
            matA(i, 5) = coeff.y;
            matB(i, 0) = -coeff.intensity;
        }

        matAt = matA.transpose();
        matAtA = matAt * matA;
        matAtB = matAt * matB;
        matX = matAtA.colPivHouseholderQr().solve(matAtB);

        Eigen::Matrix<float, 1, 6> matE(1, 6);
        Eigen::Matrix<float, 6, 6> matV(6, 6);
        Eigen::Matrix<float, 6, 6> matV2(6, 6);
        Eigen::Matrix<float, 6, 6> matP(6, 6);
        bool isDegenerate = false;

        if (iterCount == 0) {
            Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> eigenSolver(matAtA);

            if (eigenSolver.info() != Eigen::Success) {
                return false;
            }

            matE = eigenSolver.eigenvalues(); 
            matV = eigenSolver.eigenvectors();
            matV2 = matV;

            float eignThre[6] = {100, 100, 100, 100, 100, 100};
            for (int i = 5; i >= 0; i--) {
                if (matE(0, i) < eignThre[i]) {
                    for (int j = 0; j < 6; j++) {
                        matV2(i, j) = 0;
                    }
                    isDegenerate = true;
                } else {
                    break;
                }
            }
            matP = matV.inverse() * matV2;
        }

        if (isDegenerate) {
            Eigen::Matrix<float, 6, 1> matX2(6, 1);
            matX2 = matX;
            matX = matP * matX2;
        }

        rpyxyz[0] += matX(0, 0);
        rpyxyz[1] += matX(1, 0);
        rpyxyz[2] += matX(2, 0);
        rpyxyz[3] += matX(3, 0);
        rpyxyz[4] += matX(4, 0);
        rpyxyz[5] += matX(5, 0);

        float deltaR = sqrt(pow(pcl::rad2deg(matX(0, 0)), 2) + pow(pcl::rad2deg(matX(1, 0)), 2) + pow(pcl::rad2deg(matX(2, 0)), 2));
        float deltaT = sqrt(pow(matX(3, 0) * 100, 2) + pow(matX(4, 0) * 100, 2) + pow(matX(5, 0) * 100, 2));

        if (deltaR < 0.05 && deltaT < 0.05) {
            RPYXYZ2State(rpyxyz, cur_state);
            return true;
        }

        return false;
    }

    void reset() {
        std::fill(std::begin(point_selected_edge), std::end(point_selected_edge), false);
        std::fill(std::begin(point_selected_surf), std::end(point_selected_surf), false);

        normvec->clear();
        dirvec->clear();

        near_edge.clear();
        near_surf.clear();

        effect_surf->clear();
        coeff_surf->clear();
        effect_edge->clear();
        coeff_edge->clear();
        effect_all->clear();
        coeff_all->clear();

        edge_cloud_size = 0;
        surf_cloud_size = 0;
    }
};

#endif
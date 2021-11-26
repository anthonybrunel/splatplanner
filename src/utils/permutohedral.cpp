#include "../../include/utils/permutohedral.h"


//FilterReg implementation of permutohedral filter for 3d features only


#define DIM_PERMUTO 3
#include <Eigen/Core>


float permutohedral_scale_noblur(int index) {
    return (DIM_PERMUTO + 1) * sqrtf((1.0f / 6.0f) / ((index + 1) * (index + 2)));
}

void permutohedral_lattice_noblur(
        const float *feature,
        LatticeCoordKey *lattice_coord_keys,
        float *barycentric
) {
    float elevated[DIM_PERMUTO + 1];
//    elevated[DIM_PERMUTO] = -DIM_PERMUTO * (feature[DIM_PERMUTO - 1]) * permutohedral_scale_noblur(DIM_PERMUTO - 1);

//    for (int i = DIM_PERMUTO - 1; i > 0; i--) {
//        elevated[i] = (elevated[i + 1] -
//                       i * (feature[i - 1]) * permutohedral_scale_noblur(i - 1) +
//                       (i + 2) * (feature[i]) * permutohedral_scale_noblur(i));
//    }
//    elevated[0] = elevated[1] + 2 * (feature[0]) * permutohedral_scale_noblur(0);

    float scaleFactor[DIM_PERMUTO];
    float invStdDev = (DIM_PERMUTO + 1) * sqrt(2.0 / 3);
    for (int i = 0; i < DIM_PERMUTO; i++) {
        scaleFactor[i] = 1.0 / (sqrt((i + 1) * (i + 2))) * invStdDev;
    }
    float sm = 0;
    for (int i = DIM_PERMUTO; i > 0; i--) {
        float cf = feature[i - 1] * scaleFactor[i - 1];
        elevated[i] = sm - i * cf;
        sm += cf;
    }
    elevated[0] = sm;


    short greedy[DIM_PERMUTO + 1];
    signed short sum = 0;
    for (int i = 0; i <= DIM_PERMUTO; i++) {
        float v = elevated[i] * (1.0f / (DIM_PERMUTO + 1));
        float up = ceilf(v) * (DIM_PERMUTO + 1);
        float down = floorf(v) * (DIM_PERMUTO + 1);
        if (up - elevated[i] < elevated[i] - down) {
            greedy[i] = (signed short) up;
        } else {
            greedy[i] = (signed short) down;
        }
        sum += greedy[i];
    }
    sum /= DIM_PERMUTO + 1;

    short rank[DIM_PERMUTO + 1] = {0};
    for (int i = 0; i < DIM_PERMUTO; i++) {
        for (int j = i + 1; j <= DIM_PERMUTO; j++) {
            if (elevated[i] - greedy[i] < elevated[j] - greedy[j])
                rank[i]++;
            else
                rank[j]++;
        }
    }

    if (sum > 0) {
        for (int i = 0; i <= DIM_PERMUTO; i++) {
            if (rank[i] >= DIM_PERMUTO + 1 - sum) {
                greedy[i] -= DIM_PERMUTO + 1;
                rank[i] += sum - (DIM_PERMUTO + 1);
            } else {
                rank[i] += sum;
            }
        }
    } else if (sum < 0) {
        for (int i = 0; i <= DIM_PERMUTO; i++) {
            if (rank[i] < -sum) {
                greedy[i] += DIM_PERMUTO + 1;
                rank[i] += (DIM_PERMUTO + 1) + sum;
            } else {
                rank[i] += sum;
            }
        }
    }

    for (int i = 0; i <= DIM_PERMUTO + 1; i++) {
        barycentric[i] = 0;
    }

    for (int i = 0; i <= DIM_PERMUTO; i++) {
        float delta = (elevated[i] - greedy[i]) * (1.0f / (DIM_PERMUTO + 1));


        barycentric[DIM_PERMUTO - rank[i]] += delta;
        barycentric[DIM_PERMUTO + 1 - rank[i]] -= delta;
    }
    barycentric[0] += 1.0f + barycentric[DIM_PERMUTO + 1];

    for (auto color = 0; color <= DIM_PERMUTO; color++) {
        short *key = lattice_coord_keys[color].key;

        for (int i = 0; i < DIM_PERMUTO; i++) {
            key[i] = greedy[i] + color;
            if (rank[i] > DIM_PERMUTO - color) key[i] -= (DIM_PERMUTO + 1);
        }
    }
}


void buildPermutohedralpt2pt(const Eigen::Matrix<float,3,Eigen::Dynamic> &cloud,
                                                phmap::flat_hash_map<LatticeCoordKey, LatticeInfo, Hasher>& lattice_map_Y,
                                                const Eigen::Vector3f &invsigma) {
    lattice_map_Y.clear();

    int dim = cloud.rows();



    Eigen::Vector3f scaled_feature;

    LatticeCoordKey lattice_key[dim + 1];
    float lattice_weight[dim + 2];

    for(auto i = 0; i < cloud.cols(); i++) {

        const Eigen::Vector3f& vertex_i = cloud.col(i);
        scaled_feature[0] = cloud(0,i) * invsigma(0);
        scaled_feature[1] = cloud(1,i) * invsigma(1);
        scaled_feature[2] = cloud(2,i) * invsigma(2);

        permutohedral_lattice_noblur(scaled_feature.data(), lattice_key, lattice_weight);
        for(auto lattice_j_idx = 0; lattice_j_idx < dim + 1; lattice_j_idx++) {
            const auto& lattice_j = lattice_key[lattice_j_idx];
            const float weight_j = lattice_weight[lattice_j_idx];

            auto iter = lattice_map_Y.find(lattice_j);

            if(iter == lattice_map_Y.end()) {
                LatticeInfo info;
                info.weight = weight_j;
                info.weight_vertex = weight_j * vertex_i;
                info.weight_vTv = weight_j * vertex_i.dot(vertex_i);
                lattice_map_Y.emplace(lattice_j, info);

            } else {
                LatticeInfo& info = iter->second;
                info.weight += weight_j;
                info.weight_vertex += weight_j * vertex_i;
                info.weight_vTv += weight_j * vertex_i.dot(vertex_i);
            }
        }
    }

}

void computeTargetpt2pt(const Eigen::Matrix<float,3,Eigen::Dynamic> &cloud,
                                           const phmap::flat_hash_map<LatticeCoordKey, LatticeInfo, Hasher> &lattice_map_Y,
                                           const Eigen::Vector3f &invsigma,
                                           std::vector<Aggregatept2pt> &results,const GMMParameters & params) {



    int dim = cloud.rows();

    float scaled_feature[3];
    LatticeCoordKey lattice_key[dim + 1];
    float lattice_weight[dim + 2];
    for(auto i = 0; i < cloud.cols(); i++) {
        scaled_feature[0] = cloud(0,i) * invsigma(0);
        scaled_feature[1] = cloud(1,i) * invsigma(1);
        scaled_feature[2] = cloud(2,i) * invsigma(2);

        permutohedral_lattice_noblur(scaled_feature, lattice_key, lattice_weight);
        LatticeInfo info;
//
        for(auto lattice_j_idx = 0; lattice_j_idx < dim + 1; lattice_j_idx++) {


            const auto& lattice_j = lattice_key[lattice_j_idx];
            const float weight_j = lattice_weight[lattice_j_idx];

            auto iter = lattice_map_Y.find(lattice_j);


            if(iter != lattice_map_Y.end()) {
                const LatticeInfo &lattice_info = iter->second;
                info.weight += weight_j * lattice_info.weight;
                info.weight_vertex += weight_j * lattice_info.weight_vertex;
                info.weight_vTv += weight_j * lattice_info.weight_vTv;
            }
        }


        results[i].idx = i;
        results[i].pos_ = cloud.col(i);
        if(info.weight > 1e-2f) {
            results[i]._M0 = info.weight;
            results[i]._M1 = info.weight_vertex * (1.f/info.weight);
            results[i]._M2 = info.weight_vTv *(1.f/info.weight);
            results[i]._woutlier = info.weight / (info.weight + params.outlier_cst);


        }else{
            results[i]._M0 = 0;
            results[i]._M1 = Eigen::Vector3f(0,0,0);
            results[i]._M2 = 0;
            results[i]._woutlier = 0;
        }

    }

}

#include <iostream>
void blur(const phmap::flat_hash_map<LatticeCoordKey, LatticeInfo, Hasher> &lattice_map_Y, bool reverse)
{


        //old and new values contain the lattice points before and after blur
        //auto new_values = new T[vd * hashTable.size()];
        LatticeInfo new_values[lattice_map_Y.size()];

//        auto zero = new T[vd]{0};
        //for (int k = 0; k < vd; k++)
        //    zero[k] = 0;

        // For each of pd+1 axes,
        LatticeCoordKey latticeKeyLeft[DIM_PERMUTO + 1];
        LatticeCoordKey latticeKeyRight[DIM_PERMUTO + 1];
        for (int remainder=reverse?DIM_PERMUTO:0; remainder >= 0 && remainder <= DIM_PERMUTO; reverse?remainder--:remainder++){
            // For each vertex in the lattice,
            size_t i =0;
            for (auto it = lattice_map_Y.begin(); it != lattice_map_Y.end(); ++it,++i) { // blur point i in dimension j
//                short *key = lattice_map_Y.getKeys() + i * pd; // keys to current vertex


                for (int k = 0; k < DIM_PERMUTO; k++) {
                    latticeKeyLeft->key[k] = it->first.key[k] + 1;
                    latticeKeyRight->key[k] = it->first.key[k] - 1;
                }

                latticeKeyLeft->key[remainder] = it->first.key[remainder] - DIM_PERMUTO;
                latticeKeyRight->key[remainder] = it->first.key[remainder] + DIM_PERMUTO; // keys to the neighbors along the given axis.


                new_values[i].weight = it->second.weight*0.5f;

                for(auto k = 0; k < DIM_PERMUTO + 1; k++) {
                    const auto& k_left = latticeKeyLeft[k];
                    const auto& k_right = latticeKeyRight[k];

                    auto iter = lattice_map_Y.find(k_left);


                    //inc left
                    if(iter != lattice_map_Y.end()) {
                        new_values[i].weight += iter->second.weight *0.25;
                        new_values[i].weight_vertex += iter->second.weight_vertex *0.25;
                        new_values[i].weight_vTv += iter->second.weight_vTv *0.25;
                    }

                    iter = lattice_map_Y.find(k_right);

                    if(iter != lattice_map_Y.end()) {
                        new_values[i].weight += iter->second.weight *0.25;
                        new_values[i].weight_vertex += iter->second.weight_vertex *0.25;
                        new_values[i].weight_vTv += iter->second.weight_vTv *0.25;

                    }
                }
            }
            i=0;
            for(auto &val: lattice_map_Y){
                const_cast<LatticeInfo&>(val.second).weight = new_values[i].weight;
                const_cast<LatticeInfo&>(val.second).weight_vTv = new_values[i].weight_vTv;
                const_cast<LatticeInfo&>(val.second).weight_vertex = new_values[i].weight_vertex;

                ++i;

            }
        }
}

void updatePermutoWeight(const Eigen::Matrix<float, 3, Eigen::Dynamic> &cloud, phmap::flat_hash_map<LatticeCoordKey, LatticeInfo, Hasher> &lattice_map_Y, const Eigen::Vector3f &invsigma)
{
    int dim = cloud.rows();

    Eigen::Vector3f scaled_feature;

    LatticeCoordKey lattice_key[dim + 1];
    float lattice_weight[dim + 2];

    float multiplier = 1000;

    for(auto i = 0; i < cloud.cols(); i++) {

        const Eigen::Vector3f& vertex_i = cloud.col(i);
        scaled_feature[0] = cloud(0,i) * invsigma(0);
        scaled_feature[1] = cloud(1,i) * invsigma(1);
        scaled_feature[2] = cloud(2,i) * invsigma(2);

        permutohedral_lattice_noblur(scaled_feature.data(), lattice_key, lattice_weight);
        for(auto lattice_j_idx = 0; lattice_j_idx < dim + 1; lattice_j_idx++) {
            const auto& lattice_j = lattice_key[lattice_j_idx];
            const float weight_j = lattice_weight[lattice_j_idx];

            auto iter = lattice_map_Y.find(lattice_j);

            if(iter == lattice_map_Y.end()) {
                LatticeInfo info;
                info.weight = weight_j*multiplier;
                info.weight_vertex = weight_j * vertex_i*multiplier;
                info.weight_vTv = weight_j * vertex_i.dot(vertex_i)*multiplier;
                lattice_map_Y.emplace(lattice_j, info);

            } else {
                LatticeInfo& info = iter->second;
                info.weight += weight_j*multiplier;
                info.weight_vertex += weight_j * vertex_i*multiplier;
                info.weight_vTv += weight_j * vertex_i.dot(vertex_i)*multiplier;
            }
        }
    }
}

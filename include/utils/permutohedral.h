#pragma once

//FilterReg implementation of permutohedral filter for 3d features only



#define PERMUTO_DIM 3
#include <Eigen/Core>
#include "external/parallel_hashmap/phmap.h"
#include <vector>

struct LatticeInfo {
    float weight = 0;
    Eigen::Vector3f weight_vertex = Eigen::Vector3f(0, 0, 0);
    float weight_vTv = 0;

    float outlier_coeff = 0.0;
};

struct Aggregatept2pt
{

    size_t idx = 0;
    Eigen::Vector3f pos_ = Eigen::Vector3f(0,0,0);
    float _M2 = 0.f;
    Eigen::Vector3f _M1 = Eigen::Vector3f(0,0,0);
    float _M0 = 0.f;
    float _woutlier = 0.0f;
};

struct GMMParameters{
    int max_iter = 40;
    bool stop_optimization = false;
    float outlier_cst = 0.2;
    Eigen::Vector3f init_invsigma = Eigen::Vector3f(1.0f/1.0f,1.f/1.0f,1.f/1.0f);
};

struct LatticeCoordKey {
    short key[PERMUTO_DIM];

    unsigned hash() const {
        unsigned hash_value = 0;
        for (auto i = 0; i < PERMUTO_DIM; i++) {
            hash_value += key[i];
            hash_value *= 1500007;

        }
        return hash_value;
    }

    //The comparator of a key
    char less_than(const LatticeCoordKey &rhs) const {
        char is_less_than = 0;
        for (auto i = 0; i < PERMUTO_DIM; i++) {
            if (key[i] < rhs.key[i]) {
                is_less_than = 1;
                break;
            } else if (key[i] > rhs.key[i]) {
                is_less_than = -1;
                break;
            }
            //Else, continue
        }
        return is_less_than;
    }

    //Operator
    bool operator==(const LatticeCoordKey &rhs) const {
        for (auto i = 0; i < PERMUTO_DIM; i++) {
            if (key[i] != rhs.key[i]) return false;
        }
        return true;
    }
};

struct Hasher {
    std::size_t operator()(const LatticeCoordKey &lattice) const {
        return lattice.hash();
    }
};



float permutohedral_scale_noblur(int index);

void permutohedral_lattice_noblur(
        const float *feature,
        LatticeCoordKey *lattice_coord_keys,
        float *barycentric
        );

void buildPermutohedralpt2pt(const Eigen::Matrix<float,3,Eigen::Dynamic> &cloud,
                             phmap::flat_hash_map<LatticeCoordKey, LatticeInfo, Hasher>& lattice_map_Y,
                             const Eigen::Vector3f& sigma);

void updatePermutoWeight(const Eigen::Matrix<float,3,Eigen::Dynamic> &cloud,
                         phmap::flat_hash_map<LatticeCoordKey, LatticeInfo, Hasher>& lattice_map_Y,
                         const Eigen::Vector3f& invsigma);

void computeTargetpt2pt(const Eigen::Matrix<float,3,Eigen::Dynamic> &cloud,
                                           const phmap::flat_hash_map<LatticeCoordKey, LatticeInfo, Hasher> &lattice_map_Y,
                                           const Eigen::Vector3f& sigma,
                                           std::vector<Aggregatept2pt> &results,const GMMParameters & params);


void blur(const phmap::flat_hash_map<LatticeCoordKey, LatticeInfo, Hasher> &lattice_map_Y, bool reverse = false);

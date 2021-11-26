#ifndef NN_HASHMAP_HPP
#define NN_HASHMAP_HPP
#include <Eigen/Dense>

#include "external/parallel_hashmap/phmap.h"




static size_t __p1 = 73856093;
static size_t __p2 = 19349669;
static size_t __p3 = 83492791;



class Cell{
public:
    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> pts_;
    Cell(){
        pts_.reserve(2);
    }
    void insert(const Eigen::Vector3f &p){
        pts_.push_back(p);
    }
};



struct CellCoordKey{
    Eigen::Vector3i coord_;

    int64_t hash_value ()const
    {
          return ((__p1 * static_cast<int64_t>(coord_.x()))^
                  (__p2 * static_cast<int64_t>(coord_.y()))^
                  (__p3 * static_cast<int64_t>(coord_.z()))) % static_cast<int64_t>(1e9);
    }

    bool operator==(const CellCoordKey &rhs) const {
        for (auto i = 0; i < 3; i++) {
            if (coord_[i] != rhs.coord_[i]) return false;
        }
        return true;
    }
};

struct Hasher{
    int64_t operator()(const CellCoordKey &coord_cell) const {
        return coord_cell.hash_value();
    }
};

class NNHashMap{
    float inv_resolution_ = 1./1.5;
    Eigen::Vector3f half_ = Eigen::Vector3f(0.5,0.5,0.5);
    phmap::flat_hash_map<Cell, Cell, Hasher> buffer_;

    void checkNN(const Eigen::Vector3f &p){
        Eigen::Vector3i c = ((p*inv_resolution_)+half_).cast<int>();




    }
    void insert(const Eigen::Vector3f &p){

    }
};

#endif // NN_HASHMAP_HPP

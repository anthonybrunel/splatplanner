#ifndef BOUNDINGVOLUME_H
#define BOUNDINGVOLUME_H

#include <Eigen/Core>



class BoundingVolume
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    BoundingVolume();
    BoundingVolume(const Eigen::Vector3f &min_i, const Eigen::Vector3f &max_i);

    float clamp(float n, float lower, float upper) {
      return std::max(lower, std::min(n, upper));
    }
    void clamp(Eigen::Vector3f & pos_i){
        pos_i.x() = clamp(pos_i.x(),min_.x(),max_.x());
        pos_i.y() = clamp(pos_i.y(),min_.y(),max_.y());
        pos_i.z() = clamp(pos_i.z(),min_.z(),max_.z());

    }


    Eigen::Vector3f max_;
    Eigen::Vector3f min_;

};

#endif // BOUNDINGVOLUME_H

#include "../../include/utils/boundingvolume.h"

BoundingVolume::BoundingVolume()
{

}

BoundingVolume::BoundingVolume(const Eigen::Vector3f &min_i, const Eigen::Vector3f &max_i)
{
    min_ = max_i;
    max_ = max_i;
}

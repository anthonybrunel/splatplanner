#include "include/map/mapsearchspace.h"

#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl_conversions/pcl_conversions.h>

MapSearchSpace::MapSearchSpace()
{

}

MapSearchSpace::MapSearchSpace(const ros::NodeHandle &nh_i, const ros::NodeHandle &nh_private_i):nh_(nh_i),nh_privaite_(nh_private_i)
{
    mapServer_.reset(new MapCoreServer(nh_i,nh_private_i));
    Eigen::Vector3f map_size;
    std::cout << "Map Searche Space INITIALIZED\n";

    robot_size_ = Eigen::Vector3f(1.2,1.2,0.7);
}

void MapSearchSpace::init(ros::NodeHandle &nh_i)
{
//    mapServeur_.init(nh_i);
//    nh_ = nh_i;
//    float resolution;
//    Eigen::Vector3f map_size;
//    std::cout << "Map Searche Space INITIALIZED\n";

//    robot_size_ = Eigen::Vector3f(1.2,1.2,1.2);
}



float MapSearchSpace::evaluate_node(Eigen::Vector3f pos_i)
{
    float dist =1000;
    mapServer_->getDistance(pos_i,dist);
}

bool MapSearchSpace::is_free(const Eigen::Vector3f &pos_i)
{
    float dist = 10000;
    mapServer_->getDistance(pos_i,dist);
    return dist > robot_size_.x() && dist > robot_size_.y() && dist > robot_size_.z();
}

bool MapSearchSpace::is_free(const Eigen::Vector3i &pos_i)
{
    float dist = 10000;
    mapServer_->getDistance(pos_i,dist);
    return dist > robot_size_.x() && dist > robot_size_.y() && dist > robot_size_.z() && mapServer_->getState(pos_i);
}


bool MapSearchSpace::checkOcclusion(const Eigen::Vector3f &pos, const Eigen::Vector3f &goal)
{
    bool display = false;//(goal.y() < 0.5 || goal.y() > -0.5) && goal.x() > 2.2 && goal.x() > 1.5;
    //inversé goal étant la pos du drone
    //    if(!map_->is_inside(pos) || !map_->is_inside(goal))
    //        return false;
    Eigen::Matrix<int,3,1> start;
    Eigen::Matrix<int,3,1> end;
    mapServer_->convert(goal,end);
    mapServer_->convert(pos,start);
    int i, dx, dy, dz, l, m, n, x_inc, y_inc, z_inc, err_1, err_2, dx2, dy2,
            dz2;

    if(!mapServer_->is_inside(goal)){
        Eigen::Vector3i res;

        Eigen::Vector3f dir = (end-start).cast<float>().normalized();
        if(!clampGridRay(start, dir, res)){
            return 0;
        }

        end = res;
    }
    dx = end[0] - start[0];
    dy = end[1] - start[1];
    dz = end[2] - start[2];



    x_inc = (dx < 0) ? -1 : 1;
    l = abs(dx);
    y_inc = (dy < 0) ? -1 : 1;
    m = abs(dy);
    z_inc = (dz < 0) ? -1 : 1;
    n = abs(dz);
    dx2 = l << 1;
    dy2 = m << 1;
    dz2 = n << 1;
    uint8_t tmp;



    if ((l >= m) && (l >= n)) {
        err_1 = dy2 - l;
        err_2 = dz2 - l;
        for (i = 0; i < l; i++) {
            tmp = getState(start);

            if((tmp & VoxelBuffer::occupied_flag)){
                return 0;
            }
            if (err_1 > 0) {
                start[1] += y_inc;
                err_1 -= dx2;
            }
            if (err_2 > 0) {
                start[2] += z_inc;
                err_2 -= dx2;
            }
            err_1 += dy2;
            err_2 += dz2;
            start[0] += x_inc;
        }
    } else if ((m >= l) && (m >= n)) {
        err_1 = dx2 - m;
        err_2 = dz2 - m;
        for (i = 0; i < m; i++) {
            tmp = getState(start);

            if((tmp & VoxelBuffer::occupied_flag)){
                return 0;
            }


            if (err_1 > 0) {
                start[0] += x_inc;
                err_1 -= dy2;
            }
            if (err_2 > 0) {
                start[2] += z_inc;
                err_2 -= dy2;
            }
            err_1 += dx2;
            err_2 += dz2;
            start[1] += y_inc;
        }
    } else {
        err_1 = dy2 - n;
        err_2 = dx2 - n;
        for (i = 0; i < n; i++) {

            tmp = getState(start);

            if((tmp & VoxelBuffer::occupied_flag)){
                return 0;
            }


            if (err_1 > 0) {
                start[1] += y_inc;
                err_1 -= dz2;
            }
            if (err_2 > 0) {
                start[0] += x_inc;
                err_2 -= dz2;
            }
            err_1 += dy2;
            err_2 += dx2;
            start[2] += z_inc;
        }
    }
    tmp = getState(start);

    if((tmp & VoxelBuffer::occupied_flag)){
        return 0;
    }


    return 1;
}

void MapSearchSpace::getDistanceGrad(const Eigen::Vector3f &pos, float &dist, Eigen::Vector3f &grad)
{
    mapServer_->getDistAndGrad(pos,dist,grad);
}



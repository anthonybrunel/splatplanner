#include "../../include/map/viewutility.h"
#include <opencv2/core.hpp>
#include <string>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common.h>

#include <pcl/common/io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/common/transforms.h>

#include <pcl/ModelCoefficients.h>
#include <limits>


ViewUtility::ViewUtility()
{

}

ViewUtility::ViewUtility(const ros::NodeHandle &nh_i,const ros::NodeHandle &nh_private_i,MapSearchSpace::Ptr& map_i):nh_(nh_i),map_(map_i)
{
    lattice_map.reserve(900000);
    nh_private_i.param("camera/fovx", horizontal_angle, horizontal_angle);
    nh_private_i.param("camera/fovy", height_, height_);
    nh_private_i.param("camera/max_range", max_range_, max_range_-0.2f);
    height_-=4;

    nh_private_i.param("camera/oc_x", t_oc_.x(), 0.f);
    nh_private_i.param("camera/oc_y", t_oc_.y(), 0.f);
    nh_private_i.param("camera/oc_z", t_oc_.z(), 0.f);
    nh_private_i.param("camera/oc_qx", r_oc_.x(), 0.f);
    nh_private_i.param("camera/oc_qy", r_oc_.y(), 0.f);
    nh_private_i.param("camera/oc_qz", r_oc_.z(), 0.f);
    nh_private_i.param("camera/oc_qw", r_oc_.w(), 1.f);
    nh_private_i.param("max_v", max_v_, max_v_);
    nh_private_i.param("mav_radius", safety_radius_, safety_radius_);
    safety_radius_ += map_->getResolution();

    //    depth_sub_ = new message_filters::Subscriber<sensor_msgs::Image>(nh,"/map_core/depth",10);
    std::cout << "[Planner-ViewUtility] fovx;fovy: " << horizontal_angle << ";" << height_  <<std::endl
              << "[Planner-ViewUtility] Max range: " << max_range_ <<std::endl;

    nh_ =nh_i;
    std::cout << "ViewUtility: " <<  horizontal_angle << " " << height_ << " " << max_range_ <<std::endl;
}

ViewUtility::ViewUtility(const ViewUtility &v)
{
    this->nh_ = v.nh_;

    this->map_ = v.map_;
}

float ViewUtility::computeViewUtility(const Eigen::Vector3f &pos_i,float &angle)
{

    frontiers_.clear();
    map_->getFrontiers(frontiers_);
    cvim = cv::Mat::zeros(height_,360,CV_32F);
    Timer t;
    float max_lenght = max_range_;
    Eigen::Vector3f ray;
    Eigen::Vector3i hit;
    im = Eigen::ArrayXXf::Zero(height_,360);
    hits_.clear();
    hits_.reserve(height_*360);

    Eigen::Vector3f pos = pos_i+t_oc_;


    for(int h = 0;h <height_;++h){
        //        float theta = (h-(height_/2.f))*M_PI/180.f;
        float theta = (h-(height_/2.f))*M_PI/180.f+M_PI/2.;
        ray.z() = (max_lenght * cosf(theta)+pos.z());
        for(int w = 0; w < 360; ++w){
            float phi = w*M_PI/180.f;
            ray.x() = max_lenght * sinf(theta) * cosf(phi)+pos.x();
            ray.y() = max_lenght * sinf(theta) * sinf(phi)+pos.y();
            Eigen::Vector3f ray_oc = (ray.transpose()*r_oc_.matrix());
            //            std::cout << ray.transpose() << " " << ray_oc.transpose() <<std::endl;
            cvim.at<float>(h,w) = rayUtility(pos,ray_oc,hit);
            im(h,w) = cvim.at<float>(h,w);
            hits_.push_back(hit);
        }
    }
    Features f;
    double count = 0;
    for(int y = 0; y < height_;++y){
        for(int x = 0; x<360;++x){
            if(hits_[y*360+x].x() < 0 && hits_[y*360+x].y() < 0 && hits_[y*360+x].z() < 0 ){
                continue;
            }
            //            map_->getFrontierFeatures(hits_[y*360+x],f);

            im(y,x) = cvim.at<float>(y,x);;
            //            cvim.at<float>(y,x) = 1;//f.planarity;
            count += cvim.at<float>(y,x);
        }
    }

//    std::cout << "Image score "<< count << std::endl;
    double score;
    angle = getMaximumAngleGain(im,score);

    return score;
    //    std::cout << "t: " << t.elapsed_ms() <<std::endl;
    //    size_t count = 0;
    //    for(size_t i = 0; i < frontiers.size(); ++i){
    //        Eigen::Vector3f f_pos; map_->convert(frontiers[i],f_pos);
    //        Eigen::Vector3f cf_pos = f_pos-pos_i;
    //        float norm = cf_pos.norm();
    //        if(norm < 7 && norm > 0.01){
    //            count++;
    //        }else{
    //            continue;
    //        }
    //        cf_pos.normalize();

    //        float phi = std::atan2(cf_pos.y(),cf_pos.x());//0 2pi
    //        if(phi < 0)
    //            phi+=2*M_PI;
    //        phi = phi * 180.f/M_PI;
    //        int theta = std::acos(cf_pos.z()/norm)*180.f/M_PI-60;//0 180



    //        if(theta < 0 || theta >= 60 || phi >= 360){
    //            continue;
    //        }

    //        std::cout << theta << " " << phi <<std::endl;
    //        image.at<float>(theta,static_cast<int>(phi)) = 1.;
    //    }
    //    std::cout << "Count: " << count << std::endl;

    // Wait for a keystroke in the window
    //    cv::imwrite( "/home/anthony/im.png", image );
}

bool ViewUtility::computeRapidFrontier(const Eigen::Vector3f &pos_i,const Eigen::Vector3f &v_curr,float yaw, Eigen::Vector3f &resultVelocity)
{
    //TODO: optimize covert float to int (real coordinate to grid coordinate mb later)

    frontiers_.clear();
    map_->getFrontiers(frontiers_);
    cvim = cv::Mat::zeros(height_,360,CV_32F);
    Timer t;
    float max_increment = 1;
    float max_lenght = max_range_+max_increment;
    Eigen::Vector3f ray;
    Eigen::Vector3i hit;
    im = Eigen::ArrayXXf::Ones(height_,360)*-1.f;
    hits_.clear();
    hits_.reserve(height_*360);
    Eigen::Vector3f pos = pos_i+t_oc_;
    int yaw_deg = (int)(yaw*180./M_PI);
    if(yaw_deg < 0)
        yaw_deg += 360;
    for(int h = 10;h <height_-10;++h){
        //        float theta = (h-(height_/2.f))*M_PI/180.f;
        float theta = (h-(height_/2.f))*M_PI/180.f+M_PI/2.;
        ray.z() = (max_lenght * cosf(theta)+pos.z());
        for(int w = 0; w < 360; ++w){
            cvim.at<float>(h,w) = -1;
            float diff = (w - yaw_deg + 360) % 360;
            if (diff > 180) diff = 360 - diff;
            if(diff < horizontal_angle/2){//bound fov
                float phi = w*M_PI/180.f;
                ray.x() = max_lenght * sinf(theta) * cosf(phi)+pos.x();
                ray.y() = max_lenght * sinf(theta) * sinf(phi)+pos.y();
                Eigen::Vector3f ray_oc = (ray.transpose()*r_oc_.matrix());
                //            std::cout << ray.transpose() << " " << ray_oc.transpose() <<std::endl;
                cvim.at<float>(h,w) = rayFrontiereTagger(pos,ray_oc,hit);
                im(h,w) = cvim.at<float>(h,w);
            }
            hits_.push_back(hit);

        }
    }

    Features f;
    double count = 0;
    float cost = std::numeric_limits<float>::max();
    bool found_rapid_frontier = false;
    BoundingMap3D<float> bounds_map = map_->getBoudingMapReal();
    Eigen::Vector3f v_curr_tmp = v_curr;
    float curr_speed = v_curr.norm();
    if( curr_speed <0.05){
        //set the velocity in yaw direction if speed is low
        v_curr_tmp = Eigen::Vector3f(0.1,0,0);
        v_curr_tmp.x() = 0.2 * cos(yaw);
        v_curr_tmp.y() = 0.2 * sin(yaw);
        v_curr_tmp.z() = 0;
    }
    //reduce z motion

    for(int y = 0; y < height_;++y){
        for(int x = 0; x<360;++x){
            if((hits_[y*360+x].x() < 0 && hits_[y*360+x].y() < 0 && hits_[y*360+x].z() < 0) || im(y,x) <= 0.2){
                continue;
            }
            Eigen::Vector3f frontier_i;
            map_->convert(hits_[y*360+x],frontier_i);
            if(frontier_i.z() < bounds_map.min_.z()+safety_radius_)
                continue;
            if(frontier_i.z() > bounds_map.max_.z()-safety_radius_)
                continue;
            if(frontier_i.x() < bounds_map.min_.x()+safety_radius_)
                continue;
            if(frontier_i.x() > bounds_map.max_.x()-safety_radius_)
                continue;
            if(frontier_i.y() < bounds_map.min_.y()+safety_radius_)
                continue;
            if(frontier_i.y() > bounds_map.max_.y()-safety_radius_)
                continue;
            Eigen::Vector3f dir = (frontier_i-pos_i);


            //            x = len * cos(yaw);
            //            y = sin(pitch);
            //            z = len * sin(-yaw);


            //            float norm = dir.norm();
            //            Eigen::Vector3f dir_n = dir.normalized();
            //            float dot = dir_n.z();/*Eigen::Vector3f::UnitZ().dot(dir.normalized());*/
            //            float angle = acos(dot);
            //            if(dot < 0){
            //                angle = std::fmax(0,angle+0.0174533);

            //            }else{
            //                angle = std::fmin(0,angle-0.0174533);
            //            }
            //            dir_n.z() = cos(angle);
            //            dir *= norm;
            Eigen::Vector3f v_i = dir*max_v_/(max_range_-4*map_->getResolution());
            float tmp = (v_i-v_curr_tmp).norm();
            //            cvim.at<float>(y,x) = 1;//f.planarity;

            if(cost > tmp){
                cost = tmp;
                resultVelocity = v_i;
                found_rapid_frontier=true;
                cv_circle_ = Eigen::Vector2i(x,y);
            }
        }
    }

    if(found_rapid_frontier){

        if(resultVelocity.norm() > max_v_){
            resultVelocity.normalize();
            resultVelocity *= max_v_;
        }
        float target_v = resultVelocity.norm();
        if(target_v > curr_speed && (target_v-curr_speed)>0.5){
            resultVelocity.normalize();
            resultVelocity *= (curr_speed+0.5f);
        }

        if (target_v < 0.2){
            resultVelocity.normalize();
            resultVelocity *= 0.35;
        }

    }
    //    std::cout << "Image score "<< count << std::endl;
    //    angle = getMaximumAngleGain(im,score);
    //    if(found_rapid_frontier)
    //        std::cout << "rapid vel required: " << resultVelocity.norm() <<std::endl;
    //    else
    //        std::cout << "No rapid frontier found" <<std::endl;

    return found_rapid_frontier;
    
}

float ViewUtility::rayUtility(const Eigen::Vector3f &pos,const Eigen::Vector3f &goal, Eigen::Vector3i &hit)
{
    bool display = false;//(goal.y() < 0.5 || goal.y() > -0.5) && goal.x() > 2.2 && goal.x() > 1.5;
    //inversé goal étant la pos du drone
    //    if(!map_->is_inside(pos) || !map_->is_inside(goal))
    //        return false;
    Eigen::Matrix<int,3,1> start;
    Eigen::Matrix<int,3,1> end;
    hit = Eigen::Vector3i(-1,-1,-1);
    map_->convert(goal,end);
    map_->convert(pos,start);
    Eigen::Vector3i keep_start = start;
    int i, dx, dy, dz, l, m, n, x_inc, y_inc, z_inc, err_1, err_2, dx2, dy2,
            dz2;
    float boarder_volume = 0;
    if(!map_->is_inside(goal)){

        Eigen::Vector3i res;

        Eigen::Vector3f dir = (goal-pos).cast<float>().normalized();


        if(!map_->clampGridRay(start, dir, res)){


            return 0;
        }
        //            exit(1);
        float boarder_volume = 0;
        float r1 = (res-start).norm()*map_->getResolution();
        float r2 = (start-end).norm()*map_->getResolution();
        float dr_vol = (r2*r2*r2-r1*r1*r1);
        boarder_volume = tan_a*tan_a*dr_vol*dr_vol*4./3.;


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

    float score = 0;
    float start_dist = -1;

    auto scores = [&]() -> float {
            if(start_dist < 0)
            return 0;
            float r1 = start_dist;
            float r2 = (keep_start-start).norm()*map_->getResolution();;
            float dr_vol = (r2*r2*r2-r1*r1*r1);
            float volume = tan_a*tan_a*dr_vol*dr_vol*4./3.;

            return volume;
            //            return volume+boarder_volume*0.5f;
    };



    if ((l >= m) && (l >= n)) {
        err_1 = dy2 - l;
        err_2 = dz2 - l;
        for (i = 0; i < l; i++) {

            tmp = map_->getState(start);

            if((tmp & VoxelBuffer::occupied_flag)){
                return scores();
            }else
                if((tmp & VoxelBuffer::frontier_flag)|| !tmp){
                    hit = start;
                    if(start_dist < 0)
                        start_dist = (keep_start-start).norm()*map_->getResolution();
                }else if(start_dist>0){
                    return scores();
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

            tmp = map_->getState(start);

            if((tmp & VoxelBuffer::occupied_flag )){
                return scores();
            }else
                if((tmp & VoxelBuffer::frontier_flag)|| !tmp){
                    hit = start;
                    if(start_dist < 0)
                        start_dist = (keep_start-start).norm()*map_->getResolution();

                }else if(start_dist>0){
                    return scores();
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

            tmp = map_->getState(start);

            if((tmp & VoxelBuffer::occupied_flag )){
                return score;
            }else
                if((tmp & VoxelBuffer::frontier_flag)|| !tmp){
                    hit = start;
                    if(start_dist < 0)
                        start_dist = (keep_start-start).norm()*map_->getResolution();

                }else if(start_dist>0){
                    return scores();
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

    tmp = map_->getState(start);

    if((tmp & VoxelBuffer::occupied_flag )){
        return scores();
    }else
        if((tmp & VoxelBuffer::frontier_flag) || !tmp){
            hit = start;
            if(start_dist < 0)
                start_dist = (keep_start-start).norm()*map_->getResolution();

        }else if(start_dist>0){
            return scores();
        }

    return scores();

}

float ViewUtility::rayFrontiereTagger(const Eigen::Vector3f &pos, const Eigen::Vector3f &goal, Eigen::Vector3i &hit)
{

    bool display = false;//(goal.y() < 0.5 || goal.y() > -0.5) && goal.x() > 2.2 && goal.x() > 1.5;
    //inversé goal étant la pos du drone
    //    if(!map_->is_inside(pos) || !map_->is_inside(goal))
    //        return false;
    Eigen::Matrix<int,3,1> start;
    Eigen::Matrix<int,3,1> end;
    hit = Eigen::Vector3i(-1,-1,-1);
    map_->convert(goal,end);
    map_->convert(pos,start);
    Eigen::Vector3i keep_start = start;
    int i, dx, dy, dz, l, m, n, x_inc, y_inc, z_inc, err_1, err_2, dx2, dy2,
            dz2;

    if(!map_->isInsideBuf(end)){
        Eigen::Vector3i res;

        Eigen::Vector3f dir = (end-start).cast<float>().normalized();
        if(!map_->clampGridRay(start, dir, res)){

            return -1;
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

    float score = 0;
    float start_dist = -1;
    float edt_dist;
    auto scores = [&]() -> float {
            if(start_dist < 0.75){
            hit = Eigen::Vector3i(-1,-1,-1);;
            return -1;

}
            map_->getDistance(start,edt_dist);
            if(edt_dist<safety_radius_){
            return 0;
}
            hit = start;
            return start_dist;

};

    auto validate = [&]() -> bool {
            if(start_dist < 0.5f){
            if(edt_dist<0.1f){
            return false;
}
}else{
            if(edt_dist<safety_radius_){
            return false;
}
}
            return true;
};

    if ((l >= m) && (l >= n)) {
        err_1 = dy2 - l;
        err_2 = dz2 - l;
        for (i = 0; i < l; i++) {
            tmp = map_->getState(start);
            map_->getDistance(start,edt_dist);
            if((tmp & VoxelBuffer::occupied_flag)){
                return -1;
            }

            start_dist = (keep_start-start).norm()*map_->getResolution();

            if(!validate())
                return 0;

            if((tmp & VoxelBuffer::frontier_flag)|| !tmp){

                hit = start;
                return scores();
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
            tmp = map_->getState(start);
            map_->getDistance(start,edt_dist);
            if((tmp & VoxelBuffer::occupied_flag ))
                return -1;
            start_dist = (keep_start-start).norm()*map_->getResolution();

            if(!validate())
                return 0;
            if((tmp & VoxelBuffer::frontier_flag)|| !tmp){

                hit = start;
                return scores();

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

            tmp = map_->getState(start);
            map_->getDistance(start,edt_dist);
            if((tmp & VoxelBuffer::occupied_flag ))
                return -1;
            start_dist = (keep_start-start).norm()*map_->getResolution();

            if(!validate())
                return 0;
            if((tmp & VoxelBuffer::frontier_flag)|| !tmp){

                hit = start;
                return scores();
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
    tmp = map_->getState(start);
    map_->getDistance(start,edt_dist);
    if(edt_dist<safety_radius_){
        return 0;
    }

    if((tmp & VoxelBuffer::occupied_flag )){
        return -1;
    }else
        if((tmp & VoxelBuffer::frontier_flag) ||!tmp){
            start_dist = (keep_start-start).norm()*map_->getResolution();


            hit = start;

            return scores();
        }

    return -1;

}



float ViewUtility::getMaximumAngleGain(const Eigen::ArrayXXf &im,double &max_score)
{
    max_score = 0;
    float score = 0;
    int start_col = 0;
    int best_col = 0;
    Eigen::ArrayXf col_score;
    int size = im.cols()+horizontal_angle-1;
    col_score.resize(size);
    scores.clear();
    scores.resize(im.cols());
    col_score.head(im.cols()) = im.colwise().sum();
    col_score.tail(horizontal_angle-1) = im.block(0,0,im.rows(),horizontal_angle-1).colwise().sum();
    //    std::cout << col_score <<std::endl;
    for(int w = 0; w < size; ++w){
        if(w >= horizontal_angle){
            score -= col_score[start_col];
            start_col +=1;
        }

        score+=col_score[w];
        if(max_score<score){
            max_score = score;
            best_col = start_col;
        }
        scores[start_col] = score;
        //        std::cout <<"[ViewUtility] scores: " << start_col << " " << scores[start_col] << std::endl;
    }
    //    std::cout <<"[ViewUtility]: " << best_col  << " " << scores[best_col] << std::endl;
    return (float)best_col;
}

float ViewUtility::getBestVerticalAngle(int start_angle)
{
    float best_height = 0;
    float weigths = 0;
    start_angle = start_angle%360;
    for(int w = start_angle; w <= start_angle + horizontal_angle;++w){
        int a = w%360;
        int tmp_height = 0;
        float cpt = 0;
        for(int i = 0; i < im.col(a).rows();++i){
            if(im(i,a)> 0){
                tmp_height+=i;
                cpt+=1;
            }
        }
        if(cpt > 0){
            weigths+=cpt/im.col(a).rows();
            best_height+=((float)tmp_height/cpt)*cpt/im.col(a).rows();
        }
    }
    if(weigths <= 0){
        best_height = im.rows()/2.;
    }else{
        best_height = best_height/weigths;
    }
    return best_height;

}


void ViewUtility::drawViewUtility(const Eigen::Vector2i& target)
{
    //    std::cout << target.transpose() <<std::endl;
    cv::normalize(cvim, cvim, 0, 1, cv::NORM_MINMAX);
    cvtColor(cvim, cvim, cv::COLOR_GRAY2BGR);
    cv::circle(cvim, cv::Point(cv_circle_.x(),cv_circle_.y()),5, cv::Scalar(0,0,255));
    cv::waitKey(1);
    //        while(cv::waitKey(1) != 'q'){
    cv::imshow( "Gray image ", cvim );
    //        }
}

int ViewUtility::selectNearestHorizontalFrontier(int x)
{
    int dist = 1000;
    int new_x = x;
    for(int h = 0;h <height_;++h){
        for(int w = 0; w < 360; ++w){
            int tmp;
            if (im(h,w) > 0){
                tmp = degreeDifference(x,w);
                if(dist > tmp){
                    dist = tmp;
                    new_x = w;
                }
            }
        }
    }
    return new_x;
}

void ViewUtility::computeDistribution(int x, int y)
{

    //Compute frontier along best direction of the uav
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    for(int i = 0;i < frontiers_.size(); ++i){

        cloud->points.push_back(pcl::PointXYZ());
        cloud->points[i].x = frontiers_[i].x();
        cloud->points[i].y = frontiers_[i].y();
        cloud->points[i].z = frontiers_[i].z();
    }

    pcl::PointXYZ p;
    p.x = hits_[y*360+x].x();
    p.y = hits_[y*360+x].y();
    p.z = hits_[y*360+x].z();

    if(p.x <= 0 && p.y <= 0 && p.z <= 0 ){
        return;
    }
    std::cout <<"computeDistribution: "<<x << " " << y << " " << p.x << " " << p.y << p.z <<std::endl;
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree(new pcl::KdTreeFLANN<pcl::PointXYZ>());

    std::vector<int> idx;
    std::vector<float> dist;


    kdtree->setInputCloud(cloud);


    kdtree->radiusSearch(p,2,idx,dist);
    Eigen::Matrix3f cov;
    pcl::PointXYZ average_centroid;

    pcl::computeCentroid(*cloud,idx,average_centroid);
    pcl::computeCovarianceMatrixNormalized(*cloud,idx,average_centroid.getVector4fMap(),cov);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(cov, Eigen::ComputeEigenvectors);

    Eigen::Vector3i p_grid = average_centroid.getArray3fMap().cast<int>();
    Eigen::Vector3f coord;
    map_->convert(p_grid,coord);
    target_ = coord;
    //    path_.push(coord);
    //    std::cout << average_centroid.getArray3fMap().transpose() << " " <<  eigen_solver.eigenvectors().col(2).transpose() <<std::endl;
    insertRemovedFrontiers();
}

void ViewUtility::computeLocalDistribution(const Eigen::Vector3f &pos, const Eigen::Vector3f &dir)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    Eigen::Vector3i pos_grid;


    Eigen::Matrix<float,3,Eigen::Dynamic> selected_frontiers;
    selected_frontiers.resize(3,frontiers_.size());
    size_t count = 0;

    for(int i = 0;i < frontiers_.size(); ++i){
        float dist = dir.dot((frontiers_[i] - pos_grid).cast<float>())/map_->getResolution();

        if(dist > 0 && dist < 7){
            Eigen::Vector3f real_frontier;
            map_->convert(frontiers_[i],real_frontier);
            Eigen::Vector3f proj = pos + dir*dist;

            //squared distance to line
            if((proj-real_frontier).squaredNorm() < (25)){
                cloud->points.push_back(pcl::PointXYZ());
                cloud->points[i].x = real_frontier.x();
                cloud->points[i].y = real_frontier.y();
                cloud->points[i].z = real_frontier.z();
                count++;
                selected_frontiers.col(i) = real_frontier;

            }
        }
    }
    //    buildPermutohedralpt2pt(selected_frontiers, lattice_map, params);




    //    path_.push(coord);
    //    std::cout << average_centroid.getArray3fMap().transpose() << " " <<  eigen_solver.eigenvectors().col(2).transpose() <<std::endl;

}

void ViewUtility::insertRemovedFrontiers()
{

    lock_lattice_map.lock();


    Timer t;
    std::vector<Eigen::Vector3i,Eigen::aligned_allocator<Eigen::Vector3i>> removed_frontiersi;
    std::vector<Eigen::Vector3i,Eigen::aligned_allocator<Eigen::Vector3i>> frontiers;
    map_->getRemovedFrontiers(removed_frontiersi);
    //    if(removed_frontiersi.size() == 0){
    //        lock_lattice_map.unlock();
    //        return;
    //    }



    map_->getFrontiers(frontiers);
    Eigen::Matrix<float,3,Eigen::Dynamic> removed_frontiersf;
    removed_frontiersf.resize(3,frontiers.size());
    Eigen::Vector3f means (0,0,0);
    //    for(size_t i = 0; i < removed_frontiersi.size(); ++i){
    //        Eigen::Vector3f tmp ;
    //        map_->convert(removed_frontiersi[i],tmp);
    //        free_space_.push_back(tmp);
    //        means += tmp;
    //    }

    float dist = 100000;



    //    means /= (float)removed_frontiersi.size();
    //    path_.push(means);

    for(size_t i = 0; i < frontiers.size(); ++i){
        Eigen::Vector3f tmp ;
        map_->convert(frontiers[i],tmp);
        removed_frontiersf.col(i) = tmp;
    }
    Eigen::Vector3f target_tmp;
    for(size_t i = 0; i < frontiers.size(); ++i){
        if((removed_frontiersf.col(i)-target_).norm() < dist){
            target_tmp = removed_frontiersf.col(i);
            dist = (removed_frontiersf.col(i)-target_).norm();
        }
    }
    target_ = target_tmp;

    free_space_.clear();
    //    std::cout << target_.transpose() <<std::endl;
    for(float x= -3; x < 3; x+=0.1){
        for(float y= -3; y <3;y+=0.1){
            for(float z= -3; z < 3;z+=0.1){
                Eigen::Vector3f pos = target_+Eigen::Vector3f(x,y,z);
                float obs_dist;
                if(map_-> is_inside(pos)){
                    map_->getDistance(pos,obs_dist);
                    if(obs_dist > 0.6f && (map_->getState(pos) & VoxelBuffer::visited_flag)){
                        free_space_.push_back(pos);
                    }
                }
            }
        }
    }

    float sigma = 1.;
    //    buildPermutohedralpt2pt(removed_frontiersf, lattice_map, 1.2);
    //    std::cout << "Insert frontiers g " << t.elapsed_ms() <<std::endl;
    lock_lattice_map.unlock();

}

void ViewUtility::computeGaussianMap(std::vector<pcl::PointXYZI> &map)
{

    lock_lattice_map.lock();

    Timer t;
    std::vector<Aggregatept2pt> aggregate_results;

    if(free_space_.size() < 10){
        lock_lattice_map.unlock();
        return;
    }
    Eigen::Matrix<float,3,Eigen::Dynamic> grid;
    grid.resize(3,free_space_.size());
    int count = 0;
    for(auto it = free_space_.begin(); it != free_space_.end() ;++it){
        if(it->z() > 0.8 && it->z() < 1.6)
            grid.col(count++) = *it;
    }
    grid.conservativeResize(3,count);

    aggregate_results.resize(count);

    GMMParameters params;

    Eigen::Isometry3f T;
    params.outlier_cst = 0.2f;

    //    float sigma = params.init_sigma;
    //    float sigma_tmp = sigma;
    //    computeTargetpt2pt(grid, lattice_map, 1.2, aggregate_results,params);
    float max_m0 = 0;
    int idx = 0;
    for(size_t i = 0; i < grid.cols(); ++i){
        if(max_m0 < aggregate_results[i]._M0){
            max_m0 = aggregate_results[i]._M0;
            idx = i;
        }
    }

    //        if(grid.size() > 0){
    //            map.resize(1);
    //            map[0].getArray3fMap() = grid.col(idx);
    //            map[0]._PointXYZI::intensity = aggregate_results[idx]._M0/max_m0;
    //        }
    map.resize(count);

    for(size_t i = 0; i < grid.cols(); ++i){

        pcl::PointXYZI p;
        //        grid.col(i) = aggregate_results[i]._M1;//project onto the gaussian cluster
        p.x = grid.col(i).x();
        p.y = grid.col(i).y();
        p.z = grid.col(i).z();
        p.getVector3fMap() = grid.col(i);
        p._PointXYZI::intensity = aggregate_results[i]._M0/max_m0;
        map[i] = p;
    }
    lock_lattice_map.unlock();


}

int ViewUtility::getHorizontal_angle() const
{
    return horizontal_angle;
}

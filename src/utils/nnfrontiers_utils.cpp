#include "../../include/utils/nnfrontiers_utils.h"

#include<bits/stdc++.h>
#include <atomic>
nnfrontiers_utils::nnfrontiers_utils(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private_i, MapSearchSpace::Ptr map_i)

{
    nh_ =nh;
    nh_private_= nh_private_i;
    map_ = map_i;
    nh_private_i.param("camera/fovx", fovx_deg_, 90.f);
    nh_private_i.param("camera/fovy", fovy_deg_, 60.f);

    fovy_deg_ -= 5; //remove 10deg to ensure visibilities
    fovy_rad_ = ((fovy_deg_ * M_PI))/180.;
    fovx_rad_ = ((fovx_deg_ * M_PI))/180.;

    map_->get3DGridSize(grid_max_);
    position_buffer_.resize(grid_max_.x()*grid_max_.y()*grid_max_.z());
    nh_private_.param("mav_radius", radius_, 0.3f);

    djikstra_bounds = map_->getBoudingMapReal();
    //    grid_max_ = ((djikstra_bounds.max_-Eigen::Vector3f(0.6f,0.6f,0.6f))/map_->getResolution()).cast<int>();
    grid_min_ = Eigen::Vector3i::Zero();
    for(size_t x = 0;x < grid_max_.x();++x){
        for(size_t y = 0;y < grid_max_.y();++y){
            for(size_t z = 0;z < grid_max_.z();++z){
                size_t idx = map_i->get_idx(Eigen::Vector3i(x,y,z));

                position_buffer_[idx] = Eigen::Vector3i(x,y,z);
            }
        }
    }


    frontier_map_.resize(grid_max_.x()*grid_max_.y()*grid_max_.z(),0);
    grid_max_.z() -= 3; //avoid planner to go out of the map

}

int nnfrontiers_utils::validity_check(Eigen::Vector3i p, std::vector<uint8_t> &states, std::vector<float> &edt){
    size_t idx = map_->get_idx(p);
    std::cout << p.transpose()<<std::endl;

    if((states[idx] & VoxelBuffer::frontier_flag)){
        return 2;
    }

    if((states[idx]) & VoxelBuffer::occupied_flag || (states[idx]) & VoxelBuffer::unknown){
        std::cout << "Occupied cell or unknown"<<std::endl;
        return 0;
    }

    return 1;
}

bool nnfrontiers_utils::nn_frontier_djikstra(const Eigen::Vector3i &start, std::vector<uint8_t> &states, std::vector<bool> &target_frontier)
{
    std::cout << "Start djikstra: " << start.transpose() <<std::endl;



    std::vector<float> edt;
    map_->getEdtMap(edt);

    std::vector<int> parent(states.size(),-1);
    uint32_t N = 2000000;

    uint32_t rear = 0, front = 0;
    std::vector<size_t> indices(N);
    Eigen::Vector3i safe_start = start;
    size_t next_idx = map_->get_idx(start);
    if(edt[next_idx] < 0.5  || start.x() >= (grid_max_.x()-2)
            || start.y() >= (grid_max_.y()-2)
            || start.z() >= (grid_max_.z()-2)){
        //shift
        std::cout << "Start not in djikstra bound or to close of an obstacle (shift)" <<std::endl;
        size_t next_idx = shiftStart(start,edt,states,safe_start);
    }
    //custom queue function
    auto enqueue = [&indices,&rear](size_t idx){
        indices[rear] = idx;
        rear++;
    };


    auto dequeue= [&indices,&next_idx,&front](){
        next_idx = indices[front];
        front++;
    };


    auto valid_path = [this,&parent](int key){
        Eigen::Vector3f p_tmp;
        map_->convert(position_buffer_[key],p_tmp);
        Eigen::Vector3f start_p = p_tmp;

        while (key >= 0) {
            map_->convert(position_buffer_[key],p_tmp);
            if((start_p - p_tmp).norm() > radius_+0.01 && (start_p - p_tmp).norm() < 2.){
                float dot = (p_tmp-start_p).normalized().z();
                float angle = acos(dot);
                if(dot < 0){
                    angle = angle-M_PI_2;
                }else{
                    angle = M_PI_2-angle;
                }
                if(angle<fovy_rad_/2.){
                    if(map_->checkOcclusion(p_tmp,start_p))
                        return true;
                }

            }
            key = parent[key];
        }
        return false;
    };

    auto djik_validity_check = [this,&states,&edt,&next_idx,&parent,&enqueue,&target_frontier,&valid_path](const Eigen::Vector3i &p, int &res){
        size_t child_idx = map_->get_idx(p);

        if(parent[child_idx] != -1){
            res = 0;
            return;
        }
        res = 1;
        if((states[child_idx]) & VoxelBuffer::frontier_flag){
            parent[child_idx] = next_idx;
            if(valid_path(child_idx)){
                res= 2;
                next_idx = child_idx;
            }else{
                parent[child_idx] = -2;

                res = 0;
            }
            return;

        }
        //        if(target_frontier[child_idx]){
        //            res= 2;
        //            parent[child_idx] = next_idx;
        //            next_idx = child_idx;
        //            return;
        //        }
        if(edt[child_idx] <= radius_ ||(states[child_idx]) & VoxelBuffer::occupied_flag || (states[child_idx])==0){
            parent[child_idx] = -2;

            res = 0;
            return;
        }

        parent[child_idx] = next_idx;


        enqueue(child_idx);
    };

    enqueue(next_idx);
    parent[next_idx] = -3;//start
    int res;
    while(front < rear){
        //dequeue nearest cell
        dequeue();
        res = 0;
        Eigen::Vector3i cell = position_buffer_[next_idx];
        int incr_x = 1;
        int decr_x = 1;

        int incr_y = 1;
        int decr_y = 1;

        int incr_z = 1;
        int decr_z = 1;

        if(cell.x() ==0){
            decr_x = 0;
        }
        if(cell.y() ==0){
            decr_y = 0;
        }
        if(cell.z() ==0){
            decr_z = 0;
        }


        if(cell.x() >= grid_max_.x()-1){
            incr_x = 0;
        }

        if(cell.y() >= grid_max_.y()-1){
            incr_y = 0;
        }
        if(cell.z() >= grid_max_.z()-1){
            incr_z = 0;
        }

        {
            Eigen::Vector3i f = position_buffer_[next_idx];
            f(0) = position_buffer_[next_idx](0)-decr_x;
            djik_validity_check(f,res);
            if(res == 2)
                break;

            f(0) =position_buffer_[next_idx](0)+incr_x;
            djik_validity_check(f,res);
            if(res == 2)
                break;

        }
        {
            Eigen::Vector3i f = position_buffer_[next_idx];
            f(1) =position_buffer_[next_idx](1)-decr_y;
            djik_validity_check(f,res);
            if(res == 2)
                break;

            f(1) =position_buffer_[next_idx](1)+incr_y;
            djik_validity_check(f,res);
            if(res == 2)
                break;

        }
        {
            Eigen::Vector3i f = position_buffer_[next_idx];
            f(2) =  position_buffer_[next_idx](2)-decr_z;
            djik_validity_check(f,res);
            if(res == 2)
                break;

            f(2) = position_buffer_[next_idx](2)+incr_z;
            djik_validity_check(f,res);
            if(res == 2)
                break;
        }
        //extract face 6 neighbors
        //xy, xz, yz edge

        size_t i0 = 0,i1=1;
        {
            Eigen::Vector3i e = position_buffer_[next_idx];
            e(i0) -=decr_x,e(i1)-=decr_y;
            djik_validity_check(e,res);
            if(res == 2)
                break;

            e = position_buffer_[next_idx];
            e(i0) -=decr_x,e(i1)+=incr_y;
            djik_validity_check(e,res);
            if(res == 2)
                break;

            e = position_buffer_[next_idx];
            e(i0) +=incr_x,e(i1)-=decr_y;
            djik_validity_check(e,res);
            if(res == 2)
                break;

            e = position_buffer_[next_idx];

            e(i0) +=incr_x,e(i1)+=incr_y;
            djik_validity_check(e,res);
            if(res == 2)
                break;
        }
        {
            i0 = 0,i1=2;
            Eigen::Vector3i e = position_buffer_[next_idx];
            e(i0) -=decr_x,e(i1)-=decr_z;
            djik_validity_check(e,res);
            if(res == 2)
                break;

            e = position_buffer_[next_idx];
            e(i0) -=decr_x,e(i1)+=incr_z;
            djik_validity_check(e,res);
            if(res == 2)
                break;

            e = position_buffer_[next_idx];
            e(i0) +=incr_x,e(i1)-=decr_z;
            djik_validity_check(e,res);
            if(res == 2)
                break;

            e = position_buffer_[next_idx];
            e(i0) +=incr_x,e(i1)+=incr_z;
            djik_validity_check(e,res);
            if(res == 2)
                break;

        }


        {
            i0 = 1,i1=2;
            Eigen::Vector3i e = position_buffer_[next_idx];
            e(i0) -=decr_y,e(i1)-=decr_z;
            djik_validity_check(e,res);
            if(res == 2)
                break;
            e = position_buffer_[next_idx];
            e(i0) -=decr_y,e(i1)+=incr_z;
            djik_validity_check(e,res);
            if(res == 2)
                break;
            e = position_buffer_[next_idx];
            e(i0) +=incr_y,e(i1)-=decr_z;
            djik_validity_check(e,res);
            if(res == 2)
                break;
            e = position_buffer_[next_idx];

            e(i0) +=incr_y,e(i1)+=incr_z;
            djik_validity_check(e,res);
            if(res == 2)
                break;(e,states,edt);

        }


        //corner corner 8

        {
            Eigen::Vector3i c = position_buffer_[next_idx];
            i0 = 0,i1=1;
            size_t i2  = 2;
            c = position_buffer_[next_idx];

            c(i0) -=decr_x,c(i1)-=decr_y,c(i2)-=decr_z;
            djik_validity_check(c,res);
            if(res == 2)
                break;

            c = position_buffer_[next_idx];
            c(i0) +=incr_x,c(i1)-=decr_y,c(i2)-=decr_z;
            djik_validity_check(c,res);
            if(res == 2)
                break;

            c = position_buffer_[next_idx];
            c(i0) -=decr_x,c(i1)+=incr_y,c(i2)-=decr_z;
            djik_validity_check(c,res);
            if(res == 2)
                break;

            c = position_buffer_[next_idx];
            c(i0) +=incr_x,c(i1)+=incr_y,c(i2)-=decr_z;
            djik_validity_check(c,res);
            if(res == 2)
                break;

            c = position_buffer_[next_idx];
            c(i0) -=decr_x,c(i1)-=decr_y,c(i2)+=incr_z;
            djik_validity_check(c,res);
            if(res == 2)
                break;

            c = position_buffer_[next_idx];
            c(i0) +=incr_x,c(i1)-=decr_y,c(i2)+=incr_z;
            djik_validity_check(c,res);
            if(res == 2)
                break;

            c = position_buffer_[next_idx];
            c(i0) -=decr_x,c(i1)+=incr_y,c(i2)+=incr_z;
            djik_validity_check(c,res);
            if(res == 2)
                break;

            c = position_buffer_[next_idx];
            c(i0) +=incr_x,c(i1)+=incr_y,c(i2)+=incr_z;
            djik_validity_check(c,res);
            if(res == 2)
                break;


        }
    }
    if(res != 2){
        return false;
    }


    int next_key = next_idx;
    path_.clear();
    Eigen::Vector3f p_tmp;
    map_->convert(position_buffer_[next_key],p_tmp);
    Eigen::Vector3f start_p = p_tmp;
    std::cout << "Target djikstra: " << position_buffer_[next_key].transpose() <<std::endl;

    goal = p_tmp;
    bool insert_path = false;
    while (next_key >= 0) {
        map_->convert(position_buffer_[next_key],p_tmp);

        if((start_p - p_tmp).norm() > radius_+0.01){
            float dot = (start_p - p_tmp).normalized().z();
            float angle = acos(dot);
            if(dot < 0){
                angle = angle-M_PI_2;
            }else{
                angle = M_PI_2-angle;
            }
            if(angle<fovy_rad_*0.3){
                insert_path = true;
            }
            if(insert_path)
                path_.push_back(p_tmp);
        }
        //        std::cout <<position_buffer_[next_key].transpose() <<std::endl;
        next_key = parent[next_key];
    }
    //revert path start to end
    std::reverse(path_.begin(), path_.end());
    convert_simplify_path();
    //    for(int i = 0; i < simplified_path_.size();++i){
    //        std::cout <<simplified_path_[i].transpose() <<std::endl;
    //    }

    std::cout << "djikstra: "<< res<< std::endl;

    return true;
}



bool nnfrontiers_utils::djiktra_priority_queue(const Eigen::Vector3i &start, std::vector<uint8_t> &states, std::vector<bool> &target_frontier)
{



    std::priority_queue< std::pair<float,int>, std::vector <std::pair<float,int>>,auto(*)(const std::pair<float,int>&,const std::pair<float,int>&)->bool > pq{
        [](const std::pair<float,int> &lhs,const std::pair<float,int>& rhs)->bool { return lhs.first >= rhs.first; }
    };

    std::vector<float> edt;
    map_->getEdtMap(edt);

    std::vector<std::pair<float,int>> parents(states.size(),std::pair<float,int>(std::numeric_limits<float>::max(),-1));
    uint32_t N = 2000000;

    std::vector<int> indices(N);
    Eigen::Vector3i safe_start = start;
    int next_idx = map_->get_idx(start);

    auto isInside = [this](const Eigen::Vector3i &pt){

        return  pt.x() >= grid_min_.x() &&
                pt.x() < grid_max_.x() &&
                pt.y() >= grid_min_.y() &&
                pt.y() < grid_max_.y() &&
                pt.z() >= grid_min_.z() &&
                pt.z() < grid_max_.z();
    };
    if(edt[next_idx] < 0.5  || !isInside(start)){
        //shift
        std::cout << "Start not in djikstra bound or to close of an obstacle (shift)" <<std::endl;
        int next_idx = shiftStart(start,edt,states,safe_start);
    }


    pq.push(std::make_pair(0.f, next_idx));
    parents[next_idx].first = 0.f;
    parents[next_idx].second = -1;

    auto valid_path = [this,&parents](int key){
        Eigen::Vector3f p_tmp;
        map_->convert(position_buffer_[key],p_tmp);
        Eigen::Vector3f start_p = p_tmp;
        int count = 0;
        while (key >= 0) {
            map_->convert(position_buffer_[key],p_tmp);
            if((start_p - p_tmp).norm() > radius_ && (start_p - p_tmp).norm() < 2.){
                float dot = (p_tmp-start_p).normalized().z();
                float angle = acos(dot);
                if(dot < 0){
                    angle = angle-M_PI_2;
                }else{
                    angle = M_PI_2-angle;
                }
                if(angle<fovy_rad_*0.3){
                    if(map_->checkOcclusion(p_tmp,start_p))
                        return true;
                }

            }
            //            if (count++ > (int)(2.5/map_->getResolution()))
            //                return true;
            key = parents[key].second;
        }
        return false;
    };

    while (!pq.empty())
    {
        next_idx = pq.top().second;
        pq.pop();
        Eigen::Vector3i cell = position_buffer_[next_idx];

        for(int x_step = -1; x_step <= 1;++x_step){
            for(int y_step = -1; y_step <= 1;++y_step){
                for(int z_step = -1; z_step <= 1;++z_step){
                    if(x_step==0 && y_step== 0 && z_step==0){
                        continue;
                    }
                    Eigen::Vector3i step(x_step,y_step,z_step);
                    Eigen::Vector3i adj_cell = cell+step;
                    //if inside grid maybe use djikstra bound?...
                    if(isInside(adj_cell)){

                        int adj_idx = map_->get_idx(adj_cell);
                        float obstacle_repulsion = std::fmax(std::fmin(2.f,edt[adj_idx]),0.1);
                        obstacle_repulsion = 1.f/(obstacle_repulsion)-0.5f;
                        obstacle_repulsion *=obstacle_repulsion;
                        obstacle_repulsion*=2;
//                        obstacle_repulsion /=4.f;
                        float weight = (step.cast<float>()*map_->getResolution()).norm()+ obstacle_repulsion;
                        // If there is shorted path to v through u.
                        if (parents[adj_idx].first > parents[next_idx].first + weight)
                        {
                            if((states[adj_idx]) & VoxelBuffer::occupied_flag || (states[adj_idx])==0){
                                continue;
                            }
                            //                            else if((states[adj_idx]) & VoxelBuffer::frontier_flag){
                            else if(target_frontier[adj_idx]){
                                parents[adj_idx].second = next_idx;

                                if(valid_path(adj_idx) /*&& parents[next_idx].second + weight > 2*/){
                                    parents[adj_idx].first = parents[next_idx].first + weight;
                                    next_idx = adj_idx;
                                    goto found_frontier;
                                }else{
                                    parents[adj_idx].second = -1;

                                }
                            }else if(states[adj_idx] & VoxelBuffer::frontier_flag)
                            {
                                continue;
                            }else{

                                parents[adj_idx].second = next_idx;
                                parents[adj_idx].first = parents[next_idx].first + weight;
                                pq.push(std::make_pair(parents[adj_idx].first,adj_idx));
                            }

                        }
                    }
                }
            }
        }

    }

    std::cout << "[DJISTRA] No valid path found\n";
    //remove all cluster center and iterate until target_frontier is empty
    return false;

found_frontier:

    int next_key = next_idx;
    path_.clear();
    Eigen::Vector3f p_tmp;
    map_->convert(position_buffer_[next_key],p_tmp);
    Eigen::Vector3f start_p = p_tmp;
    frontier_map_[next_key]  = 2;

    goal = p_tmp;
    bool insert_path = false;
    while (next_key >= 0) {
        map_->convert(position_buffer_[next_key],p_tmp);
        //        path_.push_back(p_tmp);

        if((start_p - p_tmp).norm() > radius_){
            float dot = (start_p - p_tmp).normalized().z();
            float angle = acos(dot);
            if(dot < 0){
                angle = angle-M_PI_2;
            }else{
                angle = M_PI_2-angle;
            }
            if(angle<fovy_rad_/2.){
                insert_path = true;
            }
            if(insert_path)
                path_.push_back(p_tmp);
        }
        std::cout <<position_buffer_[next_key].transpose() <<std::endl;
        next_key = parents[next_key].second;
    }

    std::reverse(path_.begin(), path_.end());

    std::cout << "Target djikstra pq: " << path_[0].transpose() << " to " << path_.back().transpose() <<std::endl;
    //    path_.resize(path_.size()-1);// remove last element
    convert_simplify_path();


    return true;

}

std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > nnfrontiers_utils::getSimplifiedPath()
{
    return simplified_path_;
}

std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > nnfrontiers_utils::getPath()
{
    return path_;

}

void nnfrontiers_utils::convert_simplify_path()
{
    simplified_path_.clear();
    if(path_.size()<2){
        return;
    }
    if(path_.size()==2){
        simplified_path_ = path_;
        return;
    }
    int start=0;
    int end=1;
    simplified_path_ = path_;

//    simplified_path_.push_back(path_[start]);
//    for(int i = 2; i < path_.size();++i){
//        //        std::cout <<path_[start].transpose() <<std::endl;
//        //        std::cout <<path_[i-1].transpose() <<std::endl;
//        //        std::cout <<path_[i].transpose() <<std::endl;
//        //        std::cout << line_pts_angle(path_[start],path_[i-1],path_[i]) <<std::endl;
//        if(line_pts_angle(path_[start],path_[i-1],path_[i])<0.9){
//            simplified_path_.push_back(path_[i-1]);
//            start = i-1;
//        }
//    }
//    simplified_path_.push_back(path_[path_.size()-1]);

    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > tmp;
    tmp.push_back(simplified_path_[0]);
    int i=1;
    for(;i < simplified_path_.size();++i){
        //cast to the farest reachable position:
        const Eigen::Vector3f &p1 = tmp[tmp.size()-1];
        int j = i;
        while(j < simplified_path_.size() ){
            const Eigen::Vector3f &p2 = simplified_path_[j];

            Eigen::Vector3f dir = (p2-p1);
            float len = dir.norm();
            dir.normalize();
            float incr = 0.02f;
            float s = incr;
            bool found = true;
//            std::cout << p1.transpose() << " " << p2.transpose() << std::endl;
//            std::cout << i << " " << j << std::endl;
            while (s < len) {
                float d;
                Eigen::Vector3f p = p1 + dir*s;
                if(!map_->is_inside(p) || (map_->getState(p) & VoxelBuffer::visited_flag) == 0){
                    found=false;
                    break;
                }
                map_->getDistance(p,d);
                if(d<radius_){
//                    std::cout << "out" <<std::endl;
                    found=false;
                    break;
                }
                s+=incr;
            }
            if(!found || j == simplified_path_.size()-1){
                if(j==i){
                    tmp.push_back(simplified_path_[j]);
                }else{
                    tmp.push_back(simplified_path_[j-1]);

                }
                break;
            }
            ++j;
        }
        i=j;
    }
//    std::cout << "SIZE " << simplified_path_.size() <<" " << tmp.size() <<std::endl;

//    std::cout <<std::endl;

    simplified_path_.clear();
    simplified_path_ = tmp;
}

float nnfrontiers_utils::line_pts_angle(Eigen::Vector3f p1, Eigen::Vector3f p2, Eigen::Vector3f x)
{
    Eigen::Vector3f dir1 = (p2-p1).normalized();
    Eigen::Vector3f dir2 = (x-p2).normalized();
    return fabs(dir1.dot(dir2));
}

Eigen::Vector3f nnfrontiers_utils::getGoal() const
{
    return goal;
}

size_t nnfrontiers_utils::shiftStart(const Eigen::Vector3i &start, std::vector<float> &edt,std::vector<uint8_t> &states,Eigen::Vector3i &shiftedStart)
{

    Eigen::Vector3i step(3,3,3);
    float dist = 9999999;
    size_t idx = map_->get_idx(start);
    shiftedStart = start;

    for(int i = 0; i < 3 ; ++i){
        if(shiftedStart[i]<grid_min_[i])
            shiftedStart[i]=grid_min_[i]+1;

        if(shiftedStart[i]>=grid_max_[i])
            shiftedStart[i]=grid_max_[i]-2;
    }
    Eigen::Vector3i cpy_start = shiftedStart;
    auto isInside = [this](Eigen::Vector3i &pt){

        return  pt.x() >= grid_min_.x() &&
                pt.x() < grid_max_.x() &&
                pt.y() >= grid_min_.y() &&
                pt.y() < grid_max_.y() &&
                pt.z() >= grid_min_.z() &&
                pt.z() < grid_max_.z();
    };
    for(int x = -step.x(); x<=step.x(); ++x)   {
        for(int y = -step.y(); y<=step.y(); ++y)   {
            for(int z = -step.z(); z<=step.z(); ++z)   {
                if(x==0&&y == 0 && z==0)
                    continue;

                Eigen::Vector3i tmp = cpy_start+step;
                size_t tmpidx = map_->get_idx(cpy_start+step);

                if(!isInside(tmp))
                    continue;

                if(edt[tmpidx]>radius_ &&  !((states[tmpidx]) & VoxelBuffer::occupied_flag) && !((states[tmpidx]) & VoxelBuffer::frontier_flag) && (states[tmpidx])!=0){
                    float tmpdist = (tmp-cpy_start).squaredNorm();
                    if(tmpdist<dist){
                        shiftedStart = tmp;
                        dist=tmpdist;
                        idx = tmpidx;
                    }
                }
            }
        }
    }

    return idx;

}

void nnfrontiers_utils::updateFrontierMap(const std::vector<uint8_t> &map_states)
{

    for(size_t i = 0; i < map_states.size(); ++i){
        if(map_states[i] & VoxelBuffer::frontier_flag){
            if(frontier_map_[i] == 0){
                frontier_map_[i]  = 1;
            }
        }else if(map_states[i] != 0){
            frontier_map_[i]  = 2;
        }

    }
}



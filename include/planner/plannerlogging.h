#include <vector>
#include <string>

#include <fstream>
#include "dirent.h"
#include <iostream>
#include "map_core/../../utils/timer.hpp"
#include <Eigen/Core>
enum LogAction{
    Concat,
    NewLine
};

class PlannerLogging
{
public:
    PlannerLogging()
    {
        speed_.reserve(10000);
        computation_time_.reserve(10000);
        planned_distance_.reserve(10000);

        dist_and_speed_ts.reserve(10000);


    }


    void addSpeed(double s){
        if(dist_and_speed_ts.empty()){
            log_ts.restart();
            dist_and_speed_ts.push_back(0);
        }else{
            dist_and_speed_ts.push_back(log_ts.elapsed_ms());
        }

        if(speed_log_timer_.elapsed_ms()> 250){
            speed_.push_back(s);
            speed_log_timer_.restart();
        }
    }

    void addTime(double t){
        computation_time_.push_back(t);
    }

    void log_dist_speed(double s, const Eigen::Vector3d &pos){
        if(dist_and_speed_ts.empty()){
            log_ts.restart();
            dist_and_speed_ts.push_back(0);
            planned_distance_.push_back(0);
            speed_.push_back(0);


        }else{
            if(dist_log_timer_.elapsed_ms()> 100){
                dist_and_speed_ts.push_back(log_ts.elapsed_ms());
                planned_distance_.push_back(planned_distance_.back()+save_dist+(last_pos_-pos).norm());
                speed_.push_back(s);
                dist_log_timer_.restart();
                save_dist=0;
            }else{
                save_dist += (last_pos_-pos).norm();
            }
        }
        last_pos_ = pos;


    }
    void addDistance(const Eigen::Vector3d &pos){
        if(dist_log_timer_.elapsed_ms()> 250){
            if(planned_distance_.empty()){
                planned_distance_.push_back(0);
            }else{
                planned_distance_.push_back(planned_distance_.back()+(last_pos_-pos).norm());
            }
            last_pos_ = pos;
            dist_log_timer_.restart();
        }

    }




    void addLine(std::ofstream &outdata, std::vector<double> &vec){
        if(vec.empty()){
            return;
        }
        int j = 0;
        for (; j<vec.size()-1; ++j){
            outdata << vec[j] << " ";
        }
        outdata << vec[j];
    }
    void saveData(std::string &path){
        std::ofstream outdata; // outdata is like cin
        DIR *dp;
        int i = 0;
        struct dirent *ep;

        dp = opendir (path.c_str());

        if (dp != NULL)
        {
            while (ep = readdir (dp))
                i++;

            (void) closedir (dp);
        }
        else
            perror ("Couldn't open the directory");

        i-=1;

        std::string file = path;
        file+= "/result_"+std::to_string(i)+".txt";

        std::cout <<"Exploration log to file " << file<<std::endl;
        outdata.open(file); // opens the file
        if( !outdata ) { // file couldn't be opened
            std::cerr << "Error: file could not be opened" << std::endl;
            return;
        }


        addLine(outdata,dist_and_speed_ts);
        outdata << std::endl;
        addLine(outdata,speed_);
        outdata << std::endl;
        addLine(outdata,planned_distance_);
        outdata << std::endl;
        addLine(outdata,computation_time_);


        outdata.close();
    }
    std::vector<double> dist_and_speed_ts;
    std::vector<double> speed_;
    std::vector<double> planned_distance_;
    std::vector<double> computation_time_;
    std::string save_path_ = "";

private:

    Timer log_ts;

    Timer speed_log_timer_;
    Timer dist_log_timer_;
    Eigen::Vector3d last_pos_;
    double save_dist = 0;
};


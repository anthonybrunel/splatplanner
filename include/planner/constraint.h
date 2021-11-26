#ifndef CONSTRAINT_H
#define CONSTRAINT_H


class Constraint
{
public:
    Constraint();

    double max_v_; // m/s
    double max_a_; // m/s^2
    double max_ang_v_;
    double max_ang_a_;

    double max_yaw_vel_;
    double max_yaw_acc_;



    double margin_ = 0.3;
};

#endif // CONSTRAINT_H

#pragma once


enum PlannerState{
    INIT,
    TAKEOFF,
    YAWTURN,
    TRAJ,
    IDLE,
    LANDING,
    STOP,
    ERROR
};

enum TrajectoryState{
    RUN,
    BRAKING,
    WAITING
};

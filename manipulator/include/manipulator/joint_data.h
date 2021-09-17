#pragma once

#include <cstdio>
#include <iostream>
#include <string>

typedef struct
{
public:
    int id_;
    std::string joint_name_;
    double joint_angle_;
    double min_angle_;
    double max_angle_;
    double max_velocity_;
    double velocity_;
    double acceleration_;
    double deceleration_;
    double angle_cmd_;
    double velocity_cmd_;
    double effort_;
    double home_offset_;
} JointData;
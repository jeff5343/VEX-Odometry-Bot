#ifndef __ODOMETRY_H_INCLUDED__
#define __ODOMETRY_H_INCLUDED__

#include "vex.h"
#include "pose.h"

class Odometry
{
private:
    const double WHEEL_RADIUS_INCHES = 0.0;
    const double DIST_CENTER_TO_RIGHT_WHEEL = 0.0;
    const double DIST_CENTER_TO_LEFT_WHEEL = 0.0;
    const double DIST_CENTER_TO_BOT_WHEEL = 0.0;

    vex::encoder &leftEncoder;
    vex::encoder &rightEncoder;
    vex::encoder &backEncoder;

    double leftDistInches;
    double rightDistInches;
    double backDistInches;

    double deltaLeftDistInches;
    double deltaRightDistInches;
    double deltaBackDistInches;

    Pose pose;

    void updateEncoderDistances();

public:
    Odometry(vex::encoder &leftEncoder, vex::encoder &rightEncoder, vex::encoder &backEncoder)
        : leftEncoder(leftEncoder), rightEncoder(rightEncoder), backEncoder(backEncoder) {};

    /* updates pose, should be called every 20ms */
    void updateOdometry();

    /* returns calculated pose */
    Pose getPose() const { return pose; }
};

#endif
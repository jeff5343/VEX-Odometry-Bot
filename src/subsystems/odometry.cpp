#ifdef __TESTING__
#include "../../include/subsystems/odometry.h"
#include "../../include/util/angle.h"
#else
#include "subsystems/odometry.h"
#include "util/angle.h"
#endif

void Odometry::updateEncoderDistances()
{
    double wheelCircumference = 2 * M_PI * WHEEL_RADIUS_INCHES;

    double leftD = leftEncoder.position(vex::rev) * wheelCircumference;
    double rightD = rightEncoder.position(vex::rev) * wheelCircumference;
    double backD = backEncoder.position(vex::rev) * wheelCircumference;

    setNewEncoderDistances(leftD, rightD, backD);
}

void Odometry::setNewEncoderDistances(double leftD, double rightD, double backD)
{
    // update deltas
    dLeftDist = leftD - leftDist;
    dRightDist = rightD - rightDist;
    dBackDist = backD - backDist;

    // update distances of encoders
    leftDist = leftD;
    rightDist = rightD;
    backDist = backD;
}

void Odometry::updatePose()
{
    // find delta heading
    double deltaTheta = (dLeftDist - dRightDist) /
                        (DIST_CENTER_TO_RIGHT_WHEEL + DIST_CENTER_TO_LEFT_WHEEL);

    double correctedDBackDist = dBackDist -
                       // this subtracts the back wheel's distance when moving
                       ((deltaTheta / (2 * M_PI)) * (2 * DIST_CENTER_TO_BOT_WHEEL * M_PI));

    // printf("back delta: %.2f\n", correctedDBackDist);
    // printf("delta theta: %.2f\n", deltaTheta);

    // calculate relative distance traveled
    double relXDist, relYDist;
    if (dLeftDist == dRightDist)
    {
        relXDist = correctedDBackDist;
        relYDist = dLeftDist;
    }
    else
    {
        double cof = 2.0 * sin(deltaTheta / 2.0);
        relXDist = cof * ((correctedDBackDist / deltaTheta) +
                          DIST_CENTER_TO_BOT_WHEEL);
        relYDist = cof * ((dRightDist / deltaTheta) +
                          DIST_CENTER_TO_RIGHT_WHEEL);
    }

    // convert relative vector into global vector
    double relativeMagnitude = sqrt((relXDist * relXDist) + (relYDist * relYDist));
    double relativeHeading = atan2(relYDist, relXDist);

    relativeHeading -= (pose.radians + (deltaTheta / 2.0));

    double dX = cos(relativeHeading) * relativeMagnitude;
    double dY = sin(relativeHeading) * relativeMagnitude;

    // check for invalid values
    if (std::isnan(dX))
        dX = 0;
    if (std::isnan(dY))
        dY = 0;
    if (std::isnan(deltaTheta))
        deltaTheta = 0;

    // update pose object
    pose.x += dX;
    pose.y += dY;
    pose.radians += deltaTheta;
    // wrap radians
    pose.radians = Angle::wrapRadians(pose.radians);
}

void Odometry::update()
{
    updateEncoderDistances();
    updatePose();
}
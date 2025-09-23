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

    // (used for debugging)
    // the distance the back wheel should have traveled if there was no drift
    // double backWheelNormalTravelDistance = (-deltaTheta * DIST_CENTER_TO_BOT_WHEEL);
    // printf(" backWheelDrift: %.3f\n", dBackDist - backWheelNormalTravelDistance);

    // calculate relative distance traveled
    //  Y axis is relative side to side movement
    //  X axis is relative front and back movement
    double relYDist, relXDist;
    if (dLeftDist == dRightDist)
    {
        // heading has not changed
        relYDist = dBackDist;
        relXDist = dLeftDist;
    }
    else
    {
        // heading has changed, formulas to find distance
        // traveled using the arcs traveled by the encoders
        double cof = 2.0 * sin(deltaTheta / 2.0);
        relYDist = cof * ((dBackDist / deltaTheta) +
                          (DIST_CENTER_TO_BOT_WHEEL));
        relXDist = cof * ((dRightDist / deltaTheta) +
                          DIST_CENTER_TO_RIGHT_WHEEL);
    }

    // convert relative vector into global vector
    double relativeMagnitude = sqrt((relYDist * relYDist) + (relXDist * relXDist));
    double relativeHeading = atan2(relXDist, relYDist);

    // subtract (current heading of robot + the change in heading)
    relativeHeading -= (pose.radians + (deltaTheta / 2.0));

    double dY = cos(relativeHeading) * relativeMagnitude;
    double dX = sin(relativeHeading) * relativeMagnitude;

    // check for invalid values
    if (std::isnan(dY))
        dY = 0;
    if (std::isnan(dX))
        dX = 0;
    if (std::isnan(deltaTheta))
        deltaTheta = 0;

    // update pose object
    pose.x += dX;
    pose.y += dY;
    // subtracting instead of adding so CCW+ and CW-
    pose.radians -= deltaTheta;
    // wrap radians
    pose.radians = Angle::wrapRadians(pose.radians);
}

void Odometry::update()
{
    updateEncoderDistances();
    updatePose();
}
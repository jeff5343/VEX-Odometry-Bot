#include "odometry.h"

void Odometry::updateEncoderDistances()
{
    double wheelCircumference = 2 * M_PI * WHEEL_RADIUS_INCHES;

    double leftD = leftEncoder.position(vex::rev) * wheelCircumference;
    double rightD = rightEncoder.position(vex::rev) * wheelCircumference;
    double backD = backEncoder.position(vex::rev) * wheelCircumference;

    // update distances of encoders
    deltaLeftDistInches = leftD - leftDistInches;
    deltaRightDistInches = rightD - rightDistInches;
    deltaBackDistInches = backD - backDistInches;

    // update deltas
    leftDistInches = leftD;
    rightDistInches = rightD;
    backDistInches = backD;
}

void Odometry::updateOdometry()
{
    updateEncoderDistances();

    // find heading
    double newHeading = (deltaLeftDistInches - deltaRightDistInches) /
                     (DIST_CENTER_TO_RIGHT_WHEEL + DIST_CENTER_TO_LEFT_WHEEL);
    double deltaHeading = newHeading - pose.radians;

    // calculate relative distance traveled
    double relativeXDist, relativeYDist;
    if (deltaHeading == 0)
    {
        relativeXDist = deltaBackDistInches;
        relativeYDist = deltaLeftDistInches;
    }
    else
    {
        double cof = 2.0 * sin(deltaHeading / 2.0);
        relativeXDist = cof * ((deltaBackDistInches / deltaHeading) -
                               DIST_CENTER_TO_BOT_WHEEL);
        relativeYDist = cof * ((deltaLeftDistInches / deltaHeading) -
                               DIST_CENTER_TO_LEFT_WHEEL);
    }

    // convert relative vector into global vector
    double relativeMagnitude = sqrt(relativeXDist * relativeXDist + relativeYDist * relativeYDist);
    double relativeHeading = atan2(relativeYDist, relativeXDist);

    relativeHeading += pose.radians + (deltaHeading / 2.0);

    // update pose object
    pose.radians = newHeading;
    pose.x = cos(relativeHeading) * relativeMagnitude;
    pose.y = sin(relativeHeading) * relativeMagnitude;
}
#include "robot.h"
#include <cmath>

void Robot::init() {

};

void Robot::usercontrolPeriodic()
{
    /* TELEOP DRIVING: */

    // convert axis positions to range -1.0 to 1.0
    double x = static_cast<double>(controller.Axis1.position()) / 100.0;
    double y = static_cast<double>(controller.Axis3.position()) / 100.0;

    double deadband = 0.01;
    if (std::fabs(x) <= deadband)
    {
        x = 0;
    }
    if (std::fabs(y) <= deadband)
    {
        y = 0;
    }
    if (std::fabs(x) > deadband || std::fabs(y) > deadband)
    {
        drivetrain.setVelocityLeftMotors(50);
        // drivetrain.arcadeDrive(x, y);
    }
    else
    {
        drivetrain.stop();
    }

    /* ODOMETRY TESTING: */
    // drivetrain.log();
}
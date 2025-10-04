#include "robot.h"
#include <cmath>

const vex::brain Robot::BRAIN = vex::brain();

void Robot::init() {

};

void Robot::usercontrolPeriodic()
{
    // convert axis positions to range -1.0 to 1.0
    double x = static_cast<double>(controller.Axis1.position()) / 100.0;
    double y = static_cast<double>(controller.Axis3.position()) / 100.0;

    double deadband = 0.01;

    if (std::abs(x) <= deadband)
        x = 0;
    if (std::abs(y) <= deadband)
        y = 0;

    if (std::abs(x) > deadband || std::abs(y) > deadband)
    {
        drivetrain.arcadeDrive(x, y);
    }
    else
    {
        drivetrain.stop();
    }
}

void Robot::autonomousPeriodic()
{
    if (!driveSetpointSet)
    {
        pidDrive.setTargetPose(Pose{1.0, 0.0, 0.0});
        driveSetpointSet = true;
    }
    pidDrive.update();
}
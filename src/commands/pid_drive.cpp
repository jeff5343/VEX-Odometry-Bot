#include "commands/pid_drive.h"

void PidDrive::setTargetPose(Pose pose)
{
    target = pose;
    anglePid.setSetpoint(pose.radians);
}

void PidDrive::update()
{
    Pose current = drivetrain.getPose();

    if (!anglePid.isAtSetpoint())
    {
        double turnOutput = anglePid.calculate(current.radians);
        drivetrain.setPercentOut(-turnOutput, turnOutput);
        return;
    }

    double errorX = current.x - target.x;
    double errorY = current.y - target.y;
    double errorDist = sqrt(errorX * errorX + errorY * errorY);

    double straightOutput = anglePid.calculate(errorDist);
    drivetrain.setPercentOut(straightOutput, straightOutput);
}

bool PidDrive::isAtSetpoint()
{
    return anglePid.isAtSetpoint() && straightPid.isAtSetpoint();
}
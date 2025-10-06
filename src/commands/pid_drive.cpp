#include "commands/pid_drive.h"

void PidDrive::setTargetPose(Pose pose)
{
    target = pose;
    straightPid.setSetpoint(0);
    anglePid.setSetpoint(0);
}

void PidDrive::update()
{
    Pose current = drivetrain.getPose();

    double errorX = current.x - target.x;
    double errorY = current.y - target.y;
    double errorDist = sqrt(errorX * errorX + errorY * errorY);
    double errorAngle = atan2(errorY, errorX);

    // PID based on error to target (setpoints are 0)
    double straightOutput = -straightPid.calculate(errorDist);
    double turnOutput = anglePid.calculate(errorAngle);

    // moving to pose
    if (!straightPid.isAtSetpoint())
    {
        // first point to pose
        if (!anglePid.isAtSetpoint())
        {
            drivetrain.setPercentOut(-turnOutput, turnOutput);
            return;
        }
        // drive to pose
        drivetrain.setPercentOut(straightOutput, straightOutput);
    }

    // TODO: add rotating to pose defined angle at the end!
    // if (anglePid.isAtSetpoint()) {
    //     anglePid.setSetpoint(target.radians);
    // }
}

bool PidDrive::isAtSetpoint()
{
    return anglePid.isAtSetpoint() && straightPid.isAtSetpoint();
}
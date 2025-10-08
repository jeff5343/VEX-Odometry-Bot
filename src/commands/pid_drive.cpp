#include "commands/pid_drive.h"
#include "util/math_util.h"

void PidDrive::setTargetPose(Pose pose)
{
    target = pose;
    straightPid.setSetpoint(0);
    anglePid.setSetpoint(0);
}

void PidDrive::update()
{
    Pose current = drivetrain.getPose();

    // calculate error to target pose
    double errorX = current.x - target.x;
    double errorY = current.y - target.y;
    double errorDist = sqrt(errorX * errorX + errorY * errorY);
    double errorAngle = atan2(errorY, errorX);

    // PID based on error to target (setpoints are 0)
    // TODO: clamp pid values
    double straightPidOut = -MathUtil::clamp(straightPid.calculate(errorDist), -1.0, 1.0);
    double turnPidOut = MathUtil::clamp(anglePid.calculate(errorAngle), -1.0, 1.0);

    // output values to left and right wheels
    double leftOut = 0, rightOut = 0;

    // moving to pose
    if (!straightPid.isAtSetpoint())
    {
        // first point to pose
        if (!anglePid.isAtSetpoint())
        {
            leftOut = -turnPidOut;
            rightOut = turnPidOut;
        }
        // drive to pose
        else
        {
            leftOut = straightPidOut;
            rightOut = straightPidOut;
        }
    }

    drivetrain.setPercentOut(leftOut, rightOut);

    // TODO: add rotating to pose defined angle at the end!
    // if (anglePid.isAtSetpoint()) {
    //     anglePid.setSetpoint(target.radians);
    // }
}

bool PidDrive::isAtSetpoint()
{
    return anglePid.isAtSetpoint() && straightPid.isAtSetpoint();
}
#ifndef __PID_DRIVE_H_INCLUDED__
#define __PID_DRIVE_H_INCLUDED__

#include "pid_constants.h"
#include "drivetrain.h"
#include "pid.h"

class PidDrive
{
private:
    Pid anglePid;
    Pid straightPid;
    Pose target;

    Drivetrain &drivetrain;

public:
    PidDrive(PidConstants anglePidConstants, PidConstants straightPidConstants, Drivetrain &drivetrain)
        : anglePid(anglePidConstants), straightPid(anglePidConstants), drivetrain(drivetrain) {};

    void setTargetPose(Pose pose)
    {
        target = pose;
        anglePid.setSetpoint(pose.radians);
    }

    void update()
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

    bool isAtSetpoint()
    {
        return anglePid.isAtSetpoint() && straightPid.isAtSetpoint();
    }
};

#endif
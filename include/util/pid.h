#ifndef __PID_H_INCLUDED__
#define __PID_H_INCLUDED__

#include "pid_constants.h"
#include <cmath>

/* very crude implementation, can be a lot more sophisticated */
class Pid
{
private:
    /* CONSTANTS */
    PidConstants constants;
    // in seconds
    const double deltaTime = 0.02;
    const double setpointTolerance = 0.05;

    double setpoint;
    double prevError;
    double totalError;

public:
    // TODO: make sure main loop runs at 20ms
    Pid(PidConstants constants)
        : Pid(constants, setpointTolerance) {};

    Pid(PidConstants constants, double setpointTolerance)
        : Pid(constants, deltaTime, setpointTolerance) {};

    Pid(PidConstants constants, double deltaTime, double setpointTolerance)
        : constants(constants), deltaTime(deltaTime), setpointTolerance(setpointTolerance) {};

    void setSetpoint(double setpoint)
    {
        reset();
        this->setpoint = setpoint;
    }

    double getSetpoint()
    {
        return setpoint;
    }

    double getError()
    {
        return prevError;
    }

    bool isAtSetpoint()
    {
        return std::abs(prevError) < setpointTolerance;
    }

    // TODO: add field for 360 deg setpoints
    double calculate(double measurement)
    {
        double error = setpoint - measurement;

        totalError += error * deltaTime;

        double errorDerivative = (error - prevError) / deltaTime;

        prevError = error;

        return (constants.kP * error) + (constants.kI * totalError) + (constants.kD * errorDerivative);
    }

    void reset()
    {
        setpoint = 0;
        prevError = 0;
        totalError = 0;
    }
};

#endif
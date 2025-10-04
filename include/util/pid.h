#ifndef __PID_H_INCLUDED__
#define __PID_H_INCLUDED__

/* very crude implementation, can be a lot more sophisticated */
class PID
{
private:
    /* CONSTANTS */
    double kP;
    double kI;
    double kD;
    // in seconds
    const double deltaTime;

    double setpoint;
    double prevError;
    double totalError;

public:
    // TODO: make sure main loop runs at 20ms
    PID(double kP, double kI, double kD)
        : PID(kP, kI, kD, 0.02) {};

    PID(double kP, double kI, double kD, double deltaTime)
        : kP(kP), kI(kI), kD(kD), deltaTime(deltaTime) {};

    void setSetpoint(double setpoint)
    {
        reset();
        this->setpoint = setpoint;
    }

    double getSetpoint()
    {
        return setpoint;
    }

    double getError() {
        return prevError;
    }

    double calculate(double measurement)
    {
        double error = setpoint - measurement;

        totalError += error * deltaTime;

        double errorDerivative = (error - prevError) / deltaTime;

        return (kP * error) + (kI * totalError) - (kD * errorDerivative);
    }

    void reset() {
        setpoint = 0;
        prevError = 0;
        totalError = 0;
    }
};

#endif

#ifndef __DRIVETRAIN_H_INCLUDED__
#define __DRIVETRAIN_H_INCLUDED__

#include "vex.h"
#include "odometry.h"

class Drivetrain
{
private:
    // 36:1 (100 rpm), 18:1 (200 rpm), 6:1 (600 rpm)
    static constexpr double MAX_RPM = 600;
    static constexpr bool IS_BRAKE_MODE = true;

    // odometry
    Odometry odometry{};

    void setVelocityLeftMotors(double rpm);
    void setVelocityRightMotors(double rpm);

public:
    Drivetrain();

    /* Stops all motors */
    void stop();

    /**
     * Controls the drivetrain with arcade drive.
     * Uses two joystick axes for driving forward/backwards with rotation.
     * @param x value of x axis of joystick
     * @param y value of y axis of joystick
     */
    void arcadeDrive(double x, double y);

    /**
     * Log stuffs for drivetrain.
     * Should be called every loop, probably need better structure.
     */
    void log();
};

#endif
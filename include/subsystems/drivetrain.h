
#ifndef __DRIVETRAIN_H_INCLUDED__
#define __DRIVETRAIN_H_INCLUDED__

#include "vex.h"
#include "ports.h"
#include "odometry.h"

class Drivetrain
{
private:
    // 36:1 (100 rpm), 18:1 (200 rpm), 6:1 (600 rpm)
    static constexpr double MAX_RPM = 400;
    static constexpr bool IS_BRAKE_MODE = false;
    static constexpr bool LEFT_MOTORS_INVERTED = true;
    static constexpr bool RIGHT_MOTORS_INVERTED = false;

    // TODO: rename motors, mid motors are the motors that are elevated
    // motors
    vex::motor topLeftMotor{Ports::DRIVE_TOP_LEFT_MOTOR, vex::ratio6_1, LEFT_MOTORS_INVERTED};
    vex::motor midLeftMotor{Ports::DRIVE_MID_LEFT_MOTOR, vex::ratio6_1, !LEFT_MOTORS_INVERTED};
    vex::motor botLeftMotor{Ports::DRIVE_BOT_LEFT_MOTOR, vex::ratio6_1, LEFT_MOTORS_INVERTED};
    vex::motor topRightMotor{Ports::DRIVE_TOP_RIGHT_MOTOR, vex::ratio6_1, RIGHT_MOTORS_INVERTED};
    vex::motor midRightMotor{Ports::DRIVE_MID_RIGHT_MOTOR, vex::ratio6_1, !RIGHT_MOTORS_INVERTED};
    vex::motor botRightMotor{Ports::DRIVE_BOT_RIGHT_MOTOR, vex::ratio6_1, RIGHT_MOTORS_INVERTED};

    vex::motor_group leftMotorGroup{topLeftMotor, midLeftMotor, botLeftMotor};
    vex::motor_group rightMotorGroup{topRightMotor, midRightMotor, botRightMotor};

    // encoders
    // vex::rotation leftEncoder{Ports::DRIVE_LEFT_ENCODER};
    // vex::rotation rightEncoder{Ports::DRIVE_RIGHT_ENCODER};
    // vex::rotation backEncoder{Ports::DRIVE_BOTTOM_ENCODER};

    // odometry
    Odometry odometry{leftEncoder, rightEncoder, backEncoder};

    void setVelocityLeftMotors(double rpm);
    void setVelocityRightMotors(double rpm);

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
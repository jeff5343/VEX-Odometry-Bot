
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
    static constexpr bool LEFT_MOTORS_INVERTED = true;
    static constexpr bool RIGHT_MOTORS_INVERTED = false;

    // motors (top motors are the elevate ones)
    vex::motor frontLeftMotor{
        vex::PORT4, vex::ratio6_1, LEFT_MOTORS_INVERTED};
    vex::motor topLeftMotor{
        vex::PORT6, vex::ratio6_1, !LEFT_MOTORS_INVERTED};
    vex::motor backLeftMotor{
        vex::PORT5, vex::ratio6_1, LEFT_MOTORS_INVERTED};
    vex::motor frontRightMotor{
        vex::PORT1, vex::ratio6_1, RIGHT_MOTORS_INVERTED};
    vex::motor topRightMotor{
        vex::PORT3, vex::ratio6_1, !RIGHT_MOTORS_INVERTED};
    vex::motor backRightMotor{
        vex::PORT2, vex::ratio6_1, RIGHT_MOTORS_INVERTED};

    vex::motor_group leftMotorGroup{frontLeftMotor, topLeftMotor, backLeftMotor};
    vex::motor_group rightMotorGroup{frontRightMotor, topRightMotor, backRightMotor};

    // encoders
    // vex::rotation leftEncoder{vex::PORT9};
    // vex::rotation rightEncoder{vex::PORT10};
    // vex::rotation backEncoder{vex::PORT16};

    // odometry
    // Odometry odometry{leftEncoder, rightEncoder, backEncoder};

public:
    // TODO: PUT BACK P:RIVATE!!
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
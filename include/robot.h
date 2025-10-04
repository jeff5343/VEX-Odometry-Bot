#ifndef __ROBOT_H_INCLUDED__
#define __ROBOT_H_INCLUDED__

#include "subsystems/drivetrain.h"
#include "commands/pid_drive.h"

class Robot
{
private:
    static const vex::brain BRAIN;

    // subsystems
    Drivetrain drivetrain{};

    PidDrive pidDrive{
        // turning pid constants
        PidConstants{1.0, 0.0, 0.0}, 
        // straight pid constants
        PidConstants{1.0, 0.0, 0.0}, 
        drivetrain};

    const vex::controller controller{};

    bool driveSetpointSet = false;

public:
    /* get brain for encoder ports */
    static const vex::brain &getBrain() { return BRAIN; }

    /* called in pre_auton */
    void init();

    /* called every 20ms in autonomous */
    void autonomousPeriodic();

    /* called every 20ms in usercontrol */
    void usercontrolPeriodic();
};

#endif
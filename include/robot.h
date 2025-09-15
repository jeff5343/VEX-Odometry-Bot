#ifndef __ROBOT_H_INCLUDED__
#define __ROBOT_H_INCLUDED__

#include "drivetrain.h"

class Robot
{
private:
    static const vex::brain BRAIN;

    // subsystems
    Drivetrain drivetrain{};

    const vex::controller controller{};

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
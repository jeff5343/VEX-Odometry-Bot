#ifndef __PORTS_H_INCLUDED__
#define __PORTS_H_INCLUDED__

#include "vex.h"

class Ports
{
public:
    static const int32_t DRIVE_TOP_LEFT_MOTOR;
    static const int32_t DRIVE_MID_LEFT_MOTOR;
    static const int32_t DRIVE_BOT_LEFT_MOTOR;
    static const int32_t DRIVE_TOP_RIGHT_MOTOR;
    static const int32_t DRIVE_MID_RIGHT_MOTOR;
    static const int32_t DRIVE_BOT_RIGHT_MOTOR;

    // cannot be const because encoder constructor requires non-const argument
    static vex::triport::port DRIVE_LEFT_ENCODER;
    static vex::triport::port DRIVE_RIGHT_ENCODER;
    static vex::triport::port DRIVE_BOTTOM_ENCODER;
};

#endif

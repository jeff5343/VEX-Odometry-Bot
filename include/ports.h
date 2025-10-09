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

    static const int32_t DRIVE_LEFT_ENCODER;
    static const int32_t DRIVE_RIGHT_ENCODER;
    static const int32_t DRIVE_BOTTOM_ENCODER;
};

#endif

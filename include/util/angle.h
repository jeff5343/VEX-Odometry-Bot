#ifndef __ANGLE_H_INCLUDED__
#define __ANGLE_H_INCLUDED__

#include <cmath>

namespace Angle
{
    /* Clamps radians between 0 and 2pi */
    inline double wrapRadians(double radians)
    {
        double twopi = 2 * M_PI;
        return radians - (twopi * std::floor(radians / twopi));
    }
}

#endif
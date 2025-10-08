#ifndef __MATH_UTIL_H_INCLUDED__
#define __MATH_UTIL_H_INCLUDED__

#include <algorithm>

class MathUtil
{
public:
    static double clamp(double value, double low, double high)
    {
        return std::max(low, std::min(value, high));
    }
};

#endif
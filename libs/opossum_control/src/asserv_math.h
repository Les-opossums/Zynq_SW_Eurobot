#ifndef ASSERV_MATH_H
#define ASSERV_MATH_H

static inline float limit_float(float value, float lower, float upper)
{
    if (value < lower)
        return lower;
    if (value > upper)
        return upper;
    return value;
}

#endif

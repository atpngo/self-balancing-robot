#ifndef _UTIL
#define _UTIL

inline double clamp(double value, double low, double high) {
    if (value > high)
        return high;
    else if (value < low)
        return low;
    return value;
}

#endif // _UTIL
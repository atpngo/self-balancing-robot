#ifndef _UTIL
#define _UTIL

inline int clamp(int value, int low, int high) {
    if (value > high)
        return high;
    else if (value < low)
        return low;
    return value;
}

#endif // _UTIL
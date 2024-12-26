#ifndef _UTIL
#define _UTIL

#define M_PI		3.14159265358979323846

inline double clamp(double value, double low, double high) {
    if (value > high)
        return high;
    else if (value < low)
        return low;
    return value;
}

inline double degToRad(double degrees) {
    return degrees * (180.0/M_PI);
}



#endif // _UTIL
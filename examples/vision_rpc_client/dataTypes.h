#ifndef __DATA_TYPES_H__
#define __DATA_TYPES_H__

#include <thread>
#include <chrono>
#include <cmath>

typedef struct _Point {
    double x;
    double y;
    double z;
} Point;

typedef struct _Quaternion {
    double x;
    double y;
    double z;
    double w;

    void fromEuler(double roll, double pitch, double yaw) { // roll (x), pitch (y), yaw (z), angles are in radians
        double cr = cos(roll * 0.5);
        double sr = sin(roll * 0.5);
        double cp = cos(pitch * 0.5);
        double sp = sin(pitch * 0.5);
        double cy = cos(yaw * 0.5);
        double sy = sin(yaw * 0.5);

        w = cr * cp * cy + sr * sp * sy;
        x = sr * cp * cy - cr * sp * sy;
        y = cr * sp * cy + sr * cp * sy;
        z = cr * cp * sy - sr * sp * cy;
    }
} Quaternion;

typedef struct _Gridmap
{
    double hmap[120][120];          // cell height in meter
    uint8_t steppability[120][120]; // 0:non-steppable, 1:steppable
    Point offset{0.0, 0.0, 0.0};
    Point center{0.0, 0.0, 0.0};
    double resolution{0.05};
    uint32_t dim_x{120};
    uint32_t dim_y{120};
} Gridmap;

typedef struct _Odom
{
    Point position;
    Quaternion orientation;
} Odom;

#endif // __DATA_TYPES_H__
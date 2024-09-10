#ifndef __DATA_TYPES_H__
#define __DATA_TYPES_H__

#include "config.h"
#include <chrono>
#include <cmath>
#include <thread>

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
    double hmap[GRIDMAP_DIM_X][GRIDMAP_DIM_Y];          // cell height in meter
    uint8_t steppability[GRIDMAP_DIM_X][GRIDMAP_DIM_Y]; // 0:non-steppable, 1:steppable
    Point offset{0.0, 0.0, 0.0};
    Point center{0.0, 0.0, 0.0};
    double resolution{GRIDMAP_RES};
    uint32_t dim_x{GRIDMAP_DIM_X};
    uint32_t dim_y{GRIDMAP_DIM_Y};
} Gridmap;

typedef struct _Odom
{
    Point position;
    Quaternion orientation;
} Odom;

#endif // __DATA_TYPES_H__
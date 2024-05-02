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

#define MAX_VERTEX (128)
typedef struct _Polygon {
    Point center;
    Point normal;
    Point vertex[MAX_VERTEX];
    uint32_t vertex_count;
} Polygon;

#endif // __DATA_TYPES_H__
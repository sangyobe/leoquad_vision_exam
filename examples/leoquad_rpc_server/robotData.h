#ifndef ROBOTDATA_H
#define ROBOTDATA_H

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
#define MAX_POLYGON (16)
typedef struct _Polygon {
    Point center;
    Point normal;
    Point vertex[MAX_VERTEX];
    uint32_t vertex_count;
} Polygon;

typedef struct _SteppablePolygon
{
    Polygon polygon;
    uint32_t index;
    uint32_t properties;
} SteppablePolygon;

class RobotData
{
public:
    // kinematic odom
    Point basePos;
    Quaternion baseRot;

    // joint position
    double jointPos[12];

    // foot position
    Point footPos[4];

    // foot contact
    bool footContact[4];

    // visual odom received
    Point visualOdomPos;
    Quaternion visualOdomRot;

    // steppable and unsteppable areas
    SteppablePolygon steppables[MAX_POLYGON];
    uint32_t steppablesCount;
};

#endif // ROBOTDATA_H
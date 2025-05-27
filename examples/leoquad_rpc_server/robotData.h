#ifndef ROBOTDATA_H
#define ROBOTDATA_H

#include <cmath>
#include <cstdint>
#include <ctime>
#include <dtCore/type.h>

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

#define GRIDMAP_RES (0.03)
#define GRIDMAP_DIM_X (120)
#define GRIDMAP_DIM_Y (120)

typedef struct _Gridmap
{
    double hmap[GRIDMAP_DIM_X][GRIDMAP_DIM_Y];          // cell height in meter
    uint8_t steppability[GRIDMAP_DIM_X][GRIDMAP_DIM_Y]; // 0:non-steppable, 1:steppable
    double costmap[GRIDMAP_DIM_X][GRIDMAP_DIM_Y];       // cell height in meter
    Point offset{0.0, 0.0, 0.0};
    Point center{0.0, 0.0, 0.0};
    double resolution{GRIDMAP_RES};
    uint32_t dim_x{GRIDMAP_DIM_X};
    uint32_t dim_y{GRIDMAP_DIM_Y};
} Gridmap;

typedef struct _SE2Pose
{
    double x;
    double y;
    double th;
} SE2Pose;

typedef struct _SE2Velocity
{
    double vx;
    double vy;
    double w;
} SE2Velocity;

#define NAVCOMMAND_TRAJ_MAX (32)

typedef struct _SE2Trajectory
{
    typedef struct _SE2TrajectoryPoint
    {
        SE2Pose pose;
        double duration;
    } SE2TrajectoryPoint;
    SE2TrajectoryPoint points[NAVCOMMAND_TRAJ_MAX];
    dt::TimeStamp ref_time;
    int nop; //<! number of valid points
} SE2Trajectory;

typedef struct _RobotCommand
{
    typedef struct _NavCommand
    {
        enum CommandType
        {
            NavCommandType_SE2Traj = 0,
            NavCommandType_SE2Pose,
            NavCommandType_SE2Vel,
            // NavCommandType_SE3Traj,
            // NavCommandType_SE3Pose,
            // NavCommandType_SE3Vel,
        };
        CommandType cmd_type;
        dt::TimeStamp recv_time;
        dt::TimeStamp end_time; //<! The timestamp (in robot time) that the command is valid until.
        SE2Trajectory traj;     //<! trajectory(route) to follow
        SE2Pose pose;           //<! goal pose
        SE2Velocity vel;
    } NavCommand;
    NavCommand nav;
} RobotCommand;

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
    uint32_t visualOdomMsgSeq{0};

    // local grid map
    Gridmap gridmap;
    uint32_t gridmapMsgSeq{0};

    // robot command
    RobotCommand robotCommand;
    uint32_t robotCommandMsgSeq{0};
};

#endif // ROBOTDATA_H
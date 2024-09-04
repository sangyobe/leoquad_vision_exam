#ifndef __RPCSERVER_H__
#define __RPCSERVER_H__

#include <atomic>
#include <dtCore/src/dtDAQ/grpc/dtServiceListenerGrpc.hpp>
#include <dtCore/src/dtDAQ/grpc/dtStatePublisherGrpc.hpp>
#include <dtProto/Service.grpc.pb.h>
#include <dtProto/nav_msgs/Grid.pb.h>
#include <dtProto/robot_msgs/RobotState.pb.h>
#include <dtProto/sensor_msgs/Imu.pb.h>
#include <memory>
#include <string>
#include <thread>

class RobotData;
class RpcServer
{
public:
    RpcServer(void *robotData);
    virtual ~RpcServer();

public:
    void Run();
    void Stop();

private:
    std::unique_ptr<dt::DAQ::ServiceListenerGrpc> _navServiceListener{nullptr};
    std::unique_ptr<dt::DAQ::StatePublisherGrpc<dtproto::nav_msgs::GridTimeStamped>> _gridmapPublisher{nullptr};
    std::unique_ptr<dt::DAQ::StatePublisherGrpc<dtproto::robot_msgs::RobotStateTimeStamped>> _robotStatePublisher{nullptr};
    std::unique_ptr<dt::DAQ::StatePublisherGrpc<dtproto::sensor_msgs::ImuTimeStamped>> _imuPublisher{nullptr};

private:
    std::thread _dataUpdater;
    std::atomic_bool _runUpdater;
    RobotData *_robotData;
    dtproto::robot_msgs::RobotStateTimeStamped _robotStateMsg;
    dtproto::nav_msgs::GridTimeStamped _gridmapMsg;
    dtproto::sensor_msgs::ImuTimeStamped _imuMsg;
};

#endif // __RPCSERVER_H__
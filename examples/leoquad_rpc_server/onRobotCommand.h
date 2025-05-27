#ifndef __ONROBOTCOMMAND_H__
#define __ONROBOTCOMMAND_H__

#include "QuadrupedNav.grpc.pb.h"
#include "robotData.h"
#include <dtCore/src/dtDAQ/grpc/dtServiceListenerGrpc.hpp>
#include <dtProto/Service.grpc.pb.h>
#include <dtProto/robot_msgs/RobotCommand.pb.h>
#include <memory>
#include <string>

class OnRobotCommand : public dt::DAQ::ServiceListenerGrpc::Session
{
    using CallState = typename dt::DAQ::ServiceListenerGrpc::Session::CallState;
    using ServiceType = dtproto::quadruped::Nav::AsyncService;

public:
    OnRobotCommand(dt::DAQ::ServiceListenerGrpc *server, grpc::Service *service, grpc::ServerCompletionQueue *cq, void *udata = nullptr);
    ~OnRobotCommand();
    bool OnCompletionEvent(bool ok) override;

private:
    ::dtproto::robot_msgs::RobotCommandTimeStamped _request;
    ::dtproto::std_msgs::Response _response;
    ::grpc::ServerAsyncReader<::dtproto::std_msgs::Response, ::dtproto::robot_msgs::RobotCommandTimeStamped> _responder;
    RobotData *_robotData;
};

#endif // __ONROBOTCOMMAND_H__
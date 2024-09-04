#ifndef __ONLOCALGRIDMAP_H__
#define __ONLOCALGRIDMAP_H__

#include "QuadrupedNav.grpc.pb.h"
#include "robotData.h"
#include <dtCore/src/dtDAQ/grpc/dtServiceListenerGrpc.hpp>
#include <dtProto/Service.grpc.pb.h>
#include <dtProto/nav_msgs/Grid.pb.h>
#include <memory>
#include <string>

class OnLocalGridmap : public dt::DAQ::ServiceListenerGrpc::Session
{
    using CallState = typename dt::DAQ::ServiceListenerGrpc::Session::CallState;
    using ServiceType = dtproto::quadruped::Nav::AsyncService;

public:
    OnLocalGridmap(dt::DAQ::ServiceListenerGrpc *server, grpc::Service *service, grpc::ServerCompletionQueue *cq, void *udata = nullptr);
    ~OnLocalGridmap();
    bool OnCompletionEvent(bool ok) override;

private:
    ::dtproto::nav_msgs::GridTimeStamped _request;
    ::dtproto::std_msgs::Response _response;
    ::grpc::ServerAsyncReader<::dtproto::std_msgs::Response, ::dtproto::nav_msgs::GridTimeStamped> _responder;
    RobotData *_robotData;
};

#endif // __ONLOCALGRIDMAP_H__
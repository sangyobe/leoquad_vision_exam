#ifndef __ONVISUALODOM_H__
#define __ONVISUALODOM_H__

#include "QuadrupedNav.grpc.pb.h"
#include "robotData.h"
#include <dtCore/src/dtDAQ/grpc/dtServiceListenerGrpc.hpp>
#include <dtProto/Service.grpc.pb.h>
#include <dtProto/nav_msgs/Odom.pb.h>
#include <memory>
#include <string>

class OnVisualOdom : public dtCore::dtServiceListenerGrpc::Session
{
    using CallState = typename dtCore::dtServiceListenerGrpc::Session::CallState;
    using ServiceType = dtproto::quadruped::Nav::AsyncService;

public:
    OnVisualOdom(dtCore::dtServiceListenerGrpc *server, grpc::Service *service, grpc::ServerCompletionQueue *cq, void *udata = nullptr);
    ~OnVisualOdom();
    bool OnCompletionEvent(bool ok) override;

private:
    ::dtproto::nav_msgs::OdomTimeStamped _request;
    ::dtproto::std_msgs::Response _response;
    ::grpc::ServerAsyncReader<::dtproto::std_msgs::Response, ::dtproto::nav_msgs::OdomTimeStamped> _responder;
    RobotData *_robotData;
};

#endif // __ONVISUALODOM_H__
#ifndef __ONNOTIFYSTEPPABLEAREA_H__
#define __ONNOTIFYSTEPPABLEAREA_H__

#include <dtCore/src/dtDAQ/grpc/dtServiceListenerGrpc.hpp>
#include <string>
#include <memory>
#include "QuadrupedNav.grpc.pb.h"
#include "robotData.h"

class OnNotifySteppableArea : public dtCore::dtServiceListenerGrpc::Session
{
    using CallState = typename dtCore::dtServiceListenerGrpc::Session::CallState;
    using ServiceType = dtproto::quadruped::Nav::AsyncService;

public:
    OnNotifySteppableArea(dtCore::dtServiceListenerGrpc *server, grpc::Service *service, grpc::ServerCompletionQueue *cq, void *udata = nullptr);
    ~OnNotifySteppableArea();
    bool OnCompletionEvent(bool ok) override;

private:
    ::dtproto::nav_msgs::SteppableAreaTimeStamped _request;
    ::dtproto::std_msgs::Response _response;
    ::grpc::ServerAsyncResponseWriter<::dtproto::std_msgs::Response> _responder;
    RobotData* _robotData;
};

#endif // __ONNOTIFYSTEPPABLEAREA_H__
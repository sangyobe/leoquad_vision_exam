#ifndef __ONREQUESTODOMETRY_H__
#define __ONREQUESTODOMETRY_H__

#include <dtCore/src/dtDAQ/grpc/dtServiceListenerGrpc.hpp>
#include <string>
#include <memory>
#include "QuadrupedNav.grpc.pb.h"
#include "robotData.h"

class OnRequestOdometry : public dtCore::dtServiceListenerGrpc::Session
{
    using CallState = typename dtCore::dtServiceListenerGrpc::Session::CallState;
    using ServiceType = dtproto::quadruped::Nav::AsyncService;

public:
    OnRequestOdometry(dtCore::dtServiceListenerGrpc *server, grpc::Service *service, grpc::ServerCompletionQueue *cq, void *udata = nullptr);
    ~OnRequestOdometry();
    bool OnCompletionEvent(bool ok) override;

private:
    ::dtproto::nav_msgs::OdomTimeStamped _request;
    ::dtproto::quadruped::OdomWithJointPosTimeStamped _response;
    ::grpc::ServerAsyncResponseWriter<::dtproto::quadruped::OdomWithJointPosTimeStamped> _responder;
    RobotData* _robotData;
};

#endif // __ONREQUESTODOMETRY_H__
#ifndef __ONSTREAMODOMETRY_H__
#define __ONSTREAMODOMETRY_H__

#include <dtCore/src/dtDAQ/grpc/dtServiceListenerGrpc.hpp>
#include <string>
#include <memory>
#include "QuadrupedNav.grpc.pb.h"
#include "robotData.h"

class OnStreamOdometry : public dtCore::dtServiceListenerGrpc::Session
{
    using CallState = typename dtCore::dtServiceListenerGrpc::Session::CallState;
    using ServiceType = dtproto::quadruped::Nav::AsyncService;

public:
    OnStreamOdometry(dtCore::dtServiceListenerGrpc *server, grpc::Service *service, grpc::ServerCompletionQueue *cq, void *udata = nullptr);
    ~OnStreamOdometry();
    bool OnCompletionEvent(bool ok) override;

private:
    ::dtproto::nav_msgs::OdomTimeStamped _request;
    ::dtproto::quadruped::OdomWithJointPosTimeStamped _response;
    ::grpc::ServerAsyncReaderWriter<::dtproto::quadruped::OdomWithJointPosTimeStamped, ::dtproto::nav_msgs::OdomTimeStamped> _responder;
    RobotData* _robotData;
};

#endif // __ONSTREAMODOMETRY_H__
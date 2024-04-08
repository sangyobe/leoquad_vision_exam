#ifndef __REQUESTODOMETRYCALL_H__
#define __REQUESTODOMETRYCALL_H__

#include <dtCore/src/dtDAQ/grpc/dtServiceCallerGrpc.hpp>
#include <memory>
#include <string>
#include "QuadrupedNav.grpc.pb.h"
#include "emulOdom.h"
#include "emulSteppables.h"

using ServiceType = dtproto::quadruped::Nav;

class RequestOdometryCall : public dtCore::dtServiceCallerGrpc<ServiceType>::Call {
    using CallState = typename dtCore::dtServiceCallerGrpc<ServiceType>::Call::CallState;

public:
    RequestOdometryCall(ServiceType::Stub *stub, grpc::CompletionQueue *cq, void *udata = nullptr);
    ~RequestOdometryCall();
  
    bool OnCompletionEvent(bool ok) override;

private:
    ::dtproto::nav_msgs::OdomTimeStamped _request;
    ::dtproto::quadruped::OdomWithJointPosTimeStamped _response;
    std::unique_ptr<::grpc::ClientAsyncResponseReader<::dtproto::quadruped::OdomWithJointPosTimeStamped>> _responder;
    OdomData* _odomData{nullptr};
    static uint32_t _req_seq;
};

#endif // __REQUESTODOMETRYCALL_H__
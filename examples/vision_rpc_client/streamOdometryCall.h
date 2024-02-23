#ifndef __STREAMODOMETRYCALL_H__
#define __STREAMODOMETRYCALL_H__

#include <dtCore/src/dtDAQ/grpc/dtServiceCallerGrpc.hpp>
#include <memory>
#include <string>
#include "QuadrupedNav.grpc.pb.h"
#include "emulOdom.h"
#include "emulSteppables.h"

using ServiceType = dtproto::quadruped::Nav;

class StreamOdometryCall : public dtCore::dtServiceCallerGrpc<ServiceType>::Call {
    using CallState = typename dtCore::dtServiceCallerGrpc<ServiceType>::Call::CallState;

public:
    StreamOdometryCall(ServiceType::Stub *stub, grpc::CompletionQueue *cq, void *udata = nullptr);
    ~StreamOdometryCall();
  
    bool OnCompletionEvent(bool ok) override;

private:
    ::dtproto::nav_msgs::OdomTimeStamped _request;
    ::dtproto::quadruped::OdomWithJointPosTimeStamped _response;
    std::unique_ptr<::grpc::ClientAsyncReaderWriter<::dtproto::nav_msgs::OdomTimeStamped, ::dtproto::quadruped::OdomWithJointPosTimeStamped>> _responder;
    OdomData* _odomData{nullptr};
    // FOR TESTING
    int _msg_count{0};
};

#endif // __STREAMODOMETRYCALL_H__
#ifndef __NOTIFYSTEPPABLEAREACALL_H__
#define __NOTIFYSTEPPABLEAREACALL_H__

#include <dtCore/src/dtDAQ/grpc/dtServiceCallerGrpc.hpp>
#include <memory>
#include <string>
#include "QuadrupedNav.grpc.pb.h"
#include "emulOdom.h"
#include "emulSteppables.h"

using ServiceType = dtproto::quadruped::Nav;

class NotifySteppableAreaCall : public dtCore::dtServiceCallerGrpc<ServiceType>::Call {
    using CallState = typename dtCore::dtServiceCallerGrpc<ServiceType>::Call::CallState;

public:
    NotifySteppableAreaCall(ServiceType::Stub *stub, grpc::CompletionQueue *cq, void *udata = nullptr);
    ~NotifySteppableAreaCall();
  
    bool OnCompletionEvent(bool ok) override;

private:
    ::dtproto::nav_msgs::SteppableAreaTimeStamped _request;
    ::dtproto::std_msgs::Response _response;
    std::unique_ptr<::grpc::ClientAsyncResponseReader<::dtproto::std_msgs::Response>> _responder;
    SteppableArea* _steppables{nullptr};
    static uint32_t _req_seq;
};

#endif // __NOTIFYSTEPPABLEAREACALL_H__
#ifndef __PUBVISUALODOM_H__
#define __PUBVISUALODOM_H__

#include "QuadrupedNav.grpc.pb.h"
#include "emulOdom.h"
#include <dtCore/src/dtDAQ/grpc/dtServiceCallerGrpc.hpp>
#include <memory>
#include <string>

using ServiceType = dtproto::quadruped::Nav;

class PubVisualOdometry : public dt::DAQ::ServiceCallerGrpc<ServiceType>::Call
{
    using CallState = typename dt::DAQ::ServiceCallerGrpc<ServiceType>::Call::CallState;

public:
    PubVisualOdometry(ServiceType::Stub *stub, grpc::CompletionQueue *cq, void *udata = nullptr);
    ~PubVisualOdometry();

    bool OnCompletionEvent(bool ok) override;

    bool Publish(const Odom &odom);

private:
    ::dtproto::nav_msgs::OdomTimeStamped _request;
    ::dtproto::std_msgs::Response _response;
    std::unique_ptr<::grpc::ClientAsyncWriter<::dtproto::nav_msgs::OdomTimeStamped>> _writer;
    Odom *_odomData{nullptr};
};

#endif // __PUBVISUALODOM_H__
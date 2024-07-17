#ifndef __PUBGRIDMAP_H__
#define __PUBGRIDMAP_H__

#include "QuadrupedNav.grpc.pb.h"
#include "emulGridmap.h"
#include <dtCore/src/dtDAQ/grpc/dtServiceCallerGrpc.hpp>
#include <memory>
#include <string>

using ServiceType = dtproto::quadruped::Nav;

class PubGridmap : public dt::DAQ::ServiceCallerGrpc<ServiceType>::Call
{
    using CallState = typename dt::DAQ::ServiceCallerGrpc<ServiceType>::Call::CallState;

public:
    PubGridmap(ServiceType::Stub *stub, grpc::CompletionQueue *cq, void *udata = nullptr);
    ~PubGridmap();

    bool OnCompletionEvent(bool ok) override;

    bool Publish(const GridmapData &gridmap);

private:
    ::dtproto::nav_msgs::GridTimeStamped _request;
    ::dtproto::std_msgs::Response _response;
    std::unique_ptr<::grpc::ClientAsyncWriter<::dtproto::nav_msgs::GridTimeStamped>> _writer;
    GridmapData *_gridmapData{nullptr};
};

#endif // __PUBGRIDMAP_H__
#ifndef __PUBGRIDMAP_H__
#define __PUBGRIDMAP_H__

#include "QuadrupedNav.grpc.pb.h"
#include "emulGridmap.h"
#include <dtCore/src/dtDAQ/grpc/dtServiceCallerGrpc.hpp>
#include <memory>
#include <string>

class PubGridmap : public dt::DAQ::ServiceCallerGrpc<dtproto::quadruped::Nav>::Call
{
    using CallState = typename dt::DAQ::ServiceCallerGrpc<dtproto::quadruped::Nav>::Call::CallState;

public:
    PubGridmap(dtproto::quadruped::Nav::Stub *stub, grpc::CompletionQueue *cq, void *udata = nullptr);
    ~PubGridmap();

    bool OnCompletionEvent(bool ok) override;

    bool Publish(const Gridmap &gridmap);

private:
    ::dtproto::nav_msgs::GridTimeStamped _request;
    ::dtproto::std_msgs::Response _response;
    std::unique_ptr<::grpc::ClientAsyncWriter<::dtproto::nav_msgs::GridTimeStamped>> _writer;
    Gridmap *_gridmapData{nullptr};
    uint32_t _msgSeq{0};
};

#endif // __PUBGRIDMAP_H__
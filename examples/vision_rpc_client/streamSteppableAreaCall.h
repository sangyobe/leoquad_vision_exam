#ifndef __STREAMSTEPPABLEAREACALL_H__
#define __STREAMSTEPPABLEAREACALL_H__

#include "QuadrupedNav.grpc.pb.h"
#include "emulOdom.h"
#include "emulSteppables.h"
#include <dtCore/src/dtDAQ/grpc/dtServiceCallerGrpc.hpp>
#include <memory>
#include <string>

using ServiceType = dtproto::quadruped::Nav;

class StreamSteppableAreaCall
    : public dtCore::dtServiceCallerGrpc<ServiceType>::Call {
  using CallState =
      typename dtCore::dtServiceCallerGrpc<ServiceType>::Call::CallState;

public:
  StreamSteppableAreaCall(ServiceType::Stub *stub, grpc::CompletionQueue *cq,
                          void *udata = nullptr);
  ~StreamSteppableAreaCall();

  bool OnCompletionEvent(bool ok) override;

private:
  ::dtproto::nav_msgs::SteppableAreaTimeStamped _request;
  ::dtproto::std_msgs::Response _response;
  std::unique_ptr<
      ::grpc::ClientAsyncWriter<::dtproto::nav_msgs::SteppableAreaTimeStamped>>
      _responder;
  SteppableArea *_steppables{nullptr};
  // FOR TESTING
  int _steppable_count{0};
};

#endif // __STREAMSTEPPABLEAREACALL_H__
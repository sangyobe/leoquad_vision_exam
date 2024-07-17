// #ifndef __ONNOTIFYSTEPPABLEAREA_H__
// #define __ONNOTIFYSTEPPABLEAREA_H__

// #include <dtCore/src/dtDAQ/grpc/dtServiceListenerGrpc.hpp>
// #include <string>
// #include <memory>
// #include "QuadrupedNav.grpc.pb.h"
// #include "robotData.h"

// class OnNotifySteppableArea : public dt::DAQ::ServiceListenerGrpc::Session
// {
//     using CallState = typename dt::DAQ::ServiceListenerGrpc::Session::CallState;
//     using ServiceType = dtproto::quadruped::Nav::AsyncService;

// public:
//     OnNotifySteppableArea(dt::DAQ::ServiceListenerGrpc *server, grpc::Service *service, grpc::ServerCompletionQueue *cq, void *udata = nullptr);
//     ~OnNotifySteppableArea();
//     bool OnCompletionEvent(bool ok) override;

// private:
//     ::dtproto::nav_msgs::SteppableAreaTimeStamped _request;
//     ::dtproto::std_msgs::Response _response;
//     ::grpc::ServerAsyncResponseWriter<::dtproto::std_msgs::Response> _responder;
//     RobotData* _robotData;
// };

// #endif // __ONNOTIFYSTEPPABLEAREA_H__
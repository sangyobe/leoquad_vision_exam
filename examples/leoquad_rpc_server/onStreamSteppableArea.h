// #ifndef __ONSTREAMSTEPPABLEAREA_H__
// #define __ONSTREAMSTEPPABLEAREA_H__

// #include <dtCore/src/dtDAQ/grpc/dtServiceListenerGrpc.hpp>
// #include <string>
// #include <memory>
// #include "QuadrupedNav.grpc.pb.h"
// #include "robotData.h"

// class OnStreamSteppableArea : public dt::DAQ::ServiceListenerGrpc::Session
// {
//     using CallState = typename dt::DAQ::ServiceListenerGrpc::Session::CallState;
//     using ServiceType = dtproto::quadruped::Nav::AsyncService;

// public:
//     OnStreamSteppableArea(dt::DAQ::ServiceListenerGrpc *server, grpc::Service *service, grpc::ServerCompletionQueue *cq, void *udata = nullptr);
//     ~OnStreamSteppableArea();
//     bool OnCompletionEvent(bool ok) override;

// private:
//     ::dtproto::nav_msgs::SteppableAreaTimeStamped _request;
//     ::dtproto::std_msgs::Response _response;
//     ::grpc::ServerAsyncReader<::dtproto::std_msgs::Response, ::dtproto::nav_msgs::SteppableAreaTimeStamped> _responder;
//     RobotData* _robotData;
// };

// #endif // __ONSTREAMSTEPPABLEAREA_H__
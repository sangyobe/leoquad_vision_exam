// Generated by the gRPC C++ plugin.
// If you make any local change, they will be lost.
// source: dtProto/Service.proto

#include "dtProto/Service.pb.h"
#include "dtProto/Service.grpc.pb.h"

#include <functional>
#include <grpcpp/support/async_stream.h>
#include <grpcpp/support/async_unary_call.h>
#include <grpcpp/impl/channel_interface.h>
#include <grpcpp/impl/client_unary_call.h>
#include <grpcpp/support/client_callback.h>
#include <grpcpp/support/message_allocator.h>
#include <grpcpp/support/method_handler.h>
#include <grpcpp/impl/rpc_service_method.h>
#include <grpcpp/support/server_callback.h>
#include <grpcpp/impl/server_callback_handlers.h>
#include <grpcpp/server_context.h>
#include <grpcpp/impl/service_type.h>
#include <grpcpp/support/sync_stream.h>
namespace dtproto {

static const char* dtService_method_names[] = {
  "/dtproto.dtService/Version",
  "/dtproto.dtService/StreamState",
  "/dtproto.dtService/Move",
  "/dtproto.dtService/MoveJoint",
  "/dtproto.dtService/Command",
  "/dtproto.dtService/QueryRobotInfo",
};

std::unique_ptr< dtService::Stub> dtService::NewStub(const std::shared_ptr< ::grpc::ChannelInterface>& channel, const ::grpc::StubOptions& options) {
  (void)options;
  std::unique_ptr< dtService::Stub> stub(new dtService::Stub(channel, options));
  return stub;
}

dtService::Stub::Stub(const std::shared_ptr< ::grpc::ChannelInterface>& channel, const ::grpc::StubOptions& options)
  : channel_(channel), rpcmethod_Version_(dtService_method_names[0], options.suffix_for_stats(),::grpc::internal::RpcMethod::NORMAL_RPC, channel)
  , rpcmethod_StreamState_(dtService_method_names[1], options.suffix_for_stats(),::grpc::internal::RpcMethod::SERVER_STREAMING, channel)
  , rpcmethod_Move_(dtService_method_names[2], options.suffix_for_stats(),::grpc::internal::RpcMethod::NORMAL_RPC, channel)
  , rpcmethod_MoveJoint_(dtService_method_names[3], options.suffix_for_stats(),::grpc::internal::RpcMethod::NORMAL_RPC, channel)
  , rpcmethod_Command_(dtService_method_names[4], options.suffix_for_stats(),::grpc::internal::RpcMethod::NORMAL_RPC, channel)
  , rpcmethod_QueryRobotInfo_(dtService_method_names[5], options.suffix_for_stats(),::grpc::internal::RpcMethod::NORMAL_RPC, channel)
  {}

::grpc::Status dtService::Stub::Version(::grpc::ClientContext* context, const ::google::protobuf::Empty& request, ::google::protobuf::StringValue* response) {
  return ::grpc::internal::BlockingUnaryCall< ::google::protobuf::Empty, ::google::protobuf::StringValue, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(channel_.get(), rpcmethod_Version_, context, request, response);
}

void dtService::Stub::async::Version(::grpc::ClientContext* context, const ::google::protobuf::Empty* request, ::google::protobuf::StringValue* response, std::function<void(::grpc::Status)> f) {
  ::grpc::internal::CallbackUnaryCall< ::google::protobuf::Empty, ::google::protobuf::StringValue, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(stub_->channel_.get(), stub_->rpcmethod_Version_, context, request, response, std::move(f));
}

void dtService::Stub::async::Version(::grpc::ClientContext* context, const ::google::protobuf::Empty* request, ::google::protobuf::StringValue* response, ::grpc::ClientUnaryReactor* reactor) {
  ::grpc::internal::ClientCallbackUnaryFactory::Create< ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(stub_->channel_.get(), stub_->rpcmethod_Version_, context, request, response, reactor);
}

::grpc::ClientAsyncResponseReader< ::google::protobuf::StringValue>* dtService::Stub::PrepareAsyncVersionRaw(::grpc::ClientContext* context, const ::google::protobuf::Empty& request, ::grpc::CompletionQueue* cq) {
  return ::grpc::internal::ClientAsyncResponseReaderHelper::Create< ::google::protobuf::StringValue, ::google::protobuf::Empty, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(channel_.get(), cq, rpcmethod_Version_, context, request);
}

::grpc::ClientAsyncResponseReader< ::google::protobuf::StringValue>* dtService::Stub::AsyncVersionRaw(::grpc::ClientContext* context, const ::google::protobuf::Empty& request, ::grpc::CompletionQueue* cq) {
  auto* result =
    this->PrepareAsyncVersionRaw(context, request, cq);
  result->StartCall();
  return result;
}

::grpc::ClientReader< ::dtproto::std_msgs::State>* dtService::Stub::StreamStateRaw(::grpc::ClientContext* context, const ::dtproto::std_msgs::Request& request) {
  return ::grpc::internal::ClientReaderFactory< ::dtproto::std_msgs::State>::Create(channel_.get(), rpcmethod_StreamState_, context, request);
}

void dtService::Stub::async::StreamState(::grpc::ClientContext* context, const ::dtproto::std_msgs::Request* request, ::grpc::ClientReadReactor< ::dtproto::std_msgs::State>* reactor) {
  ::grpc::internal::ClientCallbackReaderFactory< ::dtproto::std_msgs::State>::Create(stub_->channel_.get(), stub_->rpcmethod_StreamState_, context, request, reactor);
}

::grpc::ClientAsyncReader< ::dtproto::std_msgs::State>* dtService::Stub::AsyncStreamStateRaw(::grpc::ClientContext* context, const ::dtproto::std_msgs::Request& request, ::grpc::CompletionQueue* cq, void* tag) {
  return ::grpc::internal::ClientAsyncReaderFactory< ::dtproto::std_msgs::State>::Create(channel_.get(), cq, rpcmethod_StreamState_, context, request, true, tag);
}

::grpc::ClientAsyncReader< ::dtproto::std_msgs::State>* dtService::Stub::PrepareAsyncStreamStateRaw(::grpc::ClientContext* context, const ::dtproto::std_msgs::Request& request, ::grpc::CompletionQueue* cq) {
  return ::grpc::internal::ClientAsyncReaderFactory< ::dtproto::std_msgs::State>::Create(channel_.get(), cq, rpcmethod_StreamState_, context, request, false, nullptr);
}

::grpc::Status dtService::Stub::Move(::grpc::ClientContext* context, const ::dtproto::robot_msgs::MoveControl& request, ::dtproto::std_msgs::Response* response) {
  return ::grpc::internal::BlockingUnaryCall< ::dtproto::robot_msgs::MoveControl, ::dtproto::std_msgs::Response, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(channel_.get(), rpcmethod_Move_, context, request, response);
}

void dtService::Stub::async::Move(::grpc::ClientContext* context, const ::dtproto::robot_msgs::MoveControl* request, ::dtproto::std_msgs::Response* response, std::function<void(::grpc::Status)> f) {
  ::grpc::internal::CallbackUnaryCall< ::dtproto::robot_msgs::MoveControl, ::dtproto::std_msgs::Response, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(stub_->channel_.get(), stub_->rpcmethod_Move_, context, request, response, std::move(f));
}

void dtService::Stub::async::Move(::grpc::ClientContext* context, const ::dtproto::robot_msgs::MoveControl* request, ::dtproto::std_msgs::Response* response, ::grpc::ClientUnaryReactor* reactor) {
  ::grpc::internal::ClientCallbackUnaryFactory::Create< ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(stub_->channel_.get(), stub_->rpcmethod_Move_, context, request, response, reactor);
}

::grpc::ClientAsyncResponseReader< ::dtproto::std_msgs::Response>* dtService::Stub::PrepareAsyncMoveRaw(::grpc::ClientContext* context, const ::dtproto::robot_msgs::MoveControl& request, ::grpc::CompletionQueue* cq) {
  return ::grpc::internal::ClientAsyncResponseReaderHelper::Create< ::dtproto::std_msgs::Response, ::dtproto::robot_msgs::MoveControl, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(channel_.get(), cq, rpcmethod_Move_, context, request);
}

::grpc::ClientAsyncResponseReader< ::dtproto::std_msgs::Response>* dtService::Stub::AsyncMoveRaw(::grpc::ClientContext* context, const ::dtproto::robot_msgs::MoveControl& request, ::grpc::CompletionQueue* cq) {
  auto* result =
    this->PrepareAsyncMoveRaw(context, request, cq);
  result->StartCall();
  return result;
}

::grpc::Status dtService::Stub::MoveJoint(::grpc::ClientContext* context, const ::dtproto::robot_msgs::JointControl& request, ::dtproto::std_msgs::Response* response) {
  return ::grpc::internal::BlockingUnaryCall< ::dtproto::robot_msgs::JointControl, ::dtproto::std_msgs::Response, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(channel_.get(), rpcmethod_MoveJoint_, context, request, response);
}

void dtService::Stub::async::MoveJoint(::grpc::ClientContext* context, const ::dtproto::robot_msgs::JointControl* request, ::dtproto::std_msgs::Response* response, std::function<void(::grpc::Status)> f) {
  ::grpc::internal::CallbackUnaryCall< ::dtproto::robot_msgs::JointControl, ::dtproto::std_msgs::Response, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(stub_->channel_.get(), stub_->rpcmethod_MoveJoint_, context, request, response, std::move(f));
}

void dtService::Stub::async::MoveJoint(::grpc::ClientContext* context, const ::dtproto::robot_msgs::JointControl* request, ::dtproto::std_msgs::Response* response, ::grpc::ClientUnaryReactor* reactor) {
  ::grpc::internal::ClientCallbackUnaryFactory::Create< ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(stub_->channel_.get(), stub_->rpcmethod_MoveJoint_, context, request, response, reactor);
}

::grpc::ClientAsyncResponseReader< ::dtproto::std_msgs::Response>* dtService::Stub::PrepareAsyncMoveJointRaw(::grpc::ClientContext* context, const ::dtproto::robot_msgs::JointControl& request, ::grpc::CompletionQueue* cq) {
  return ::grpc::internal::ClientAsyncResponseReaderHelper::Create< ::dtproto::std_msgs::Response, ::dtproto::robot_msgs::JointControl, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(channel_.get(), cq, rpcmethod_MoveJoint_, context, request);
}

::grpc::ClientAsyncResponseReader< ::dtproto::std_msgs::Response>* dtService::Stub::AsyncMoveJointRaw(::grpc::ClientContext* context, const ::dtproto::robot_msgs::JointControl& request, ::grpc::CompletionQueue* cq) {
  auto* result =
    this->PrepareAsyncMoveJointRaw(context, request, cq);
  result->StartCall();
  return result;
}

::grpc::Status dtService::Stub::Command(::grpc::ClientContext* context, const ::dtproto::robot_msgs::ControlCmd& request, ::dtproto::std_msgs::Response* response) {
  return ::grpc::internal::BlockingUnaryCall< ::dtproto::robot_msgs::ControlCmd, ::dtproto::std_msgs::Response, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(channel_.get(), rpcmethod_Command_, context, request, response);
}

void dtService::Stub::async::Command(::grpc::ClientContext* context, const ::dtproto::robot_msgs::ControlCmd* request, ::dtproto::std_msgs::Response* response, std::function<void(::grpc::Status)> f) {
  ::grpc::internal::CallbackUnaryCall< ::dtproto::robot_msgs::ControlCmd, ::dtproto::std_msgs::Response, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(stub_->channel_.get(), stub_->rpcmethod_Command_, context, request, response, std::move(f));
}

void dtService::Stub::async::Command(::grpc::ClientContext* context, const ::dtproto::robot_msgs::ControlCmd* request, ::dtproto::std_msgs::Response* response, ::grpc::ClientUnaryReactor* reactor) {
  ::grpc::internal::ClientCallbackUnaryFactory::Create< ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(stub_->channel_.get(), stub_->rpcmethod_Command_, context, request, response, reactor);
}

::grpc::ClientAsyncResponseReader< ::dtproto::std_msgs::Response>* dtService::Stub::PrepareAsyncCommandRaw(::grpc::ClientContext* context, const ::dtproto::robot_msgs::ControlCmd& request, ::grpc::CompletionQueue* cq) {
  return ::grpc::internal::ClientAsyncResponseReaderHelper::Create< ::dtproto::std_msgs::Response, ::dtproto::robot_msgs::ControlCmd, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(channel_.get(), cq, rpcmethod_Command_, context, request);
}

::grpc::ClientAsyncResponseReader< ::dtproto::std_msgs::Response>* dtService::Stub::AsyncCommandRaw(::grpc::ClientContext* context, const ::dtproto::robot_msgs::ControlCmd& request, ::grpc::CompletionQueue* cq) {
  auto* result =
    this->PrepareAsyncCommandRaw(context, request, cq);
  result->StartCall();
  return result;
}

::grpc::Status dtService::Stub::QueryRobotInfo(::grpc::ClientContext* context, const ::google::protobuf::Empty& request, ::dtproto::robot_msgs::RobotInfo* response) {
  return ::grpc::internal::BlockingUnaryCall< ::google::protobuf::Empty, ::dtproto::robot_msgs::RobotInfo, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(channel_.get(), rpcmethod_QueryRobotInfo_, context, request, response);
}

void dtService::Stub::async::QueryRobotInfo(::grpc::ClientContext* context, const ::google::protobuf::Empty* request, ::dtproto::robot_msgs::RobotInfo* response, std::function<void(::grpc::Status)> f) {
  ::grpc::internal::CallbackUnaryCall< ::google::protobuf::Empty, ::dtproto::robot_msgs::RobotInfo, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(stub_->channel_.get(), stub_->rpcmethod_QueryRobotInfo_, context, request, response, std::move(f));
}

void dtService::Stub::async::QueryRobotInfo(::grpc::ClientContext* context, const ::google::protobuf::Empty* request, ::dtproto::robot_msgs::RobotInfo* response, ::grpc::ClientUnaryReactor* reactor) {
  ::grpc::internal::ClientCallbackUnaryFactory::Create< ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(stub_->channel_.get(), stub_->rpcmethod_QueryRobotInfo_, context, request, response, reactor);
}

::grpc::ClientAsyncResponseReader< ::dtproto::robot_msgs::RobotInfo>* dtService::Stub::PrepareAsyncQueryRobotInfoRaw(::grpc::ClientContext* context, const ::google::protobuf::Empty& request, ::grpc::CompletionQueue* cq) {
  return ::grpc::internal::ClientAsyncResponseReaderHelper::Create< ::dtproto::robot_msgs::RobotInfo, ::google::protobuf::Empty, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(channel_.get(), cq, rpcmethod_QueryRobotInfo_, context, request);
}

::grpc::ClientAsyncResponseReader< ::dtproto::robot_msgs::RobotInfo>* dtService::Stub::AsyncQueryRobotInfoRaw(::grpc::ClientContext* context, const ::google::protobuf::Empty& request, ::grpc::CompletionQueue* cq) {
  auto* result =
    this->PrepareAsyncQueryRobotInfoRaw(context, request, cq);
  result->StartCall();
  return result;
}

dtService::Service::Service() {
  AddMethod(new ::grpc::internal::RpcServiceMethod(
      dtService_method_names[0],
      ::grpc::internal::RpcMethod::NORMAL_RPC,
      new ::grpc::internal::RpcMethodHandler< dtService::Service, ::google::protobuf::Empty, ::google::protobuf::StringValue, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(
          [](dtService::Service* service,
             ::grpc::ServerContext* ctx,
             const ::google::protobuf::Empty* req,
             ::google::protobuf::StringValue* resp) {
               return service->Version(ctx, req, resp);
             }, this)));
  AddMethod(new ::grpc::internal::RpcServiceMethod(
      dtService_method_names[1],
      ::grpc::internal::RpcMethod::SERVER_STREAMING,
      new ::grpc::internal::ServerStreamingHandler< dtService::Service, ::dtproto::std_msgs::Request, ::dtproto::std_msgs::State>(
          [](dtService::Service* service,
             ::grpc::ServerContext* ctx,
             const ::dtproto::std_msgs::Request* req,
             ::grpc::ServerWriter<::dtproto::std_msgs::State>* writer) {
               return service->StreamState(ctx, req, writer);
             }, this)));
  AddMethod(new ::grpc::internal::RpcServiceMethod(
      dtService_method_names[2],
      ::grpc::internal::RpcMethod::NORMAL_RPC,
      new ::grpc::internal::RpcMethodHandler< dtService::Service, ::dtproto::robot_msgs::MoveControl, ::dtproto::std_msgs::Response, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(
          [](dtService::Service* service,
             ::grpc::ServerContext* ctx,
             const ::dtproto::robot_msgs::MoveControl* req,
             ::dtproto::std_msgs::Response* resp) {
               return service->Move(ctx, req, resp);
             }, this)));
  AddMethod(new ::grpc::internal::RpcServiceMethod(
      dtService_method_names[3],
      ::grpc::internal::RpcMethod::NORMAL_RPC,
      new ::grpc::internal::RpcMethodHandler< dtService::Service, ::dtproto::robot_msgs::JointControl, ::dtproto::std_msgs::Response, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(
          [](dtService::Service* service,
             ::grpc::ServerContext* ctx,
             const ::dtproto::robot_msgs::JointControl* req,
             ::dtproto::std_msgs::Response* resp) {
               return service->MoveJoint(ctx, req, resp);
             }, this)));
  AddMethod(new ::grpc::internal::RpcServiceMethod(
      dtService_method_names[4],
      ::grpc::internal::RpcMethod::NORMAL_RPC,
      new ::grpc::internal::RpcMethodHandler< dtService::Service, ::dtproto::robot_msgs::ControlCmd, ::dtproto::std_msgs::Response, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(
          [](dtService::Service* service,
             ::grpc::ServerContext* ctx,
             const ::dtproto::robot_msgs::ControlCmd* req,
             ::dtproto::std_msgs::Response* resp) {
               return service->Command(ctx, req, resp);
             }, this)));
  AddMethod(new ::grpc::internal::RpcServiceMethod(
      dtService_method_names[5],
      ::grpc::internal::RpcMethod::NORMAL_RPC,
      new ::grpc::internal::RpcMethodHandler< dtService::Service, ::google::protobuf::Empty, ::dtproto::robot_msgs::RobotInfo, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(
          [](dtService::Service* service,
             ::grpc::ServerContext* ctx,
             const ::google::protobuf::Empty* req,
             ::dtproto::robot_msgs::RobotInfo* resp) {
               return service->QueryRobotInfo(ctx, req, resp);
             }, this)));
}

dtService::Service::~Service() {
}

::grpc::Status dtService::Service::Version(::grpc::ServerContext* context, const ::google::protobuf::Empty* request, ::google::protobuf::StringValue* response) {
  (void) context;
  (void) request;
  (void) response;
  return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
}

::grpc::Status dtService::Service::StreamState(::grpc::ServerContext* context, const ::dtproto::std_msgs::Request* request, ::grpc::ServerWriter< ::dtproto::std_msgs::State>* writer) {
  (void) context;
  (void) request;
  (void) writer;
  return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
}

::grpc::Status dtService::Service::Move(::grpc::ServerContext* context, const ::dtproto::robot_msgs::MoveControl* request, ::dtproto::std_msgs::Response* response) {
  (void) context;
  (void) request;
  (void) response;
  return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
}

::grpc::Status dtService::Service::MoveJoint(::grpc::ServerContext* context, const ::dtproto::robot_msgs::JointControl* request, ::dtproto::std_msgs::Response* response) {
  (void) context;
  (void) request;
  (void) response;
  return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
}

::grpc::Status dtService::Service::Command(::grpc::ServerContext* context, const ::dtproto::robot_msgs::ControlCmd* request, ::dtproto::std_msgs::Response* response) {
  (void) context;
  (void) request;
  (void) response;
  return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
}

::grpc::Status dtService::Service::QueryRobotInfo(::grpc::ServerContext* context, const ::google::protobuf::Empty* request, ::dtproto::robot_msgs::RobotInfo* response) {
  (void) context;
  (void) request;
  (void) response;
  return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
}


}  // namespace dtproto


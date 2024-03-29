// Generated by the gRPC C++ plugin.
// If you make any local change, they will be lost.
// source: dtProto/std_msgs/Heartbeat.proto
#ifndef GRPC_dtProto_2fstd_5fmsgs_2fHeartbeat_2eproto__INCLUDED
#define GRPC_dtProto_2fstd_5fmsgs_2fHeartbeat_2eproto__INCLUDED

#include "dtProto/std_msgs/Heartbeat.pb.h"

#include <functional>
#include <grpcpp/generic/async_generic_service.h>
#include <grpcpp/support/async_stream.h>
#include <grpcpp/support/async_unary_call.h>
#include <grpcpp/support/client_callback.h>
#include <grpcpp/client_context.h>
#include <grpcpp/completion_queue.h>
#include <grpcpp/support/message_allocator.h>
#include <grpcpp/support/method_handler.h>
#include <grpcpp/impl/proto_utils.h>
#include <grpcpp/impl/rpc_method.h>
#include <grpcpp/support/server_callback.h>
#include <grpcpp/impl/server_callback_handlers.h>
#include <grpcpp/server_context.h>
#include <grpcpp/impl/service_type.h>
#include <grpcpp/support/status.h>
#include <grpcpp/support/stub_options.h>
#include <grpcpp/support/sync_stream.h>

namespace dtproto {
namespace std_msgs {

class CommHealthCheck final {
 public:
  static constexpr char const* service_full_name() {
    return "dtproto.std_msgs.CommHealthCheck";
  }
  class StubInterface {
   public:
    virtual ~StubInterface() {}
    virtual ::grpc::Status Ping(::grpc::ClientContext* context, const ::dtproto::std_msgs::Heartbeat& request, ::dtproto::std_msgs::Heartbeat* response) = 0;
    std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::dtproto::std_msgs::Heartbeat>> AsyncPing(::grpc::ClientContext* context, const ::dtproto::std_msgs::Heartbeat& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::dtproto::std_msgs::Heartbeat>>(AsyncPingRaw(context, request, cq));
    }
    std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::dtproto::std_msgs::Heartbeat>> PrepareAsyncPing(::grpc::ClientContext* context, const ::dtproto::std_msgs::Heartbeat& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::dtproto::std_msgs::Heartbeat>>(PrepareAsyncPingRaw(context, request, cq));
    }
    std::unique_ptr< ::grpc::ClientReaderInterface< ::dtproto::std_msgs::LatencyCheckPayload>> CheckLatency(::grpc::ClientContext* context, const ::dtproto::std_msgs::LatencyCheckRequest& request) {
      return std::unique_ptr< ::grpc::ClientReaderInterface< ::dtproto::std_msgs::LatencyCheckPayload>>(CheckLatencyRaw(context, request));
    }
    std::unique_ptr< ::grpc::ClientAsyncReaderInterface< ::dtproto::std_msgs::LatencyCheckPayload>> AsyncCheckLatency(::grpc::ClientContext* context, const ::dtproto::std_msgs::LatencyCheckRequest& request, ::grpc::CompletionQueue* cq, void* tag) {
      return std::unique_ptr< ::grpc::ClientAsyncReaderInterface< ::dtproto::std_msgs::LatencyCheckPayload>>(AsyncCheckLatencyRaw(context, request, cq, tag));
    }
    std::unique_ptr< ::grpc::ClientAsyncReaderInterface< ::dtproto::std_msgs::LatencyCheckPayload>> PrepareAsyncCheckLatency(::grpc::ClientContext* context, const ::dtproto::std_msgs::LatencyCheckRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncReaderInterface< ::dtproto::std_msgs::LatencyCheckPayload>>(PrepareAsyncCheckLatencyRaw(context, request, cq));
    }
    class async_interface {
     public:
      virtual ~async_interface() {}
      virtual void Ping(::grpc::ClientContext* context, const ::dtproto::std_msgs::Heartbeat* request, ::dtproto::std_msgs::Heartbeat* response, std::function<void(::grpc::Status)>) = 0;
      virtual void Ping(::grpc::ClientContext* context, const ::dtproto::std_msgs::Heartbeat* request, ::dtproto::std_msgs::Heartbeat* response, ::grpc::ClientUnaryReactor* reactor) = 0;
      virtual void CheckLatency(::grpc::ClientContext* context, const ::dtproto::std_msgs::LatencyCheckRequest* request, ::grpc::ClientReadReactor< ::dtproto::std_msgs::LatencyCheckPayload>* reactor) = 0;
    };
    typedef class async_interface experimental_async_interface;
    virtual class async_interface* async() { return nullptr; }
    class async_interface* experimental_async() { return async(); }
   private:
    virtual ::grpc::ClientAsyncResponseReaderInterface< ::dtproto::std_msgs::Heartbeat>* AsyncPingRaw(::grpc::ClientContext* context, const ::dtproto::std_msgs::Heartbeat& request, ::grpc::CompletionQueue* cq) = 0;
    virtual ::grpc::ClientAsyncResponseReaderInterface< ::dtproto::std_msgs::Heartbeat>* PrepareAsyncPingRaw(::grpc::ClientContext* context, const ::dtproto::std_msgs::Heartbeat& request, ::grpc::CompletionQueue* cq) = 0;
    virtual ::grpc::ClientReaderInterface< ::dtproto::std_msgs::LatencyCheckPayload>* CheckLatencyRaw(::grpc::ClientContext* context, const ::dtproto::std_msgs::LatencyCheckRequest& request) = 0;
    virtual ::grpc::ClientAsyncReaderInterface< ::dtproto::std_msgs::LatencyCheckPayload>* AsyncCheckLatencyRaw(::grpc::ClientContext* context, const ::dtproto::std_msgs::LatencyCheckRequest& request, ::grpc::CompletionQueue* cq, void* tag) = 0;
    virtual ::grpc::ClientAsyncReaderInterface< ::dtproto::std_msgs::LatencyCheckPayload>* PrepareAsyncCheckLatencyRaw(::grpc::ClientContext* context, const ::dtproto::std_msgs::LatencyCheckRequest& request, ::grpc::CompletionQueue* cq) = 0;
  };
  class Stub final : public StubInterface {
   public:
    Stub(const std::shared_ptr< ::grpc::ChannelInterface>& channel, const ::grpc::StubOptions& options = ::grpc::StubOptions());
    ::grpc::Status Ping(::grpc::ClientContext* context, const ::dtproto::std_msgs::Heartbeat& request, ::dtproto::std_msgs::Heartbeat* response) override;
    std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::dtproto::std_msgs::Heartbeat>> AsyncPing(::grpc::ClientContext* context, const ::dtproto::std_msgs::Heartbeat& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::dtproto::std_msgs::Heartbeat>>(AsyncPingRaw(context, request, cq));
    }
    std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::dtproto::std_msgs::Heartbeat>> PrepareAsyncPing(::grpc::ClientContext* context, const ::dtproto::std_msgs::Heartbeat& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::dtproto::std_msgs::Heartbeat>>(PrepareAsyncPingRaw(context, request, cq));
    }
    std::unique_ptr< ::grpc::ClientReader< ::dtproto::std_msgs::LatencyCheckPayload>> CheckLatency(::grpc::ClientContext* context, const ::dtproto::std_msgs::LatencyCheckRequest& request) {
      return std::unique_ptr< ::grpc::ClientReader< ::dtproto::std_msgs::LatencyCheckPayload>>(CheckLatencyRaw(context, request));
    }
    std::unique_ptr< ::grpc::ClientAsyncReader< ::dtproto::std_msgs::LatencyCheckPayload>> AsyncCheckLatency(::grpc::ClientContext* context, const ::dtproto::std_msgs::LatencyCheckRequest& request, ::grpc::CompletionQueue* cq, void* tag) {
      return std::unique_ptr< ::grpc::ClientAsyncReader< ::dtproto::std_msgs::LatencyCheckPayload>>(AsyncCheckLatencyRaw(context, request, cq, tag));
    }
    std::unique_ptr< ::grpc::ClientAsyncReader< ::dtproto::std_msgs::LatencyCheckPayload>> PrepareAsyncCheckLatency(::grpc::ClientContext* context, const ::dtproto::std_msgs::LatencyCheckRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncReader< ::dtproto::std_msgs::LatencyCheckPayload>>(PrepareAsyncCheckLatencyRaw(context, request, cq));
    }
    class async final :
      public StubInterface::async_interface {
     public:
      void Ping(::grpc::ClientContext* context, const ::dtproto::std_msgs::Heartbeat* request, ::dtproto::std_msgs::Heartbeat* response, std::function<void(::grpc::Status)>) override;
      void Ping(::grpc::ClientContext* context, const ::dtproto::std_msgs::Heartbeat* request, ::dtproto::std_msgs::Heartbeat* response, ::grpc::ClientUnaryReactor* reactor) override;
      void CheckLatency(::grpc::ClientContext* context, const ::dtproto::std_msgs::LatencyCheckRequest* request, ::grpc::ClientReadReactor< ::dtproto::std_msgs::LatencyCheckPayload>* reactor) override;
     private:
      friend class Stub;
      explicit async(Stub* stub): stub_(stub) { }
      Stub* stub() { return stub_; }
      Stub* stub_;
    };
    class async* async() override { return &async_stub_; }

   private:
    std::shared_ptr< ::grpc::ChannelInterface> channel_;
    class async async_stub_{this};
    ::grpc::ClientAsyncResponseReader< ::dtproto::std_msgs::Heartbeat>* AsyncPingRaw(::grpc::ClientContext* context, const ::dtproto::std_msgs::Heartbeat& request, ::grpc::CompletionQueue* cq) override;
    ::grpc::ClientAsyncResponseReader< ::dtproto::std_msgs::Heartbeat>* PrepareAsyncPingRaw(::grpc::ClientContext* context, const ::dtproto::std_msgs::Heartbeat& request, ::grpc::CompletionQueue* cq) override;
    ::grpc::ClientReader< ::dtproto::std_msgs::LatencyCheckPayload>* CheckLatencyRaw(::grpc::ClientContext* context, const ::dtproto::std_msgs::LatencyCheckRequest& request) override;
    ::grpc::ClientAsyncReader< ::dtproto::std_msgs::LatencyCheckPayload>* AsyncCheckLatencyRaw(::grpc::ClientContext* context, const ::dtproto::std_msgs::LatencyCheckRequest& request, ::grpc::CompletionQueue* cq, void* tag) override;
    ::grpc::ClientAsyncReader< ::dtproto::std_msgs::LatencyCheckPayload>* PrepareAsyncCheckLatencyRaw(::grpc::ClientContext* context, const ::dtproto::std_msgs::LatencyCheckRequest& request, ::grpc::CompletionQueue* cq) override;
    const ::grpc::internal::RpcMethod rpcmethod_Ping_;
    const ::grpc::internal::RpcMethod rpcmethod_CheckLatency_;
  };
  static std::unique_ptr<Stub> NewStub(const std::shared_ptr< ::grpc::ChannelInterface>& channel, const ::grpc::StubOptions& options = ::grpc::StubOptions());

  class Service : public ::grpc::Service {
   public:
    Service();
    virtual ~Service();
    virtual ::grpc::Status Ping(::grpc::ServerContext* context, const ::dtproto::std_msgs::Heartbeat* request, ::dtproto::std_msgs::Heartbeat* response);
    virtual ::grpc::Status CheckLatency(::grpc::ServerContext* context, const ::dtproto::std_msgs::LatencyCheckRequest* request, ::grpc::ServerWriter< ::dtproto::std_msgs::LatencyCheckPayload>* writer);
  };
  template <class BaseClass>
  class WithAsyncMethod_Ping : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithAsyncMethod_Ping() {
      ::grpc::Service::MarkMethodAsync(0);
    }
    ~WithAsyncMethod_Ping() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status Ping(::grpc::ServerContext* /*context*/, const ::dtproto::std_msgs::Heartbeat* /*request*/, ::dtproto::std_msgs::Heartbeat* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    void RequestPing(::grpc::ServerContext* context, ::dtproto::std_msgs::Heartbeat* request, ::grpc::ServerAsyncResponseWriter< ::dtproto::std_msgs::Heartbeat>* response, ::grpc::CompletionQueue* new_call_cq, ::grpc::ServerCompletionQueue* notification_cq, void *tag) {
      ::grpc::Service::RequestAsyncUnary(0, context, request, response, new_call_cq, notification_cq, tag);
    }
  };
  template <class BaseClass>
  class WithAsyncMethod_CheckLatency : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithAsyncMethod_CheckLatency() {
      ::grpc::Service::MarkMethodAsync(1);
    }
    ~WithAsyncMethod_CheckLatency() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status CheckLatency(::grpc::ServerContext* /*context*/, const ::dtproto::std_msgs::LatencyCheckRequest* /*request*/, ::grpc::ServerWriter< ::dtproto::std_msgs::LatencyCheckPayload>* /*writer*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    void RequestCheckLatency(::grpc::ServerContext* context, ::dtproto::std_msgs::LatencyCheckRequest* request, ::grpc::ServerAsyncWriter< ::dtproto::std_msgs::LatencyCheckPayload>* writer, ::grpc::CompletionQueue* new_call_cq, ::grpc::ServerCompletionQueue* notification_cq, void *tag) {
      ::grpc::Service::RequestAsyncServerStreaming(1, context, request, writer, new_call_cq, notification_cq, tag);
    }
  };
  typedef WithAsyncMethod_Ping<WithAsyncMethod_CheckLatency<Service > > AsyncService;
  template <class BaseClass>
  class WithCallbackMethod_Ping : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithCallbackMethod_Ping() {
      ::grpc::Service::MarkMethodCallback(0,
          new ::grpc::internal::CallbackUnaryHandler< ::dtproto::std_msgs::Heartbeat, ::dtproto::std_msgs::Heartbeat>(
            [this](
                   ::grpc::CallbackServerContext* context, const ::dtproto::std_msgs::Heartbeat* request, ::dtproto::std_msgs::Heartbeat* response) { return this->Ping(context, request, response); }));}
    void SetMessageAllocatorFor_Ping(
        ::grpc::MessageAllocator< ::dtproto::std_msgs::Heartbeat, ::dtproto::std_msgs::Heartbeat>* allocator) {
      ::grpc::internal::MethodHandler* const handler = ::grpc::Service::GetHandler(0);
      static_cast<::grpc::internal::CallbackUnaryHandler< ::dtproto::std_msgs::Heartbeat, ::dtproto::std_msgs::Heartbeat>*>(handler)
              ->SetMessageAllocator(allocator);
    }
    ~WithCallbackMethod_Ping() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status Ping(::grpc::ServerContext* /*context*/, const ::dtproto::std_msgs::Heartbeat* /*request*/, ::dtproto::std_msgs::Heartbeat* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    virtual ::grpc::ServerUnaryReactor* Ping(
      ::grpc::CallbackServerContext* /*context*/, const ::dtproto::std_msgs::Heartbeat* /*request*/, ::dtproto::std_msgs::Heartbeat* /*response*/)  { return nullptr; }
  };
  template <class BaseClass>
  class WithCallbackMethod_CheckLatency : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithCallbackMethod_CheckLatency() {
      ::grpc::Service::MarkMethodCallback(1,
          new ::grpc::internal::CallbackServerStreamingHandler< ::dtproto::std_msgs::LatencyCheckRequest, ::dtproto::std_msgs::LatencyCheckPayload>(
            [this](
                   ::grpc::CallbackServerContext* context, const ::dtproto::std_msgs::LatencyCheckRequest* request) { return this->CheckLatency(context, request); }));
    }
    ~WithCallbackMethod_CheckLatency() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status CheckLatency(::grpc::ServerContext* /*context*/, const ::dtproto::std_msgs::LatencyCheckRequest* /*request*/, ::grpc::ServerWriter< ::dtproto::std_msgs::LatencyCheckPayload>* /*writer*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    virtual ::grpc::ServerWriteReactor< ::dtproto::std_msgs::LatencyCheckPayload>* CheckLatency(
      ::grpc::CallbackServerContext* /*context*/, const ::dtproto::std_msgs::LatencyCheckRequest* /*request*/)  { return nullptr; }
  };
  typedef WithCallbackMethod_Ping<WithCallbackMethod_CheckLatency<Service > > CallbackService;
  typedef CallbackService ExperimentalCallbackService;
  template <class BaseClass>
  class WithGenericMethod_Ping : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithGenericMethod_Ping() {
      ::grpc::Service::MarkMethodGeneric(0);
    }
    ~WithGenericMethod_Ping() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status Ping(::grpc::ServerContext* /*context*/, const ::dtproto::std_msgs::Heartbeat* /*request*/, ::dtproto::std_msgs::Heartbeat* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
  };
  template <class BaseClass>
  class WithGenericMethod_CheckLatency : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithGenericMethod_CheckLatency() {
      ::grpc::Service::MarkMethodGeneric(1);
    }
    ~WithGenericMethod_CheckLatency() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status CheckLatency(::grpc::ServerContext* /*context*/, const ::dtproto::std_msgs::LatencyCheckRequest* /*request*/, ::grpc::ServerWriter< ::dtproto::std_msgs::LatencyCheckPayload>* /*writer*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
  };
  template <class BaseClass>
  class WithRawMethod_Ping : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithRawMethod_Ping() {
      ::grpc::Service::MarkMethodRaw(0);
    }
    ~WithRawMethod_Ping() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status Ping(::grpc::ServerContext* /*context*/, const ::dtproto::std_msgs::Heartbeat* /*request*/, ::dtproto::std_msgs::Heartbeat* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    void RequestPing(::grpc::ServerContext* context, ::grpc::ByteBuffer* request, ::grpc::ServerAsyncResponseWriter< ::grpc::ByteBuffer>* response, ::grpc::CompletionQueue* new_call_cq, ::grpc::ServerCompletionQueue* notification_cq, void *tag) {
      ::grpc::Service::RequestAsyncUnary(0, context, request, response, new_call_cq, notification_cq, tag);
    }
  };
  template <class BaseClass>
  class WithRawMethod_CheckLatency : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithRawMethod_CheckLatency() {
      ::grpc::Service::MarkMethodRaw(1);
    }
    ~WithRawMethod_CheckLatency() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status CheckLatency(::grpc::ServerContext* /*context*/, const ::dtproto::std_msgs::LatencyCheckRequest* /*request*/, ::grpc::ServerWriter< ::dtproto::std_msgs::LatencyCheckPayload>* /*writer*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    void RequestCheckLatency(::grpc::ServerContext* context, ::grpc::ByteBuffer* request, ::grpc::ServerAsyncWriter< ::grpc::ByteBuffer>* writer, ::grpc::CompletionQueue* new_call_cq, ::grpc::ServerCompletionQueue* notification_cq, void *tag) {
      ::grpc::Service::RequestAsyncServerStreaming(1, context, request, writer, new_call_cq, notification_cq, tag);
    }
  };
  template <class BaseClass>
  class WithRawCallbackMethod_Ping : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithRawCallbackMethod_Ping() {
      ::grpc::Service::MarkMethodRawCallback(0,
          new ::grpc::internal::CallbackUnaryHandler< ::grpc::ByteBuffer, ::grpc::ByteBuffer>(
            [this](
                   ::grpc::CallbackServerContext* context, const ::grpc::ByteBuffer* request, ::grpc::ByteBuffer* response) { return this->Ping(context, request, response); }));
    }
    ~WithRawCallbackMethod_Ping() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status Ping(::grpc::ServerContext* /*context*/, const ::dtproto::std_msgs::Heartbeat* /*request*/, ::dtproto::std_msgs::Heartbeat* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    virtual ::grpc::ServerUnaryReactor* Ping(
      ::grpc::CallbackServerContext* /*context*/, const ::grpc::ByteBuffer* /*request*/, ::grpc::ByteBuffer* /*response*/)  { return nullptr; }
  };
  template <class BaseClass>
  class WithRawCallbackMethod_CheckLatency : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithRawCallbackMethod_CheckLatency() {
      ::grpc::Service::MarkMethodRawCallback(1,
          new ::grpc::internal::CallbackServerStreamingHandler< ::grpc::ByteBuffer, ::grpc::ByteBuffer>(
            [this](
                   ::grpc::CallbackServerContext* context, const::grpc::ByteBuffer* request) { return this->CheckLatency(context, request); }));
    }
    ~WithRawCallbackMethod_CheckLatency() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status CheckLatency(::grpc::ServerContext* /*context*/, const ::dtproto::std_msgs::LatencyCheckRequest* /*request*/, ::grpc::ServerWriter< ::dtproto::std_msgs::LatencyCheckPayload>* /*writer*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    virtual ::grpc::ServerWriteReactor< ::grpc::ByteBuffer>* CheckLatency(
      ::grpc::CallbackServerContext* /*context*/, const ::grpc::ByteBuffer* /*request*/)  { return nullptr; }
  };
  template <class BaseClass>
  class WithStreamedUnaryMethod_Ping : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithStreamedUnaryMethod_Ping() {
      ::grpc::Service::MarkMethodStreamed(0,
        new ::grpc::internal::StreamedUnaryHandler<
          ::dtproto::std_msgs::Heartbeat, ::dtproto::std_msgs::Heartbeat>(
            [this](::grpc::ServerContext* context,
                   ::grpc::ServerUnaryStreamer<
                     ::dtproto::std_msgs::Heartbeat, ::dtproto::std_msgs::Heartbeat>* streamer) {
                       return this->StreamedPing(context,
                         streamer);
                  }));
    }
    ~WithStreamedUnaryMethod_Ping() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable regular version of this method
    ::grpc::Status Ping(::grpc::ServerContext* /*context*/, const ::dtproto::std_msgs::Heartbeat* /*request*/, ::dtproto::std_msgs::Heartbeat* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    // replace default version of method with streamed unary
    virtual ::grpc::Status StreamedPing(::grpc::ServerContext* context, ::grpc::ServerUnaryStreamer< ::dtproto::std_msgs::Heartbeat,::dtproto::std_msgs::Heartbeat>* server_unary_streamer) = 0;
  };
  typedef WithStreamedUnaryMethod_Ping<Service > StreamedUnaryService;
  template <class BaseClass>
  class WithSplitStreamingMethod_CheckLatency : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithSplitStreamingMethod_CheckLatency() {
      ::grpc::Service::MarkMethodStreamed(1,
        new ::grpc::internal::SplitServerStreamingHandler<
          ::dtproto::std_msgs::LatencyCheckRequest, ::dtproto::std_msgs::LatencyCheckPayload>(
            [this](::grpc::ServerContext* context,
                   ::grpc::ServerSplitStreamer<
                     ::dtproto::std_msgs::LatencyCheckRequest, ::dtproto::std_msgs::LatencyCheckPayload>* streamer) {
                       return this->StreamedCheckLatency(context,
                         streamer);
                  }));
    }
    ~WithSplitStreamingMethod_CheckLatency() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable regular version of this method
    ::grpc::Status CheckLatency(::grpc::ServerContext* /*context*/, const ::dtproto::std_msgs::LatencyCheckRequest* /*request*/, ::grpc::ServerWriter< ::dtproto::std_msgs::LatencyCheckPayload>* /*writer*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    // replace default version of method with split streamed
    virtual ::grpc::Status StreamedCheckLatency(::grpc::ServerContext* context, ::grpc::ServerSplitStreamer< ::dtproto::std_msgs::LatencyCheckRequest,::dtproto::std_msgs::LatencyCheckPayload>* server_split_streamer) = 0;
  };
  typedef WithSplitStreamingMethod_CheckLatency<Service > SplitStreamedService;
  typedef WithStreamedUnaryMethod_Ping<WithSplitStreamingMethod_CheckLatency<Service > > StreamedService;
};

}  // namespace std_msgs
}  // namespace dtproto


#endif  // GRPC_dtProto_2fstd_5fmsgs_2fHeartbeat_2eproto__INCLUDED

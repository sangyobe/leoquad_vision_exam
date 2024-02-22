// Generated by the gRPC C++ plugin.
// If you make any local change, they will be lost.
// source: QuadrupedNav.proto
#ifndef GRPC_QuadrupedNav_2eproto__INCLUDED
#define GRPC_QuadrupedNav_2eproto__INCLUDED

#include "QuadrupedNav.pb.h"

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
namespace quadruped {

class Nav final {
 public:
  static constexpr char const* service_full_name() {
    return "dtproto.quadruped.Nav";
  }
  class StubInterface {
   public:
    virtual ~StubInterface() {}
    std::unique_ptr< ::grpc::ClientReaderWriterInterface< ::dtproto::nav_msgs::OdomTimeStamped, ::dtproto::nav_msgs::OdomTimeStamped>> ExchangeOdometry(::grpc::ClientContext* context) {
      return std::unique_ptr< ::grpc::ClientReaderWriterInterface< ::dtproto::nav_msgs::OdomTimeStamped, ::dtproto::nav_msgs::OdomTimeStamped>>(ExchangeOdometryRaw(context));
    }
    std::unique_ptr< ::grpc::ClientAsyncReaderWriterInterface< ::dtproto::nav_msgs::OdomTimeStamped, ::dtproto::nav_msgs::OdomTimeStamped>> AsyncExchangeOdometry(::grpc::ClientContext* context, ::grpc::CompletionQueue* cq, void* tag) {
      return std::unique_ptr< ::grpc::ClientAsyncReaderWriterInterface< ::dtproto::nav_msgs::OdomTimeStamped, ::dtproto::nav_msgs::OdomTimeStamped>>(AsyncExchangeOdometryRaw(context, cq, tag));
    }
    std::unique_ptr< ::grpc::ClientAsyncReaderWriterInterface< ::dtproto::nav_msgs::OdomTimeStamped, ::dtproto::nav_msgs::OdomTimeStamped>> PrepareAsyncExchangeOdometry(::grpc::ClientContext* context, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncReaderWriterInterface< ::dtproto::nav_msgs::OdomTimeStamped, ::dtproto::nav_msgs::OdomTimeStamped>>(PrepareAsyncExchangeOdometryRaw(context, cq));
    }
    std::unique_ptr< ::grpc::ClientWriterInterface< ::dtproto::nav_msgs::SteppableAreaTimeStamped>> StreamSteppableArea(::grpc::ClientContext* context, ::dtproto::std_msgs::Response* response) {
      return std::unique_ptr< ::grpc::ClientWriterInterface< ::dtproto::nav_msgs::SteppableAreaTimeStamped>>(StreamSteppableAreaRaw(context, response));
    }
    std::unique_ptr< ::grpc::ClientAsyncWriterInterface< ::dtproto::nav_msgs::SteppableAreaTimeStamped>> AsyncStreamSteppableArea(::grpc::ClientContext* context, ::dtproto::std_msgs::Response* response, ::grpc::CompletionQueue* cq, void* tag) {
      return std::unique_ptr< ::grpc::ClientAsyncWriterInterface< ::dtproto::nav_msgs::SteppableAreaTimeStamped>>(AsyncStreamSteppableAreaRaw(context, response, cq, tag));
    }
    std::unique_ptr< ::grpc::ClientAsyncWriterInterface< ::dtproto::nav_msgs::SteppableAreaTimeStamped>> PrepareAsyncStreamSteppableArea(::grpc::ClientContext* context, ::dtproto::std_msgs::Response* response, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncWriterInterface< ::dtproto::nav_msgs::SteppableAreaTimeStamped>>(PrepareAsyncStreamSteppableAreaRaw(context, response, cq));
    }
    class async_interface {
     public:
      virtual ~async_interface() {}
      virtual void ExchangeOdometry(::grpc::ClientContext* context, ::grpc::ClientBidiReactor< ::dtproto::nav_msgs::OdomTimeStamped,::dtproto::nav_msgs::OdomTimeStamped>* reactor) = 0;
      virtual void StreamSteppableArea(::grpc::ClientContext* context, ::dtproto::std_msgs::Response* response, ::grpc::ClientWriteReactor< ::dtproto::nav_msgs::SteppableAreaTimeStamped>* reactor) = 0;
    };
    typedef class async_interface experimental_async_interface;
    virtual class async_interface* async() { return nullptr; }
    class async_interface* experimental_async() { return async(); }
   private:
    virtual ::grpc::ClientReaderWriterInterface< ::dtproto::nav_msgs::OdomTimeStamped, ::dtproto::nav_msgs::OdomTimeStamped>* ExchangeOdometryRaw(::grpc::ClientContext* context) = 0;
    virtual ::grpc::ClientAsyncReaderWriterInterface< ::dtproto::nav_msgs::OdomTimeStamped, ::dtproto::nav_msgs::OdomTimeStamped>* AsyncExchangeOdometryRaw(::grpc::ClientContext* context, ::grpc::CompletionQueue* cq, void* tag) = 0;
    virtual ::grpc::ClientAsyncReaderWriterInterface< ::dtproto::nav_msgs::OdomTimeStamped, ::dtproto::nav_msgs::OdomTimeStamped>* PrepareAsyncExchangeOdometryRaw(::grpc::ClientContext* context, ::grpc::CompletionQueue* cq) = 0;
    virtual ::grpc::ClientWriterInterface< ::dtproto::nav_msgs::SteppableAreaTimeStamped>* StreamSteppableAreaRaw(::grpc::ClientContext* context, ::dtproto::std_msgs::Response* response) = 0;
    virtual ::grpc::ClientAsyncWriterInterface< ::dtproto::nav_msgs::SteppableAreaTimeStamped>* AsyncStreamSteppableAreaRaw(::grpc::ClientContext* context, ::dtproto::std_msgs::Response* response, ::grpc::CompletionQueue* cq, void* tag) = 0;
    virtual ::grpc::ClientAsyncWriterInterface< ::dtproto::nav_msgs::SteppableAreaTimeStamped>* PrepareAsyncStreamSteppableAreaRaw(::grpc::ClientContext* context, ::dtproto::std_msgs::Response* response, ::grpc::CompletionQueue* cq) = 0;
  };
  class Stub final : public StubInterface {
   public:
    Stub(const std::shared_ptr< ::grpc::ChannelInterface>& channel, const ::grpc::StubOptions& options = ::grpc::StubOptions());
    std::unique_ptr< ::grpc::ClientReaderWriter< ::dtproto::nav_msgs::OdomTimeStamped, ::dtproto::nav_msgs::OdomTimeStamped>> ExchangeOdometry(::grpc::ClientContext* context) {
      return std::unique_ptr< ::grpc::ClientReaderWriter< ::dtproto::nav_msgs::OdomTimeStamped, ::dtproto::nav_msgs::OdomTimeStamped>>(ExchangeOdometryRaw(context));
    }
    std::unique_ptr<  ::grpc::ClientAsyncReaderWriter< ::dtproto::nav_msgs::OdomTimeStamped, ::dtproto::nav_msgs::OdomTimeStamped>> AsyncExchangeOdometry(::grpc::ClientContext* context, ::grpc::CompletionQueue* cq, void* tag) {
      return std::unique_ptr< ::grpc::ClientAsyncReaderWriter< ::dtproto::nav_msgs::OdomTimeStamped, ::dtproto::nav_msgs::OdomTimeStamped>>(AsyncExchangeOdometryRaw(context, cq, tag));
    }
    std::unique_ptr<  ::grpc::ClientAsyncReaderWriter< ::dtproto::nav_msgs::OdomTimeStamped, ::dtproto::nav_msgs::OdomTimeStamped>> PrepareAsyncExchangeOdometry(::grpc::ClientContext* context, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncReaderWriter< ::dtproto::nav_msgs::OdomTimeStamped, ::dtproto::nav_msgs::OdomTimeStamped>>(PrepareAsyncExchangeOdometryRaw(context, cq));
    }
    std::unique_ptr< ::grpc::ClientWriter< ::dtproto::nav_msgs::SteppableAreaTimeStamped>> StreamSteppableArea(::grpc::ClientContext* context, ::dtproto::std_msgs::Response* response) {
      return std::unique_ptr< ::grpc::ClientWriter< ::dtproto::nav_msgs::SteppableAreaTimeStamped>>(StreamSteppableAreaRaw(context, response));
    }
    std::unique_ptr< ::grpc::ClientAsyncWriter< ::dtproto::nav_msgs::SteppableAreaTimeStamped>> AsyncStreamSteppableArea(::grpc::ClientContext* context, ::dtproto::std_msgs::Response* response, ::grpc::CompletionQueue* cq, void* tag) {
      return std::unique_ptr< ::grpc::ClientAsyncWriter< ::dtproto::nav_msgs::SteppableAreaTimeStamped>>(AsyncStreamSteppableAreaRaw(context, response, cq, tag));
    }
    std::unique_ptr< ::grpc::ClientAsyncWriter< ::dtproto::nav_msgs::SteppableAreaTimeStamped>> PrepareAsyncStreamSteppableArea(::grpc::ClientContext* context, ::dtproto::std_msgs::Response* response, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncWriter< ::dtproto::nav_msgs::SteppableAreaTimeStamped>>(PrepareAsyncStreamSteppableAreaRaw(context, response, cq));
    }
    class async final :
      public StubInterface::async_interface {
     public:
      void ExchangeOdometry(::grpc::ClientContext* context, ::grpc::ClientBidiReactor< ::dtproto::nav_msgs::OdomTimeStamped,::dtproto::nav_msgs::OdomTimeStamped>* reactor) override;
      void StreamSteppableArea(::grpc::ClientContext* context, ::dtproto::std_msgs::Response* response, ::grpc::ClientWriteReactor< ::dtproto::nav_msgs::SteppableAreaTimeStamped>* reactor) override;
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
    ::grpc::ClientReaderWriter< ::dtproto::nav_msgs::OdomTimeStamped, ::dtproto::nav_msgs::OdomTimeStamped>* ExchangeOdometryRaw(::grpc::ClientContext* context) override;
    ::grpc::ClientAsyncReaderWriter< ::dtproto::nav_msgs::OdomTimeStamped, ::dtproto::nav_msgs::OdomTimeStamped>* AsyncExchangeOdometryRaw(::grpc::ClientContext* context, ::grpc::CompletionQueue* cq, void* tag) override;
    ::grpc::ClientAsyncReaderWriter< ::dtproto::nav_msgs::OdomTimeStamped, ::dtproto::nav_msgs::OdomTimeStamped>* PrepareAsyncExchangeOdometryRaw(::grpc::ClientContext* context, ::grpc::CompletionQueue* cq) override;
    ::grpc::ClientWriter< ::dtproto::nav_msgs::SteppableAreaTimeStamped>* StreamSteppableAreaRaw(::grpc::ClientContext* context, ::dtproto::std_msgs::Response* response) override;
    ::grpc::ClientAsyncWriter< ::dtproto::nav_msgs::SteppableAreaTimeStamped>* AsyncStreamSteppableAreaRaw(::grpc::ClientContext* context, ::dtproto::std_msgs::Response* response, ::grpc::CompletionQueue* cq, void* tag) override;
    ::grpc::ClientAsyncWriter< ::dtproto::nav_msgs::SteppableAreaTimeStamped>* PrepareAsyncStreamSteppableAreaRaw(::grpc::ClientContext* context, ::dtproto::std_msgs::Response* response, ::grpc::CompletionQueue* cq) override;
    const ::grpc::internal::RpcMethod rpcmethod_ExchangeOdometry_;
    const ::grpc::internal::RpcMethod rpcmethod_StreamSteppableArea_;
  };
  static std::unique_ptr<Stub> NewStub(const std::shared_ptr< ::grpc::ChannelInterface>& channel, const ::grpc::StubOptions& options = ::grpc::StubOptions());

  class Service : public ::grpc::Service {
   public:
    Service();
    virtual ~Service();
    virtual ::grpc::Status ExchangeOdometry(::grpc::ServerContext* context, ::grpc::ServerReaderWriter< ::dtproto::nav_msgs::OdomTimeStamped, ::dtproto::nav_msgs::OdomTimeStamped>* stream);
    virtual ::grpc::Status StreamSteppableArea(::grpc::ServerContext* context, ::grpc::ServerReader< ::dtproto::nav_msgs::SteppableAreaTimeStamped>* reader, ::dtproto::std_msgs::Response* response);
  };
  template <class BaseClass>
  class WithAsyncMethod_ExchangeOdometry : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithAsyncMethod_ExchangeOdometry() {
      ::grpc::Service::MarkMethodAsync(0);
    }
    ~WithAsyncMethod_ExchangeOdometry() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status ExchangeOdometry(::grpc::ServerContext* /*context*/, ::grpc::ServerReaderWriter< ::dtproto::nav_msgs::OdomTimeStamped, ::dtproto::nav_msgs::OdomTimeStamped>* /*stream*/)  override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    void RequestExchangeOdometry(::grpc::ServerContext* context, ::grpc::ServerAsyncReaderWriter< ::dtproto::nav_msgs::OdomTimeStamped, ::dtproto::nav_msgs::OdomTimeStamped>* stream, ::grpc::CompletionQueue* new_call_cq, ::grpc::ServerCompletionQueue* notification_cq, void *tag) {
      ::grpc::Service::RequestAsyncBidiStreaming(0, context, stream, new_call_cq, notification_cq, tag);
    }
  };
  template <class BaseClass>
  class WithAsyncMethod_StreamSteppableArea : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithAsyncMethod_StreamSteppableArea() {
      ::grpc::Service::MarkMethodAsync(1);
    }
    ~WithAsyncMethod_StreamSteppableArea() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status StreamSteppableArea(::grpc::ServerContext* /*context*/, ::grpc::ServerReader< ::dtproto::nav_msgs::SteppableAreaTimeStamped>* /*reader*/, ::dtproto::std_msgs::Response* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    void RequestStreamSteppableArea(::grpc::ServerContext* context, ::grpc::ServerAsyncReader< ::dtproto::std_msgs::Response, ::dtproto::nav_msgs::SteppableAreaTimeStamped>* reader, ::grpc::CompletionQueue* new_call_cq, ::grpc::ServerCompletionQueue* notification_cq, void *tag) {
      ::grpc::Service::RequestAsyncClientStreaming(1, context, reader, new_call_cq, notification_cq, tag);
    }
  };
  typedef WithAsyncMethod_ExchangeOdometry<WithAsyncMethod_StreamSteppableArea<Service > > AsyncService;
  template <class BaseClass>
  class WithCallbackMethod_ExchangeOdometry : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithCallbackMethod_ExchangeOdometry() {
      ::grpc::Service::MarkMethodCallback(0,
          new ::grpc::internal::CallbackBidiHandler< ::dtproto::nav_msgs::OdomTimeStamped, ::dtproto::nav_msgs::OdomTimeStamped>(
            [this](
                   ::grpc::CallbackServerContext* context) { return this->ExchangeOdometry(context); }));
    }
    ~WithCallbackMethod_ExchangeOdometry() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status ExchangeOdometry(::grpc::ServerContext* /*context*/, ::grpc::ServerReaderWriter< ::dtproto::nav_msgs::OdomTimeStamped, ::dtproto::nav_msgs::OdomTimeStamped>* /*stream*/)  override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    virtual ::grpc::ServerBidiReactor< ::dtproto::nav_msgs::OdomTimeStamped, ::dtproto::nav_msgs::OdomTimeStamped>* ExchangeOdometry(
      ::grpc::CallbackServerContext* /*context*/)
      { return nullptr; }
  };
  template <class BaseClass>
  class WithCallbackMethod_StreamSteppableArea : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithCallbackMethod_StreamSteppableArea() {
      ::grpc::Service::MarkMethodCallback(1,
          new ::grpc::internal::CallbackClientStreamingHandler< ::dtproto::nav_msgs::SteppableAreaTimeStamped, ::dtproto::std_msgs::Response>(
            [this](
                   ::grpc::CallbackServerContext* context, ::dtproto::std_msgs::Response* response) { return this->StreamSteppableArea(context, response); }));
    }
    ~WithCallbackMethod_StreamSteppableArea() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status StreamSteppableArea(::grpc::ServerContext* /*context*/, ::grpc::ServerReader< ::dtproto::nav_msgs::SteppableAreaTimeStamped>* /*reader*/, ::dtproto::std_msgs::Response* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    virtual ::grpc::ServerReadReactor< ::dtproto::nav_msgs::SteppableAreaTimeStamped>* StreamSteppableArea(
      ::grpc::CallbackServerContext* /*context*/, ::dtproto::std_msgs::Response* /*response*/)  { return nullptr; }
  };
  typedef WithCallbackMethod_ExchangeOdometry<WithCallbackMethod_StreamSteppableArea<Service > > CallbackService;
  typedef CallbackService ExperimentalCallbackService;
  template <class BaseClass>
  class WithGenericMethod_ExchangeOdometry : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithGenericMethod_ExchangeOdometry() {
      ::grpc::Service::MarkMethodGeneric(0);
    }
    ~WithGenericMethod_ExchangeOdometry() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status ExchangeOdometry(::grpc::ServerContext* /*context*/, ::grpc::ServerReaderWriter< ::dtproto::nav_msgs::OdomTimeStamped, ::dtproto::nav_msgs::OdomTimeStamped>* /*stream*/)  override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
  };
  template <class BaseClass>
  class WithGenericMethod_StreamSteppableArea : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithGenericMethod_StreamSteppableArea() {
      ::grpc::Service::MarkMethodGeneric(1);
    }
    ~WithGenericMethod_StreamSteppableArea() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status StreamSteppableArea(::grpc::ServerContext* /*context*/, ::grpc::ServerReader< ::dtproto::nav_msgs::SteppableAreaTimeStamped>* /*reader*/, ::dtproto::std_msgs::Response* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
  };
  template <class BaseClass>
  class WithRawMethod_ExchangeOdometry : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithRawMethod_ExchangeOdometry() {
      ::grpc::Service::MarkMethodRaw(0);
    }
    ~WithRawMethod_ExchangeOdometry() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status ExchangeOdometry(::grpc::ServerContext* /*context*/, ::grpc::ServerReaderWriter< ::dtproto::nav_msgs::OdomTimeStamped, ::dtproto::nav_msgs::OdomTimeStamped>* /*stream*/)  override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    void RequestExchangeOdometry(::grpc::ServerContext* context, ::grpc::ServerAsyncReaderWriter< ::grpc::ByteBuffer, ::grpc::ByteBuffer>* stream, ::grpc::CompletionQueue* new_call_cq, ::grpc::ServerCompletionQueue* notification_cq, void *tag) {
      ::grpc::Service::RequestAsyncBidiStreaming(0, context, stream, new_call_cq, notification_cq, tag);
    }
  };
  template <class BaseClass>
  class WithRawMethod_StreamSteppableArea : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithRawMethod_StreamSteppableArea() {
      ::grpc::Service::MarkMethodRaw(1);
    }
    ~WithRawMethod_StreamSteppableArea() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status StreamSteppableArea(::grpc::ServerContext* /*context*/, ::grpc::ServerReader< ::dtproto::nav_msgs::SteppableAreaTimeStamped>* /*reader*/, ::dtproto::std_msgs::Response* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    void RequestStreamSteppableArea(::grpc::ServerContext* context, ::grpc::ServerAsyncReader< ::grpc::ByteBuffer, ::grpc::ByteBuffer>* reader, ::grpc::CompletionQueue* new_call_cq, ::grpc::ServerCompletionQueue* notification_cq, void *tag) {
      ::grpc::Service::RequestAsyncClientStreaming(1, context, reader, new_call_cq, notification_cq, tag);
    }
  };
  template <class BaseClass>
  class WithRawCallbackMethod_ExchangeOdometry : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithRawCallbackMethod_ExchangeOdometry() {
      ::grpc::Service::MarkMethodRawCallback(0,
          new ::grpc::internal::CallbackBidiHandler< ::grpc::ByteBuffer, ::grpc::ByteBuffer>(
            [this](
                   ::grpc::CallbackServerContext* context) { return this->ExchangeOdometry(context); }));
    }
    ~WithRawCallbackMethod_ExchangeOdometry() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status ExchangeOdometry(::grpc::ServerContext* /*context*/, ::grpc::ServerReaderWriter< ::dtproto::nav_msgs::OdomTimeStamped, ::dtproto::nav_msgs::OdomTimeStamped>* /*stream*/)  override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    virtual ::grpc::ServerBidiReactor< ::grpc::ByteBuffer, ::grpc::ByteBuffer>* ExchangeOdometry(
      ::grpc::CallbackServerContext* /*context*/)
      { return nullptr; }
  };
  template <class BaseClass>
  class WithRawCallbackMethod_StreamSteppableArea : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithRawCallbackMethod_StreamSteppableArea() {
      ::grpc::Service::MarkMethodRawCallback(1,
          new ::grpc::internal::CallbackClientStreamingHandler< ::grpc::ByteBuffer, ::grpc::ByteBuffer>(
            [this](
                   ::grpc::CallbackServerContext* context, ::grpc::ByteBuffer* response) { return this->StreamSteppableArea(context, response); }));
    }
    ~WithRawCallbackMethod_StreamSteppableArea() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status StreamSteppableArea(::grpc::ServerContext* /*context*/, ::grpc::ServerReader< ::dtproto::nav_msgs::SteppableAreaTimeStamped>* /*reader*/, ::dtproto::std_msgs::Response* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    virtual ::grpc::ServerReadReactor< ::grpc::ByteBuffer>* StreamSteppableArea(
      ::grpc::CallbackServerContext* /*context*/, ::grpc::ByteBuffer* /*response*/)  { return nullptr; }
  };
  typedef Service StreamedUnaryService;
  typedef Service SplitStreamedService;
  typedef Service StreamedService;
};

}  // namespace quadruped
}  // namespace dtproto


#endif  // GRPC_QuadrupedNav_2eproto__INCLUDED

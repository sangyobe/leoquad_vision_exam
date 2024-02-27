// This file is part of dtCore, a C++ library for robotics software
// development.
//
// This library is commercial and cannot be redistributed, and/or modified
// WITHOUT ANY ALLOWANCE OR PERMISSION OF Hyundai Motor Company.

#ifndef __DTCORE_DTSERVICELISTENERGRPC_H__
#define __DTCORE_DTSERVICELISTENERGRPC_H__

/** \defgroup dtDAQ
 *
 */
#include <grpc/grpc.h>
#include <grpcpp/ext/proto_server_reflection_plugin.h>
#include <grpcpp/grpcpp.h>
#include <grpcpp/health_check_service_interface.h>
#include <grpcpp/security/server_credentials.h>
#include <grpcpp/server.h>
#include <grpcpp/server_builder.h>
#include <grpcpp/server_context.h>

#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <deque>

#include <dtProto/Service.grpc.pb.h>

#define USE_THREAD_PTHREAD

namespace dtCore {

/*!
* @brief dtServiceListenerGrpc class.
* @details Is serves as the main interface for RPC server.
* It owns and runs RPC event message dispatcher thread.
* To initiate a new RPC server, use AddSession() method with proper subclass of dtServiceListenerGrpc::Session template parameter.
*/
class dtServiceListenerGrpc
{ 
public:
  dtServiceListenerGrpc(std::unique_ptr<grpc::Service> service,
                        const std::string &server_address)
      : _service(std::move(service)), _server_address(server_address) {
    grpc::ServerBuilder builder;
    builder.AddListeningPort(_server_address,
                             grpc::InsecureServerCredentials());
    builder.RegisterService(_service.get());
    _cq = builder.AddCompletionQueue();
    _server = builder.BuildAndStart();
    Run();
  }

    ~dtServiceListenerGrpc() { Stop(); }

protected:
public:
  //! Run grpc message-dispatcher
  void Run() {
    _running = true;

    // rpc event "read done / write done / close(already connected)" call-back
    // by this call completion queue rpc event "new connection / close(waiting
    // for connect)" call-back by this notification completion queue
#ifdef USE_THREAD_PTHREAD
        pthread_create(
            &_rpc_thread, NULL,
            [](void *arg) -> void * {
              dtServiceListenerGrpc *client = (dtServiceListenerGrpc *)arg;

              void *tag;
              bool ok;
              while (client->_cq->Next(&tag, &ok)) {
                // GPR_ASSERT(ok);
                if (tag) {
                  if (!static_cast<dtServiceListenerGrpc::Session*>(tag)->OnCompletionEvent(ok)) {
                    static_cast<dtServiceListenerGrpc::Session *>(tag)->TryCancelCallAndShutdown();
                    client->RemoveSession(static_cast<dtServiceListenerGrpc::Session *>(tag)->GetId());
                  }
                }
              }
              return 0;
            },
            (void *)this);
#else
        _rpc_thread = std::thread([this] {
          void *tag;
          bool ok;
          while (_cq->Next(&tag, &ok)) {
            // GPR_ASSERT(ok);
            if (tag) {
              if (!static_cast<dtServiceListenerGrpc::Session *>(tag)->OnCompletionEvent(ok)) {
                static_cast<dtServiceListenerGrpc::Session *>(tag)->TryCancelCallAndShutdown();
                RemoveSession(static_cast<dtServiceListenerGrpc::Session *>(tag)->GetId());
              }
            }
          }
        });
#endif
  }

  //! Stop all pending rpc calls and close sessions
  void Stop() {
    // {
    //   std::lock_guard<std::mutex> lock(_session_list_mtx);
    //   for (auto it : _sessions) {
    //     it.second->TryCancelCallAndShutdown();
    //   }
    //   //_sessions.clear();
    // }

    _server->Shutdown();
    _cq->Shutdown();  // drain/signal all completion queue.

#ifdef USE_THREAD_PTHREAD
    void *th_join_result;
    pthread_join(_rpc_thread, &th_join_result);
#else
    _rpc_thread.join();
#endif

    // drain the queue
    void *ignoredTag = nullptr;
    bool ok = false;
    while (_cq->Next(&ignoredTag, &ok)) {}

    _running = false;
  }
  bool IsRun() { return _running.load(); }

public:
  template <typename SessionType> bool AddSession(void *udata = nullptr) {
    std::shared_ptr<SessionType> session =
        std::make_shared<SessionType>(this, _service.get(), _cq.get(), udata);
    std::lock_guard<std::mutex> lock(_session_list_mtx);
    _sessions[session->GetId()] = session;
    return true;
  }

  void RemoveSession(uint64_t session_id) {
    std::lock_guard<std::mutex> lock(_session_list_mtx);
    _sessions.erase(session_id);
  }

public:
  class Session {
  public:
    Session(dtServiceListenerGrpc *server, grpc::Service *service,
            grpc::ServerCompletionQueue *cq,
            void * /*placeholder for user data*/)
        : _server(server), _service(service), _cq(cq),
          _call_state(CallState::WAIT_CONNECT) {
      _id = AllocSessionId();
      _call_state = CallState::WAIT_CONNECT;
    }

    Session() = delete;
    virtual ~Session() = default;
    virtual bool OnCompletionEvent(bool ok) = 0;

    uint64_t GetId() { return _id; }

    void TryCancelCallAndShutdown() {
      // std::lock_guard<std::mutex> lock(_proc_mtx);
      if (_call_state != CallState::WAIT_CONNECT &&
          _call_state != CallState::WAIT_FINISH &&
          _call_state != CallState::FINISHED) {
        _ctx.TryCancel();

        std::lock_guard<std::mutex> lock(_proc_mtx);
        //_call_state = CallState::WAIT_FINISH;
        _call_state = CallState::FINISHED;
      }

      // _call_state = CallState::FINISHED;
      // _server->RemoveSession(_id);
    }

  protected:
    uint64_t _id;
    dtServiceListenerGrpc *_server;
    grpc::Service *_service;
    grpc::ServerCompletionQueue *_cq;
    grpc::ServerContext _ctx;
    std::mutex _proc_mtx;

    enum class CallState {
      WAIT_CONNECT,
      READY_TO_READ,
      WAIT_READ_DONE,
      READY_TO_WRITE,
      WAIT_WRITE_DONE,
      WAIT_FINISH,
      FINISHED,
      PEER_DISCONNECTED
    };
    CallState _call_state;

  public:
    static uint64_t AllocSessionId() {
    static std::atomic<uint64_t> _session_id_allocator{0};
      return (++_session_id_allocator);
    }
  };

  friend class Session;

protected:
  std::string _server_address;
  std::unique_ptr<grpc::Server> _server;
  std::unique_ptr<grpc::ServerCompletionQueue> _cq;
  std::unique_ptr<grpc::Service> _service;
  std::atomic<bool> _running{false};
#ifdef USE_THREAD_PTHREAD
  pthread_t _rpc_thread;
#else
  std::thread _rpc_thread;
#endif
  std::mutex _session_list_mtx;
  std::unordered_map<uint64_t, std::shared_ptr<Session>> _sessions;
};
/*! 
* @example example_grpc_service_listener.cpp
* This examples shows how to use dtServiceListenerGrpc and dtServiceListenerGrpc::Session 
* for handling remove RPC call at server side.
* @see example_grpc_service_caller.cpp
*/

} // namespace dtCore

#endif // __DTCORE_DTSERVICELISTENERGRPC_H__
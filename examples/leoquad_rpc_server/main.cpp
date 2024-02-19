#include <iostream>
#include "dtCore/src/dtDAQ/grpc/dtServiceListenerGrpc.hpp"

class OnControlCmd : public dtCore::dtServiceListenerGrpc::Session {
public:
    OnControlCmd(dtCore::dtServiceListenerGrpc* server, dtproto::dtService::AsyncService* service, grpc::ServerCompletionQueue* cq)
    : Session(server, service, cq), _responder(&_ctx) {
        //LOG(INFO) << "NEW OnControlCmd session created.";
        _status = SessionStatus::WAIT_CONNECT;
        _service->RequestCommand(&_ctx, &_request, &_responder, _cq, _cq, this);
        //LOG(INFO) << "Wait for new ControlService::Command() service call...";
    }
    ~OnControlCmd() {
        //LOG(INFO) << "OnControlCmd session deleted.";
    }
    void OnCompletionEvent() {
        //LOG(INFO) << "OnControlCmd: " << "OnCompletionEvent";
        if (_status == SessionStatus::WAIT_CONNECT) {
            //LOG(INFO) << "NEW ControlService::Command() service call.";

            _server->AddSession<OnControlCmd>();

            {
                std::lock_guard<std::mutex> lock(_proc_mtx);

                std::cout << "Recv! cmd_mode=" << _request.cmd_mode() << " , arg=" << _request.arg() << std::endl;

                _response.set_rtn(0);
                _response.set_msg("success");

                _status = SessionStatus::WAIT_FINISH;
                _responder.Finish(_response, grpc::Status::OK, this);
            }
        } 
        else if (_status == SessionStatus::WAIT_FINISH) {
            //LOG(INFO) << "Finalize ControlService::Command() service.";
            //_status = SessionStatus::FINISHED;
            _server->RemoveSession(_id);
        }
        else {
            GPR_ASSERT(false && "Invalid Session Status.");
            //LOG(ERROR) << "Invalid session status (" << static_cast<int>(_status) << ")";
        }
    }

private:
    ::dtproto::robot_msgs::ControlCmd _request;
    ::dtproto::std_msgs::Response _response;
    grpc::ServerAsyncResponseWriter<::dtproto::std_msgs::Response> _responder;
};

int main()
{
    std::cout << "hello, leoquad_rpc_server." << std::endl;

    dtCore::dtServiceListenerGrpc listener("0.0.0.0:50052");
    listener.AddSession<OnControlCmd>();


    std::atomic<bool> bRun;
    bRun.store(true);
    while (bRun.load()) {
        std::cout << "(type \'q\' to quit) >\n";
        std::string cmd;
        std::cin >> cmd;
        if (cmd == "q" || cmd == "quit") {
            bRun = false;
        }
    }

    listener.Stop();

    return 0;
}
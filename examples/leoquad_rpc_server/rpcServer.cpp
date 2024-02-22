#include "rpcServer.h"
#include "QuadrupedNav.grpc.pb.h"
#include <dtCore/src/dtLog/dtLog.h>

/////////////////////////////////////////////////////////////////////////
// OnExchangeOdometry (Rpc service call handler)
//
//     rpc ExchangeOdometry (stream dtproto.nav_msgs.OdomTimeStamped) returns (stream dtproto.nav_msgs.OdomTimeStamped);
//
class OnExchangeOdometry : public dtCore::dtServiceListenerGrpc::Session
{
    using SessionStatus = typename dtCore::dtServiceListenerGrpc::Session::SessionStatus;
    using ServiceType = dtproto::quadruped::Nav::AsyncService;

public:
    OnExchangeOdometry(dtCore::dtServiceListenerGrpc *server, grpc::Service *service, grpc::ServerCompletionQueue *cq, void *udata = nullptr)
        : dtCore::dtServiceListenerGrpc::Session(server, service, cq, udata), _responder(&_ctx)
    {
        _status = SessionStatus::WAIT_CONNECT;
        (static_cast<ServiceType *>(_service))->RequestExchangeOdometry(&(_ctx), &_responder, _cq, _cq, this);
        LOG(info) << "NEW ExchangeOdometry() service listener created.";
    }
    ~OnExchangeOdometry()
    {
    }
    void OnCompletionEvent()
    {
        LOG(info) << "OnExchangeOdometry::OnCompletionEvent";
        if (_status == SessionStatus::WAIT_CONNECT)
        {
            LOG(info) << "NEW ExchangeOdometry() service call.";
            // add another service listener
            _server->template AddSession<OnExchangeOdometry>();
            // process incomming service call
            {
                std::lock_guard<std::mutex> lock(_proc_mtx);
                _responder.Read(&_request, (void *)this);
                _status = SessionStatus::WAIT_READ_DONE;
            }
        }
        else if (_status == SessionStatus::WAIT_WRITE_DONE)
        {
            std::lock_guard<std::mutex> lock(_proc_mtx);
            _responder.Read(&_request, (void *)this);
            _status = SessionStatus::WAIT_READ_DONE;
        }
        else if (_status == SessionStatus::WAIT_READ_DONE)
        {
            std::lock_guard<std::mutex> lock(_proc_mtx);
            _response.mutable_odom()->mutable_pose()->mutable_position()->set_x(-1.0);
            _response.mutable_odom()->mutable_pose()->mutable_position()->set_y(-2.0);
            _response.mutable_odom()->mutable_pose()->mutable_position()->set_z(-3.0);
            _response.mutable_odom()->set_child_frame_id("chile frame");
            _responder.Write(_response, (void *)this);
            _status = SessionStatus::WAIT_WRITE_DONE;
        }
        else if (_status == SessionStatus::WAIT_FINISH)
        {
            LOG(info) << "Finalize ExchangeOdometry() service.";
            //_status = SessionStatus::FINISHED;
            _server->RemoveSession(_id);
        }
        else
        {
            GPR_ASSERT(false && "Invalid Session Status.");
            LOG(err) << "Invalid session status (" << static_cast<int>(_status) << ")";
        }
    }

private:
    ::dtproto::nav_msgs::OdomTimeStamped _request;
    ::dtproto::nav_msgs::OdomTimeStamped _response;
    ::grpc::ServerAsyncReaderWriter<::dtproto::nav_msgs::OdomTimeStamped, ::dtproto::nav_msgs::OdomTimeStamped> _responder;
};

/////////////////////////////////////////////////////////////////////////
// OnStreamSteppableArea (Rpc service call handler)
//
//     rpc StreamSteppableArea (stream dtproto.nav_msgs.SteppableAreaTimeStamped) returns (std_msgs.Response);
//
class OnStreamSteppableArea : public dtCore::dtServiceListenerGrpc::Session
{
    using SessionStatus = typename dtCore::dtServiceListenerGrpc::Session::SessionStatus;
    using ServiceType = dtproto::quadruped::Nav::AsyncService;

public:
    OnStreamSteppableArea(dtCore::dtServiceListenerGrpc *server, grpc::Service *service, grpc::ServerCompletionQueue *cq, void *udata = nullptr)
        : dtCore::dtServiceListenerGrpc::Session(server, service, cq, udata), _responder(&_ctx)
    {
        _status = SessionStatus::WAIT_CONNECT;
        (static_cast<ServiceType *>(_service))->RequestStreamSteppableArea(&(_ctx), &_responder, _cq, _cq, this);
        LOG(info) << "NEW StreamSteppableArea() service listener created.";
    }
    ~OnStreamSteppableArea()
    {
    }
    void OnCompletionEvent()
    {
        LOG(info) << "OnStreamSteppableArea::OnCompletionEvent";
        if (_status == SessionStatus::WAIT_CONNECT)
        {
            LOG(info) << "NEW StreamSteppableArea() service call.";
            // add another service listener
            _server->template AddSession<OnStreamSteppableArea>();
            // process incomming service call
            {
                LOG(trace) << "[0]received steppable area count(" << _request.area().steppables_count() << ")";

                std::lock_guard<std::mutex> lock(_proc_mtx);
                _responder.Read(&_request, (void *)this);
                _status = SessionStatus::WAIT_READ_DONE;
            }
        }
        else if (_status == SessionStatus::WAIT_READ_DONE)
        {
            LOG(trace) << "[1]received steppable area count(" << _request.area().steppables_count() << ")";

            std::lock_guard<std::mutex> lock(_proc_mtx);
            _responder.Read(&_request, (void *)this);
            // _status = SessionStatus::WAIT_READ_DONE;
        }
        else if (_status == SessionStatus::WAIT_FINISH)
        {
            LOG(info) << "Finalize StreamSteppableArea() service.";
            //_status = SessionStatus::FINISHED;
            _server->RemoveSession(_id);
        }
        else
        {
            GPR_ASSERT(false && "Invalid Session Status.");
            LOG(err) << "Invalid session status (" << static_cast<int>(_status) << ")";
        }
    }

private:
    ::dtproto::nav_msgs::SteppableAreaTimeStamped _request;
    ::dtproto::std_msgs::Response _response;
    ::grpc::ServerAsyncReader<::dtproto::std_msgs::Response, ::dtproto::nav_msgs::SteppableAreaTimeStamped> _responder;
};

/////////////////////////////////////////////////////////////////////////
// RpcServer implementation
//
RpcServer::RpcServer(const std::string &server_address)
    : _listener(std::make_unique<dtCore::dtServiceListenerGrpc>(std::make_unique<dtproto::quadruped::Nav::AsyncService>(), server_address))
{
    _listener->AddSession<OnExchangeOdometry>(nullptr);
    _listener->AddSession<OnStreamSteppableArea>(nullptr);
}

RpcServer::~RpcServer()
{
    _listener->Stop();
}

void RpcServer::Run()
{
}
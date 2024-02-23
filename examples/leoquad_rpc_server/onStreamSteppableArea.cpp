#include "onStreamSteppableArea.h"
#include <dtCore/src/dtLog/dtLog.h>

OnStreamSteppableArea::OnStreamSteppableArea(dtCore::dtServiceListenerGrpc *server, grpc::Service *service, grpc::ServerCompletionQueue *cq, void *udata)
    : dtCore::dtServiceListenerGrpc::Session(server, service, cq, udata), _responder(&_ctx), _robotData((RobotData*)udata)
{
    _call_state = CallState::WAIT_CONNECT;
    (static_cast<ServiceType *>(_service))->RequestStreamSteppableArea(&(_ctx), &_responder, _cq, _cq, this);
    LOG(info) << "OnStreamSteppableArea[" << _id << "] Waiting for new service call.";
}

OnStreamSteppableArea::~OnStreamSteppableArea()
{
}

bool OnStreamSteppableArea::OnCompletionEvent(bool ok)
{
    if (_call_state == CallState::WAIT_FINISH) {
        return false;
    }
    else if (ok) {

        if (_call_state == CallState::WAIT_CONNECT)
        {
            LOG(info) << "OnStreamSteppableArea[" << _id << "] NEW service call.";
            // add another service listener
            _server->template AddSession<OnStreamSteppableArea>((void*)_robotData);
            // process incomming service call
            {
                std::lock_guard<std::mutex> lock(_proc_mtx);
                _responder.Read(&_request, (void *)this);
                _call_state = CallState::WAIT_READ_DONE;
            }
        }
        else if (_call_state == CallState::WAIT_READ_DONE)
        {
            LOG(info) << "OnStreamSteppableArea[" << _id << "] Received data.";

            LOG(trace) << "OnStreamSteppableArea[" << _id << "] \tSteppable area count = " << _request.area().steppables_count();
            LOG(trace) << "OnStreamSteppableArea[" << _id << "] \tUnsteppable area count = " << _request.area().unsteppables_count();

            std::lock_guard<std::mutex> lock(_proc_mtx);
            _responder.Read(&_request, (void *)this);
            // _call_state = CallState::WAIT_READ_DONE;
        }
    }
    else {
        if (_call_state == CallState::WAIT_CONNECT) {
            LOG(err) << "OnStreamSteppableArea[" << _id << "] Session has been shut down before receiving a matching request.";
            return false;
        }
        else {
            std::lock_guard<std::mutex> lock(_proc_mtx);
            _responder.Finish(_response, grpc::Status::CANCELLED, this);
            _call_state == CallState::WAIT_FINISH;
        }
    }
    return true;
}

#include "onStreamOdometry.h"
#include <dtCore/src/dtLog/dtLog.h>

OnStreamOdometry::OnStreamOdometry(dtCore::dtServiceListenerGrpc *server, grpc::Service *service, grpc::ServerCompletionQueue *cq, void *udata)
    : dtCore::dtServiceListenerGrpc::Session(server, service, cq, udata), _responder(&_ctx), _robotData((RobotData*)udata)
{
    _call_state = CallState::WAIT_CONNECT;
    (static_cast<ServiceType *>(_service))->RequestStreamOdometry(&(_ctx), &_responder, _cq, _cq, this);
    LOG(info) << "OnStreamOdometry[" << _id << "] Waiting for new service call.";
}

OnStreamOdometry::~OnStreamOdometry()
{
}

bool OnStreamOdometry::OnCompletionEvent(bool ok)
{
    if (_call_state == CallState::WAIT_FINISH) {
        return false;
    }
    else if (ok) {

        if (_call_state == CallState::WAIT_CONNECT)
        {
            LOG(info) << "OnStreamOdometry[" << _id << "] NEW service call.";
            // add another service listener
            _server->template AddSession<OnStreamOdometry>((void*)_robotData);
            // process incomming service call
            {
                std::lock_guard<std::mutex> lock(_proc_mtx);
                _responder.Read(&_request, (void *)this);
                _call_state = CallState::WAIT_READ_DONE;
            }
        }
        else if (_call_state == CallState::WAIT_READ_DONE)
        {
            std::lock_guard<std::mutex> lock(_proc_mtx);
            _response.mutable_odom()->mutable_pose()->mutable_position()->set_x(_request.odom().pose().position().x());
            _response.mutable_odom()->mutable_pose()->mutable_position()->set_y(_request.odom().pose().position().y());
            _response.mutable_odom()->mutable_pose()->mutable_position()->set_z(_request.odom().pose().position().z());
            _response.mutable_odom()->set_child_frame_id("chile frame");
            _responder.Write(_response, (void *)this);
            _call_state = CallState::WAIT_WRITE_DONE;
        }
        else if (_call_state == CallState::WAIT_WRITE_DONE)
        {
            std::lock_guard<std::mutex> lock(_proc_mtx);
            _responder.Read(&_request, (void *)this);
            _call_state = CallState::WAIT_READ_DONE;
        }
    }
    else {
        if (_call_state == CallState::WAIT_CONNECT) {
            LOG(err) << "OnStreamOdometry[" << _id << "] Session has been shut down before receiving a matching request.";
            return false;
        }
        else {
            std::lock_guard<std::mutex> lock(_proc_mtx);
            _responder.Finish(grpc::Status::CANCELLED, this);
            _call_state = CallState::WAIT_FINISH;
        }
    }
    return true;
}
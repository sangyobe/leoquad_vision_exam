#include "onVisualOdom.h"
#include <dtCore/src/dtLog/dtLog.h>

OnVisualOdom::OnVisualOdom(dt::DAQ::ServiceListenerGrpc *server, grpc::Service *service, grpc::ServerCompletionQueue *cq, void *udata)
    : dt::DAQ::ServiceListenerGrpc::Session(server, service, cq, udata), _responder(&_ctx), _robotData((RobotData *)udata)
{
    _call_state = CallState::WAIT_CONNECT;
    (static_cast<ServiceType *>(_service))->RequestSubscribeVisualOdom(&(_ctx), &_responder, _cq, _cq, this);
    LOG(debug) << "OnVisualOdom[" << _id << "] Waiting for new service call.";
}

OnVisualOdom::~OnVisualOdom()
{
}

bool OnVisualOdom::OnCompletionEvent(bool ok)
{
    if (_call_state == CallState::FINISHED)
    {
        return true;
    }
    else if (_call_state == CallState::WAIT_FINISH)
    {
        return false;
    }
    else if (ok)
    {

        if (_call_state == CallState::WAIT_CONNECT)
        {
            LOG(debug) << "OnVisualOdom[" << _id << "] NEW service call.";
            // add another service listener
            _server->template AddSession<OnVisualOdom>((void *)_robotData);
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

            _robotData->visualOdomPos.x = _request.odom().pose().position().x();
            _robotData->visualOdomPos.y = _request.odom().pose().position().y();
            _robotData->visualOdomPos.z = _request.odom().pose().position().z();
            _robotData->visualOdomRot.x = _request.odom().pose().orientation().x();
            _robotData->visualOdomRot.y = _request.odom().pose().orientation().y();
            _robotData->visualOdomRot.z = _request.odom().pose().orientation().z();
            _robotData->visualOdomRot.w = _request.odom().pose().orientation().w();
            LOG(trace) << "OnVisualOdom[" << _id << "] \tPosition.x = " << _request.odom().pose().position().x();
            LOG(trace) << "OnVisualOdom[" << _id << "] \tPosition.y = " << _request.odom().pose().position().y();
            LOG(trace) << "OnVisualOdom[" << _id << "] \tPosition.z = " << _request.odom().pose().position().z();
            LOG(trace) << "OnVisualOdom[" << _id << "] \tOrientation.x = " << _request.odom().pose().orientation().x();
            LOG(trace) << "OnVisualOdom[" << _id << "] \tOrientation.y = " << _request.odom().pose().orientation().y();
            LOG(trace) << "OnVisualOdom[" << _id << "] \tOrientation.z = " << _request.odom().pose().orientation().z();
            LOG(trace) << "OnVisualOdom[" << _id << "] \tOrientation.w = " << _request.odom().pose().orientation().w();

            _robotData->visualOdomMsgSeq++;

            _responder.Read(&_request, (void *)this);
            _call_state = CallState::WAIT_READ_DONE;
        }
    }
    else
    {
        if (_call_state == CallState::WAIT_CONNECT)
        {
            LOG(err) << "OnVisualOdom[" << _id << "] Session has been shut down before receiving a matching request.";
            return false;
        }
        else
        {
            std::lock_guard<std::mutex> lock(_proc_mtx);
            _responder.Finish(_response, grpc::Status::CANCELLED, this);
            _call_state = CallState::WAIT_FINISH;
        }
    }
    return true;
}
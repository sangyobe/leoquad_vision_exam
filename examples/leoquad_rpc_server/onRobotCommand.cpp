#include "onRobotCommand.h"
#include <dtCore/src/dtLog/dtLog.h>

OnRobotCommand::OnRobotCommand(dt::DAQ::ServiceListenerGrpc *server, grpc::Service *service, grpc::ServerCompletionQueue *cq, void *udata)
    : dt::DAQ::ServiceListenerGrpc::Session(server, service, cq, udata), _responder(&_ctx), _robotData((RobotData *)udata)
{
    _call_state = CallState::WAIT_CONNECT;
    (static_cast<ServiceType *>(_service))->RequestSubscribeRobotCommand(&(_ctx), &_responder, _cq, _cq, this);
    LOG(debug) << "OnRobotCommand[" << _id << "] Waiting for new service call.";
}

OnRobotCommand::~OnRobotCommand()
{
}

bool OnRobotCommand::OnCompletionEvent(bool ok)
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
            LOG(debug) << "OnRobotCommand[" << _id << "] NEW service call.";
            // add another service listener
            _server->template AddSession<OnRobotCommand>((void *)_robotData);
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

            _robotData->robotCommand.x = _request.command().nav().se2_target_pose().pose().position().x();
            _robotData->robotCommand.y = _request.command().nav().se2_target_pose().pose().position().y();
            _robotData->robotCommand.th = _request.command().nav().se2_target_pose().pose().heading();
            LOG(trace) << "OnRobotCommand[" << _id << "] \tx = " << _request.command().nav().se2_target_pose().pose().position().x();
            LOG(trace) << "OnRobotCommand[" << _id << "] \ty = " << _request.command().nav().se2_target_pose().pose().position().y();
            LOG(trace) << "OnRobotCommand[" << _id << "] \tth = " << _request.command().nav().se2_target_pose().pose().heading();

            _robotData->robotCommandMsgSeq++;

            _responder.Read(&_request, (void *)this);
            _call_state = CallState::WAIT_READ_DONE;
        }
    }
    else
    {
        if (_call_state == CallState::WAIT_CONNECT)
        {
            LOG(err) << "OnRobotCommand[" << _id << "] Session has been shut down before receiving a matching request.";
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
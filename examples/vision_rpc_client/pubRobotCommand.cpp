#include "pubRobotCommand.h"
#include <dtCore/src/dtLog/dtLog.h>

using ServiceType = dtproto::quadruped::Nav;

PubRobotCommand::PubRobotCommand(ServiceType::Stub *stub, grpc::CompletionQueue *cq, void *udata)
    : dt::DAQ::ServiceCallerGrpc<ServiceType>::Call(stub, cq, udata), _robotCommand((RobotCommand *)udata)
{
    LOG(debug) << "PubRobotCommand[" << _id << "] NEW call.";
    _writer = _stub->PrepareAsyncSubscribeRobotCommand(&(this->_ctx), &_response, this->_cq);
    this->_call_state = CallState::WAIT_CONNECT;
    _writer->StartCall((void *)this);
}

PubRobotCommand::~PubRobotCommand()
{
    // LOG(debug) << "PubRobotCommand[" << _id << "] Delete call."; // Do not output log
    // here. It might be after LOG system has been destroyed.
}

bool PubRobotCommand::Publish(const RobotCommand &cmd)
{
    if (this->_call_state != CallState::READY_TO_WRITE)
        return false;

    LOG(debug) << "PubRobotCommand[" << _id << "] publish a message.";
    std::lock_guard<std::mutex> lock(this->_proc_mtx);
    {
        struct timespec tp;
        int err = clock_gettime(CLOCK_REALTIME, &tp);

        // set message header
        _request.mutable_header()->set_seq(_msgSeq++);
        _request.mutable_header()->mutable_time_stamp()->set_seconds(tp.tv_sec);
        _request.mutable_header()->mutable_time_stamp()->set_nanos(tp.tv_nsec);
        // set message body
        _request.mutable_command()->mutable_nav()->mutable_se2_target_pose()->mutable_pose()->mutable_position()->set_x(cmd.x);
        _request.mutable_command()->mutable_nav()->mutable_se2_target_pose()->mutable_pose()->mutable_position()->set_y(cmd.y);
        _request.mutable_command()->mutable_nav()->mutable_se2_target_pose()->mutable_pose()->set_heading(cmd.th);
        _request.mutable_command()->mutable_nav()->mutable_se2_target_pose()->mutable_end_time()->set_seconds(tp.tv_sec + 2);
        _request.mutable_command()->mutable_nav()->mutable_se2_target_pose()->mutable_end_time()->set_nanos(tp.tv_nsec);

        _call_state = CallState::WAIT_WRITE_DONE;
        _writer->Write(_request, (void *)this);
    }

    return true;
}

bool PubRobotCommand::OnCompletionEvent(bool ok)
{
    if (this->_call_state == CallState::WAIT_FINISH)
    {
        switch (this->_status.error_code())
        {
        case grpc::OK:
        {
            LOG(debug) << "PubRobotCommand[" << _id << "] Complete !!!";
        }
        break;

        case grpc::CANCELLED:
        {
            LOG(warn) << "PubRobotCommand[" << _id << "] Cancelled !!!";
        }
        break;

        default:
        {
            LOG(err) << "PubRobotCommand[" << _id << "] Failed !!!";
        }
        break;
        }
        return false;
    }
    else if (ok)
    {
        if (this->_call_state == CallState::WAIT_CONNECT)
        {
            LOG(debug) << "PubRobotCommand[" << _id << "] Start call.";
            std::lock_guard<std::mutex> lock(this->_proc_mtx);
            _call_state = CallState::READY_TO_WRITE;
        }
        else if (this->_call_state == CallState::WAIT_WRITE_DONE)
        {
            std::lock_guard<std::mutex> lock(_proc_mtx);
            _call_state = CallState::READY_TO_WRITE;
        }
        else if (this->_call_state == CallState::WAIT_RESPONSE)
        {
            LOG(debug) << "PubRobotCommand[" << _id << "] Get response...";
            std::lock_guard<std::mutex> lock(this->_proc_mtx);
            {
                _call_state = CallState::WAIT_FINISH;
                _writer->Finish(&(this->_status), (void *)this);
            }
        }
        else
        {
            LOG(err) << "PubRobotCommand[" << _id << "] Invalid call state (" << static_cast<int>(_call_state) << ")";
            GPR_ASSERT(false && "Invalid Call State.");
            return false;
        }
    }
    else
    {
        std::lock_guard<std::mutex> lock(this->_proc_mtx);
        {
            _call_state = CallState::WAIT_FINISH;
            _writer->Finish(&(this->_status), (void *)this);
        }
    }

    return true;
}
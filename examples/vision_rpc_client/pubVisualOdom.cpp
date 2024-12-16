#include "pubVisualOdom.h"
#include <dtCore/src/dtLog/dtLog.h>

using ServiceType = dtproto::quadruped::Nav;

PubVisualOdometry::PubVisualOdometry(ServiceType::Stub *stub, grpc::CompletionQueue *cq, void *udata)
    : dt::DAQ::ServiceCallerGrpc<ServiceType>::Call(stub, cq, udata), _odomData((Odom *)udata)
{
    LOG(debug) << "PubVisualOdometry[" << _id << "] NEW call.";
    _writer = _stub->PrepareAsyncSubscribeVisualOdom(&(this->_ctx), &_response, this->_cq);
    _writer->StartCall((void *)this);
    this->_call_state = CallState::WAIT_CONNECT;
}

PubVisualOdometry::~PubVisualOdometry()
{
    // LOG(debug) << "PubVisualOdometry[" << _id << "] Delete call."; // Do not output log
    // here. It might be after LOG system has been destroyed.
}

bool PubVisualOdometry::Publish(const Odom &odom)
{
    if (this->_call_state != CallState::READY_TO_WRITE)
        return false;

    LOG(debug) << "PubVisualOdometry[" << _id << "] publish a message.";
    std::lock_guard<std::mutex> lock(this->_proc_mtx);
    {
        _request.mutable_odom()->mutable_pose()->mutable_position()->set_x(_odomData->position.x);
        _request.mutable_odom()->mutable_pose()->mutable_position()->set_y(_odomData->position.y);
        _request.mutable_odom()->mutable_pose()->mutable_position()->set_z(_odomData->position.z);
        _request.mutable_odom()->mutable_pose()->mutable_orientation()->set_w(_odomData->orientation.w);
        _request.mutable_odom()->mutable_pose()->mutable_orientation()->set_x(_odomData->orientation.x);
        _request.mutable_odom()->mutable_pose()->mutable_orientation()->set_y(_odomData->orientation.y);
        _request.mutable_odom()->mutable_pose()->mutable_orientation()->set_z(_odomData->orientation.z);
        _call_state = CallState::WAIT_WRITE_DONE;
        _writer->Write(_request, (void *)this);
    }

    return true;
}

bool PubVisualOdometry::OnCompletionEvent(bool ok)
{
    if (this->_call_state == CallState::WAIT_FINISH)
    {
        switch (this->_status.error_code())
        {
        case grpc::OK:
        {
            LOG(debug) << "PubVisualOdometry[" << _id << "] Complete !!!";
        }
        break;

        case grpc::CANCELLED:
        {
            LOG(warn) << "PubVisualOdometry[" << _id << "] Cancelled !!!";
        }
        break;

        default:
        {
            LOG(err) << "PubVisualOdometry[" << _id << "] Failed !!!";
        }
        break;
        }
        return false;
    }
    else if (ok)
    {

        if (this->_call_state == CallState::WAIT_CONNECT)
        {
            LOG(debug) << "PubVisualOdometry[" << _id << "] Start call.";
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
            LOG(debug) << "PubVisualOdometry[" << _id << "] Get response...";
            std::lock_guard<std::mutex> lock(this->_proc_mtx);
            {
                _call_state = CallState::WAIT_FINISH;
                _writer->Finish(&(this->_status), (void *)this);
            }
        }
        else
        {
            LOG(err) << "PubVisualOdometry[" << _id << "] Invalid call state (" << static_cast<int>(_call_state) << ")";
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
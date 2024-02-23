#include "streamOdometryCall.h"
#include <dtCore/src/dtLog/dtLog.h>

StreamOdometryCall::StreamOdometryCall(ServiceType::Stub *stub, grpc::CompletionQueue *cq, void *udata)
: dtCore::dtServiceCallerGrpc<ServiceType>::Call(stub, cq, udata), _odomData((OdomData*)udata)
{
    LOG(info) << "StreamOdometryCall[" << _id << "] NEW call.";
    _responder = _stub->PrepareAsyncStreamOdometry(&(this->_ctx), this->_cq);
    _responder->StartCall((void*)this);
    this->_call_state = CallState::WAIT_CONNECT;
}

StreamOdometryCall::~StreamOdometryCall() {
    // LOG(info) << "StreamOdometryCall[" << _id << "] Delete call."; // Do not output log
    // here. It might be after LOG system has been destroyed.
}

bool StreamOdometryCall::OnCompletionEvent(bool ok) {
    if (this->_call_state == CallState::WAIT_FINISH) {
        switch (this->_status.error_code()) {
            case grpc::OK:
            {
                LOG(info) << "StreamOdometryCall[" << _id << "] Complete !!!";
            }
            break;

            case grpc::CANCELLED:
            {
                LOG(warn) << "StreamOdometryCall[" << _id << "] Cancelled !!!";
            }
            break;

            default:
            {
                LOG(err) << "StreamOdometryCall[" << _id << "] Failed !!!";
            }
            break;
        }
        return false;
    } 
    else if (ok) {

        if (this->_call_state == CallState::WAIT_CONNECT) {
            LOG(info) << "StreamOdometryCall[" << _id << "] Start call.";
            {
                std::lock_guard<std::mutex> lock(this->_proc_mtx);

                _request.mutable_odom()->mutable_pose()->mutable_position()->set_x(1.0);
                _request.mutable_odom()->mutable_pose()->mutable_position()->set_y(2.0);
                _request.mutable_odom()->mutable_pose()->mutable_position()->set_z(3.0);
                _responder->Write(_request, (void*)this);
                _call_state = CallState::WAIT_WRITE_DONE;

                _msg_count++;
            }
        }
        else if (this->_call_state == CallState::WAIT_WRITE_DONE) {
            std::lock_guard<std::mutex> lock(_proc_mtx);
            _responder->Read(&_response, (void *)this);
            _call_state = CallState::WAIT_READ_DONE;
        }
        else if (this->_call_state == CallState::WAIT_READ_DONE) {
            {
                std::lock_guard<std::mutex> lock(this->_proc_mtx);

                if (_msg_count > 1000) {
                    grpc::WriteOptions writeOptions;
                    writeOptions.set_last_message();

                    _request.mutable_odom()->mutable_pose()->mutable_position()->set_x(0.0);
                    _request.mutable_odom()->mutable_pose()->mutable_position()->set_y(0.0);
                    _request.mutable_odom()->mutable_pose()->mutable_position()->set_z(0.0);
                    _responder->Write(_request, writeOptions, (void*)this);
                    _call_state = CallState::WAIT_RESPONSE;
                }
                else {
                    _request.mutable_odom()->mutable_pose()->mutable_position()->set_x(1.0);
                    _request.mutable_odom()->mutable_pose()->mutable_position()->set_y(2.0);
                    _request.mutable_odom()->mutable_pose()->mutable_position()->set_z(3.0);
                    _responder->Write(_request, (void*)this);
                    _call_state = CallState::WAIT_WRITE_DONE;
                }
            }
        }
        else if (this->_call_state == CallState::WAIT_RESPONSE) {
            LOG(info) << "StreamOdometryCall[" << _id << "] Get response...";
            {
                std::lock_guard<std::mutex> lock(this->_proc_mtx);
                _responder->Finish(&(this->_status), (void*)this);
                _call_state = CallState::WAIT_FINISH;
            }
        }
        else {
            GPR_ASSERT(false && "Invalid Call State.");
            LOG(err) << "StreamOdometryCall[" << _id << "] Invalid call state (" << static_cast<int>(_call_state) << ")";
            return false;
        }
    }
    else {
        std::lock_guard<std::mutex> lock(this->_proc_mtx);
        _responder->Finish(&(this->_status), (void*)this);
        _call_state = CallState::WAIT_FINISH;
    }

    return true;
}
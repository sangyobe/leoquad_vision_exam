#include "streamSteppableAreaCall.h"
#include <dtCore/src/dtLog/dtLog.h>

StreamSteppableAreaCall::StreamSteppableAreaCall(ServiceType::Stub *stub,
                                                 grpc::CompletionQueue *cq,
                                                 void *udata)
    : dtCore::dtServiceCallerGrpc<ServiceType>::Call(stub, cq, udata),
      _steppableArea((SteppableArea *)udata)
{
    LOG(debug) << "StreamSteppableAreaCall[" << _id << "] NEW call.";
    _responder = _stub->PrepareAsyncStreamSteppableArea(&(this->_ctx), &_response, this->_cq);
    _responder->StartCall((void *)this);
    this->_call_state = CallState::WAIT_CONNECT;
}

StreamSteppableAreaCall::~StreamSteppableAreaCall()
{
    // LOG(debug) << "StreamSteppableAreaCall[" << _id << "] Delete call."; // Do
    // not output log here. It might be after LOG system has been destroyed.
}

bool StreamSteppableAreaCall::OnCompletionEvent(bool ok)
{
    if (this->_call_state == CallState::WAIT_FINISH)
    {
        switch (this->_status.error_code())
        {
        case grpc::OK:
        {
            LOG(debug) << "StreamSteppableAreaCall[" << _id << "] Complete !!!";
        }
        break;

        case grpc::CANCELLED:
        {
            LOG(warn) << "StreamSteppableAreaCall[" << _id << "] Cancelled !!!";
        }
        break;

        default:
        {
            LOG(err) << "StreamSteppableAreaCall[" << _id << "] Failed !!!";
        }
        break;
        }
        return false;
    }
    else if (ok)
    {
        if (this->_call_state == CallState::WAIT_CONNECT)
        {
            LOG(debug) << "StreamSteppableAreaCall[" << _id << "] Start call.";
            {
                std::lock_guard<std::mutex> lock(this->_proc_mtx);

                _request.mutable_area()->set_steppables_count(_steppable_count % 100);
                _steppable_count++;

                _responder->Write(_request, (void *)this);
                _call_state = CallState::WAIT_WRITE_DONE;
            }
        }
        else if (this->_call_state == CallState::WAIT_WRITE_DONE)
        {
            LOG(debug) << "StreamSteppableAreaCall[" << _id
                       << "] Write done! Write another...(" << _steppable_count << ")";
            {
                std::lock_guard<std::mutex> lock(this->_proc_mtx);

                if (_steppable_count > 1000)
                {
                    grpc::WriteOptions writeOptions;
                    writeOptions.set_last_message();

                    _responder->Write(_request, writeOptions, (void *)this);
                    _call_state = CallState::WAIT_RESPONSE;
                }
                else
                {
                    _request.mutable_area()->set_steppables_count(_steppable_count % 100);
                    _steppable_count++;

                    _responder->Write(_request, (void *)this);
                    // _call_state = CallState::WAIT_WRITE_DONE;
                }
            }
        }
        else if (this->_call_state == CallState::WAIT_RESPONSE)
        {
            LOG(debug) << "StreamSteppableAreaCall[" << _id << "] Get response...";
            {
                std::lock_guard<std::mutex> lock(this->_proc_mtx);
                _responder->Finish(&(this->_status), (void *)this);
                _call_state = CallState::WAIT_FINISH;
            }
        }
        else
        {
            GPR_ASSERT(false && "Invalid Call State.");
            LOG(err) << "StreamSteppableAreaCall[" << _id << "] Invalid call state ("
                     << static_cast<int>(_call_state) << ")";
            return false;
        }
    }
    else
    {
        std::lock_guard<std::mutex> lock(this->_proc_mtx);
        _responder->Finish(&(this->_status), (void *)this);
        _call_state = CallState::WAIT_FINISH;
    }

    return true;
}
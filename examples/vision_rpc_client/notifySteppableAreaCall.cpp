#include "notifySteppableAreaCall.h"
#include <dtCore/src/dtLog/dtLog.h>

// #define MAX_VERTEX (128)
// #define MAX_POLYGON (16)

NotifySteppableAreaCall::NotifySteppableAreaCall(ServiceType::Stub *stub, grpc::CompletionQueue *cq, void *udata)
: dtCore::dtServiceCallerGrpc<ServiceType>::Call(stub, cq, udata), _steppables((SteppableArea*)udata) 
{
    for (int i=0; i<5; i++) {
        _request.mutable_area()->add_steppables();
        _request.mutable_area()->add_unsteppables();
        for (int j=0; j<10; j++) {
            _request.mutable_area()->mutable_steppables(i)->add_vertex();
            _request.mutable_area()->mutable_unsteppables(i)->add_vertex();
        }
    }

    LOG(info) << "NotifySteppableAreaCall[" << _id << "] NEW call.";
    _request.mutable_area()->set_steppables_count(99);
    _request.mutable_area()->set_unsteppables_count(1);
    _responder = _stub->PrepareAsyncNotifySteppableArea(&(this->_ctx), _request, this->_cq);
    _responder->StartCall();
    _responder->Finish(&_response, &(this->_status), (void*)this);
    this->_call_state = CallState::WAIT_FINISH;
    LOG(info) << "NotifySteppableAreaCall[" << _id << "] Wait for response.";
}

NotifySteppableAreaCall::~NotifySteppableAreaCall() {
    // LOG(info) << "NotifySteppableAreaCall[" << _id << "] Delete call."; // Do not output log
    // here. It might be after LOG system has been destroyed.
}

bool NotifySteppableAreaCall::OnCompletionEvent(bool ok) {
    if (this->_call_state == CallState::FINISHED) {
        switch (this->_status.error_code()) {
            case grpc::OK:
            {
                LOG(info) << "NotifySteppableAreaCall[" << _id << "] Complete !!!";
            }
            break;

            case grpc::CANCELLED:
            {
                LOG(warn) << "NotifySteppableAreaCall[" << _id << "] Cancelled !!!";
            }
            break;

            default:
            {
                LOG(err) << "NotifySteppableAreaCall[" << _id << "] Failed !!!";
            }
            break;
        }
        return false;
    } 
    else if (ok) {

        if (this->_call_state == CallState::WAIT_FINISH) {
            LOG(info) << "NotifySteppableAreaCall[" << _id << "] Get response.";
            {
                std::lock_guard<std::mutex> lock(this->_proc_mtx);

                LOG(trace) << "NotifySteppableAreaCall[" << this->_id << "]\treturn code : " << _response.rtn();
                LOG(trace) << "NotifySteppableAreaCall[" << this->_id << "]\tmessage : " << _response.msg();

                _call_state = CallState::FINISHED;
            }
        }
        else {
            GPR_ASSERT(false && "Invalid Call State.");
            LOG(err) << "NotifySteppableAreaCall[" << _id << "] Invalid call state (" << static_cast<int>(_call_state) << ")";
            return false;
        }
    }

    return false; // remove this call
}
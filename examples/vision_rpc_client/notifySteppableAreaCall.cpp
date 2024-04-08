#include "notifySteppableAreaCall.h"
#include <dtCore/src/dtLog/dtLog.h>

// #define MAX_VERTEX (128)
// #define MAX_POLYGON (16)

uint32_t NotifySteppableAreaCall::_req_seq = 0;

NotifySteppableAreaCall::NotifySteppableAreaCall(ServiceType::Stub *stub, grpc::CompletionQueue *cq, void *udata)
: dtCore::dtServiceCallerGrpc<ServiceType>::Call(stub, cq, udata), _steppables((SteppableArea*)udata) 
{
    _request.mutable_header()->set_seq(_req_seq++);
#ifdef _WIN32
    FILETIME ft;
    GetSystemTimeAsFileTime(&ft);
    UINT64 ticks = (((UINT64)ft.dwHighDateTime) << 32) | ft.dwLowDateTime;
    // A Windows tick is 100 nanoseconds. Windows epoch 1601-01-01T00:00:00Z
    // is 11644473600 seconds before Unix epoch 1970-01-01T00:00:00Z.
    _request.mutable_header()->mutable_time_stamp()->set_seconds(
        (INT64)((ticks / 10000000) - 11644473600LL));
    _request.mutable_header()->mutable_time_stamp()->set_nanos((INT32)((ticks % 10000000) * 100));
#else
    struct timeval tv;
    gettimeofday(&tv, NULL);
    _request.mutable_header()->mutable_time_stamp()->set_seconds(tv.tv_sec);
    _request.mutable_header()->mutable_time_stamp()->set_nanos(tv.tv_usec * 1000);
#endif

    for (int i=0; i<_steppables->steppableAreaCount; i++) {
        _request.mutable_area()->add_steppables();

        _request.mutable_area()->mutable_steppables(i)->mutable_center()->set_x(_steppables->steppableArea[i].center.x);
        _request.mutable_area()->mutable_steppables(i)->mutable_center()->set_y(_steppables->steppableArea[i].center.y);
        _request.mutable_area()->mutable_steppables(i)->mutable_center()->set_z(_steppables->steppableArea[i].center.z);
        
        _request.mutable_area()->mutable_steppables(i)->mutable_normal()->set_x(_steppables->steppableArea[i].normal.x);
        _request.mutable_area()->mutable_steppables(i)->mutable_normal()->set_y(_steppables->steppableArea[i].normal.y);
        _request.mutable_area()->mutable_steppables(i)->mutable_normal()->set_z(_steppables->steppableArea[i].normal.z);
        
        for (int j=0; j<_steppables->steppableArea[i].vertex_count; j++) {
            _request.mutable_area()->mutable_steppables(i)->add_vertex();

            _request.mutable_area()->mutable_steppables(i)->mutable_vertex(j)->set_x(_steppables->steppableArea[i].vertex[j].x);
            _request.mutable_area()->mutable_steppables(i)->mutable_vertex(j)->set_y(_steppables->steppableArea[i].vertex[j].y);
            _request.mutable_area()->mutable_steppables(i)->mutable_vertex(j)->set_z(_steppables->steppableArea[i].vertex[j].z);
        }
        _request.mutable_area()->mutable_steppables(i)->set_vertex_count(_steppables->steppableArea[i].vertex_count);
    }
    _request.mutable_area()->set_steppables_count(_steppables->steppableAreaCount);

    for (int i=0; i<_steppables->unsteppableAreaCount; i++) {
        _request.mutable_area()->add_unsteppables();

        _request.mutable_area()->mutable_unsteppables(i)->mutable_center()->set_x(_steppables->unsteppableArea[i].center.x);
        _request.mutable_area()->mutable_unsteppables(i)->mutable_center()->set_y(_steppables->unsteppableArea[i].center.y);
        _request.mutable_area()->mutable_unsteppables(i)->mutable_center()->set_z(_steppables->unsteppableArea[i].center.z);
        
        _request.mutable_area()->mutable_unsteppables(i)->mutable_normal()->set_x(_steppables->unsteppableArea[i].normal.x);
        _request.mutable_area()->mutable_unsteppables(i)->mutable_normal()->set_y(_steppables->unsteppableArea[i].normal.y);
        _request.mutable_area()->mutable_unsteppables(i)->mutable_normal()->set_z(_steppables->unsteppableArea[i].normal.z);

        for (int j=0; j<_steppables->unsteppableArea[i].vertex_count; j++) {
            _request.mutable_area()->mutable_unsteppables(i)->add_vertex();

            _request.mutable_area()->mutable_unsteppables(i)->mutable_vertex(j)->set_x(_steppables->unsteppableArea[i].vertex[j].x);
            _request.mutable_area()->mutable_unsteppables(i)->mutable_vertex(j)->set_y(_steppables->unsteppableArea[i].vertex[j].y);
            _request.mutable_area()->mutable_unsteppables(i)->mutable_vertex(j)->set_z(_steppables->unsteppableArea[i].vertex[j].z);
        }
        _request.mutable_area()->mutable_unsteppables(i)->set_vertex_count(_steppables->unsteppableArea[i].vertex_count);
    }
    _request.mutable_area()->set_unsteppables_count(_steppables->unsteppableAreaCount);

    LOG(info) << "NotifySteppableAreaCall[" << _id << "] NEW call.";

    this->_call_state = CallState::WAIT_FINISH;
    _responder = _stub->PrepareAsyncNotifySteppableArea(&(this->_ctx), _request, this->_cq);
    _responder->StartCall();
    _responder->Finish(&_response, &(this->_status), (void*)this);
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
            if (this->_status.ok()) {  // when the server's response message and status have been received.
                LOG(info) << "NotifySteppableAreaCall[" << _id << "] Get response.";
                {
                    std::lock_guard<std::mutex> lock(this->_proc_mtx);

                    LOG(trace) << "NotifySteppableAreaCall[" << this->_id << "]\treturn code : " << _response.rtn();
                    LOG(trace) << "NotifySteppableAreaCall[" << this->_id << "]\tmessage : " << _response.msg();

                    _call_state = CallState::FINISHED;
                }
            }
            else { // when the server has returned a non-OK status (no message expected in this case).
                   // when the call failed for some reason and the library generated a non-OK status. 
                LOG(err) << "NotifySteppableAreaCall[" << this->_id << "] failed.\terror code : " << this->_status.error_code();
            }
        }
        else {
            LOG(err) << "NotifySteppableAreaCall[" << _id << "] Invalid call state (" << static_cast<int>(_call_state) << ")";
            GPR_ASSERT(false && "Invalid Call State.");
            return false;
        }
    }

    return false; // remove this call
}
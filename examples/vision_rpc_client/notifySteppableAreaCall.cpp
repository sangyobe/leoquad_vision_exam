#include "notifySteppableAreaCall.h"
#include <dtCore/src/dtLog/dtLog.h>

// #define MAX_VERTEX (128)
// #define MAX_POLYGON (16)

uint32_t NotifySteppableAreaCall::_req_seq = 0;

NotifySteppableAreaCall::NotifySteppableAreaCall(ServiceType::Stub *stub, grpc::CompletionQueue *cq, void *udata)
    : dtCore::dtServiceCallerGrpc<ServiceType>::Call(stub, cq, udata), _steppableArea((SteppableArea *)udata)
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

    for (int i = 0; i < _steppableArea->steppablesCount; i++)
    {
        _request.mutable_area()->add_steppables();

        _request.mutable_area()->mutable_steppables(i)->set_index(_steppableArea->steppables[i].index);
        _request.mutable_area()->mutable_steppables(i)->set_properties(_steppableArea->steppables[i].properties);

        _request.mutable_area()->mutable_steppables(i)->mutable_polygon()->mutable_center()->set_x(_steppableArea->steppables[i].polygon.center.x);
        _request.mutable_area()->mutable_steppables(i)->mutable_polygon()->mutable_center()->set_y(_steppableArea->steppables[i].polygon.center.y);
        _request.mutable_area()->mutable_steppables(i)->mutable_polygon()->mutable_center()->set_z(_steppableArea->steppables[i].polygon.center.z);

        _request.mutable_area()->mutable_steppables(i)->mutable_polygon()->mutable_normal()->set_x(_steppableArea->steppables[i].polygon.normal.x);
        _request.mutable_area()->mutable_steppables(i)->mutable_polygon()->mutable_normal()->set_y(_steppableArea->steppables[i].polygon.normal.y);
        _request.mutable_area()->mutable_steppables(i)->mutable_polygon()->mutable_normal()->set_z(_steppableArea->steppables[i].polygon.normal.z);

        for (int j = 0; j < _steppableArea->steppables[i].polygon.vertex_count; j++)
        {
            _request.mutable_area()->mutable_steppables(i)->mutable_polygon()->add_vertex();

            _request.mutable_area()->mutable_steppables(i)->mutable_polygon()->mutable_vertex(j)->set_x(_steppableArea->steppables[i].polygon.vertex[j].x);
            _request.mutable_area()->mutable_steppables(i)->mutable_polygon()->mutable_vertex(j)->set_y(_steppableArea->steppables[i].polygon.vertex[j].y);
            _request.mutable_area()->mutable_steppables(i)->mutable_polygon()->mutable_vertex(j)->set_z(_steppableArea->steppables[i].polygon.vertex[j].z);
        }
        _request.mutable_area()->mutable_steppables(i)->mutable_polygon()->set_vertex_count(_steppableArea->steppables[i].polygon.vertex_count);
    }
    _request.mutable_area()->set_steppables_count(_steppableArea->steppablesCount);

    LOG(debug) << "NotifySteppableAreaCall[" << _id << "] NEW call.";

    this->_call_state = CallState::WAIT_FINISH;
    _responder = _stub->PrepareAsyncNotifySteppableArea(&(this->_ctx), _request, this->_cq);
    _responder->StartCall();
    _responder->Finish(&_response, &(this->_status), (void*)this);
    LOG(debug) << "NotifySteppableAreaCall[" << _id << "] Wait for response.";
}

NotifySteppableAreaCall::~NotifySteppableAreaCall() {
    // LOG(debug) << "NotifySteppableAreaCall[" << _id << "] Delete call."; // Do not output log
    // here. It might be after LOG system has been destroyed.
}

bool NotifySteppableAreaCall::OnCompletionEvent(bool ok) {
    if (this->_call_state == CallState::FINISHED) {
        switch (this->_status.error_code()) {
            case grpc::OK:
            {
                LOG(debug) << "NotifySteppableAreaCall[" << _id << "] Complete !!!";
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
                LOG(debug) << "NotifySteppableAreaCall[" << _id << "] Get response.";
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
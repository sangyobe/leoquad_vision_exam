#include "requestOdometryCall.h"
#include <dtCore/src/dtLog/dtLog.h>

RequestOdometryCall::RequestOdometryCall(ServiceType::Stub *stub, grpc::CompletionQueue *cq, void *udata)
: dtCore::dtServiceCallerGrpc<ServiceType>::Call(stub, cq, udata), _odomData((OdomData*)udata) 
{
    LOG(info) << "RequestOdometryCall[" << _id << "] NEW call.";
    _request.mutable_odom()->mutable_pose()->mutable_position()->set_x(_odomData->position.x);
    _request.mutable_odom()->mutable_pose()->mutable_position()->set_y(_odomData->position.y);
    _request.mutable_odom()->mutable_pose()->mutable_position()->set_z(_odomData->position.z);
    _request.mutable_odom()->mutable_pose()->mutable_orientation()->set_x(_odomData->orientation.x);
    _request.mutable_odom()->mutable_pose()->mutable_orientation()->set_y(_odomData->orientation.y);
    _request.mutable_odom()->mutable_pose()->mutable_orientation()->set_z(_odomData->orientation.z);
    _request.mutable_odom()->mutable_pose()->mutable_orientation()->set_w(_odomData->orientation.w);

    this->_call_state = CallState::WAIT_FINISH;
    _responder = _stub->PrepareAsyncRequestOdometry(&(this->_ctx), _request, this->_cq);
    _responder->StartCall();
    _responder->Finish(&_response, &(this->_status), (void*)this);
    LOG(info) << "RequestOdometryCall[" << _id << "] Wait for response.";
}

RequestOdometryCall::~RequestOdometryCall() {
    // LOG(info) << "RequestOdometryCall[" << _id << "] Delete call."; // Do not output log
    // here. It might be after LOG system has been destroyed.
}

bool RequestOdometryCall::OnCompletionEvent(bool ok) {
    if (this->_call_state == CallState::FINISHED) {
        switch (this->_status.error_code()) {
            case grpc::OK:
            {
                LOG(info) << "RequestOdometryCall[" << _id << "] Complete !!!";
            }
            break;

            case grpc::CANCELLED:
            {
                LOG(warn) << "RequestOdometryCall[" << _id << "] Cancelled !!!";
            }
            break;

            default:
            {
                LOG(err) << "RequestOdometryCall[" << _id << "] Failed !!!";
            }
            break;
        }
        return false;
    } 
    else if (ok) {

        if (this->_call_state == CallState::WAIT_FINISH) {
            if (this->_status.ok()) {  // when the server's response message and status have been received.

                LOG(info) << "RequestOdometryCall[" << _id << "] Get response.";
                {
                    LOG(trace).format("RequestOdometryCall[{:d}]\tposition=({:+6.3f},{:+6.3f},{:+6.3f})",
                                      this->_id,
                                      _response.odom().pose().position().x(),
                                      _response.odom().pose().position().y(),
                                      _response.odom().pose().position().z());
                    LOG(trace).format("RequestOdometryCall[{:d}]\torientation=({:+6.3f},{:+6.3f},{:+6.3f},{:+6.3f})",
                                      this->_id,
                                      _response.odom().pose().orientation().x(),
                                      _response.odom().pose().orientation().y(),
                                      _response.odom().pose().orientation().z(),
                                      _response.odom().pose().orientation().w());
                    for (int i=0; i<12 && i<_response.joint_pos_size(); i++) {
                        LOG(trace).format("RequestOdometryCall[{:d}]\tjoint_pos[{:d}]=({:+5.2f} / {:+5.2f})",
                                          this->_id,
                                          i,
                                          _response.joint_pos(i),
                                          _response.joint_pos(i) * 180.0 / 3.141592);
                    }
                    for (int i=0; i<4 && i<_response.foot_pos_size(); i++) {
                        LOG(trace).format("RequestOdometryCall[{:d}]\tfoot_pos[{:d}]=({:+6.3f},{:+6.3f},{:+6.3f})",
                                          this->_id,
                                          i,
                                          _response.foot_pos(i).x(),
                                          _response.foot_pos(i).y(),
                                          _response.foot_pos(i).z());
                    }

                    std::lock_guard<std::mutex> lock(this->_proc_mtx);
                    _call_state = CallState::FINISHED;
                }

            }
            else { // when the server has returned a non-OK status (no message expected in this case).
                   // when the call failed for some reason and the library generated a non-OK status. 
                LOG(err) << "RequestOdometryCall[" << this->_id << "] failed.\terror code : " << this->_status.error_code();
            }
        }
        else {
            LOG(err) << "RequestOdometryCall[" << _id << "] Invalid call state (" << static_cast<int>(_call_state) << ")";
            GPR_ASSERT(false && "Invalid Call State.");
            return false;
        }
    }

    return false; // remove this call
}
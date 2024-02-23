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
    _responder = _stub->PrepareAsyncRequestOdometry(&(this->_ctx), _request, this->_cq);
    _responder->StartCall();
    _responder->Finish(&_response, &(this->_status), (void*)this);
    this->_call_state = CallState::WAIT_FINISH;
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
            LOG(info) << "RequestOdometryCall[" << _id << "] Get response.";
            {
                LOG(trace) << "RequestOdometryCall[" << this->_id << "]\tposition.x : " << _response.odom().pose().position().x();
                LOG(trace) << "RequestOdometryCall[" << this->_id << "]\tposition.y : " << _response.odom().pose().position().y();
                LOG(trace) << "RequestOdometryCall[" << this->_id << "]\tposition.z : " << _response.odom().pose().position().z();
                LOG(trace) << "RequestOdometryCall[" << this->_id << "]\torientation.x : " << _response.odom().pose().orientation().x();
                LOG(trace) << "RequestOdometryCall[" << this->_id << "]\torientation.y : " << _response.odom().pose().orientation().y();
                LOG(trace) << "RequestOdometryCall[" << this->_id << "]\torientation.z : " << _response.odom().pose().orientation().z();
                LOG(trace) << "RequestOdometryCall[" << this->_id << "]\torientation.w : " << _response.odom().pose().orientation().w();
                for (int j=0; j<12; j++) {
                    LOG(trace) << "RequestOdometryCall[" << this->_id << "]\tjoint_pos[" << j << "] : " << _response.joint_pos(j);
                }

                std::lock_guard<std::mutex> lock(this->_proc_mtx);
                _call_state = CallState::FINISHED;
            }
        }
        else {
            GPR_ASSERT(false && "Invalid Call State.");
            LOG(err) << "RequestOdometryCall[" << _id << "] Invalid call state (" << static_cast<int>(_call_state) << ")";
            return false;
        }
    }

    return false; // remove this call
}
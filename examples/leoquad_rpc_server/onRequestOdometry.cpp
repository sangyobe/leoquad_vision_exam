// #include "onRequestOdometry.h"
// #include <dtCore/src/dtLog/dtLog.h>

// OnRequestOdometry::OnRequestOdometry(dt::DAQ::ServiceListenerGrpc* server, grpc::Service* service, grpc::ServerCompletionQueue* cq, void* udata)
// : dt::DAQ::ServiceListenerGrpc::Session(server, service, cq, udata), _responder(&_ctx), _robotData((RobotData*)udata) {

//     for (int j=0; j<12; j++)
//         _response.add_joint_pos(0.0);
//     for (int l=0; l<4; l++)
//         _response.add_foot_pos();

//     _call_state = CallState::WAIT_CONNECT;
//     (static_cast<ServiceType*>(_service))->RequestRequestOdometry(&(_ctx), &_request, &_responder, _cq, _cq, this);
//     LOG(debug) << "OnRequestOdometry[" << _id << "] Wait for new service call...";
// }
// OnRequestOdometry::~OnRequestOdometry() {
//     // LOG(debug) << "OnRequestOdometry session deleted."; // Do not output log here. It might be after LOG system has been destroyed.
// }
// bool OnRequestOdometry::OnCompletionEvent(bool ok) {
//     if (_call_state == CallState::FINISHED) {
//         return false;
//     }
//     else if (ok) {
//         if (_call_state == CallState::WAIT_CONNECT) {
//             LOG(debug) << "OnRequestOdometry[" << _id << "] NEW service call.";

//             _server->template AddSession<OnRequestOdometry>((void*)_robotData);
//             {
//                 std::lock_guard<std::mutex> lock(_proc_mtx);

//                 _robotData->visualOdomPos.x = _request.odom().pose().position().x();
//                 _robotData->visualOdomPos.y = _request.odom().pose().position().y();
//                 _robotData->visualOdomPos.z = _request.odom().pose().position().z();
//                 _robotData->visualOdomRot.x = _request.odom().pose().orientation().x();
//                 _robotData->visualOdomRot.y = _request.odom().pose().orientation().y();
//                 _robotData->visualOdomRot.z = _request.odom().pose().orientation().z();
//                 _robotData->visualOdomRot.w = _request.odom().pose().orientation().w();
//                 LOG(trace) << "OnRequestOdometry[" << _id << "] \tPosition.x = " << _request.odom().pose().position().x();
//                 LOG(trace) << "OnRequestOdometry[" << _id << "] \tPosition.y = " << _request.odom().pose().position().y();
//                 LOG(trace) << "OnRequestOdometry[" << _id << "] \tPosition.z = " << _request.odom().pose().position().z();
//                 LOG(trace) << "OnRequestOdometry[" << _id << "] \tOrientation.x = " << _request.odom().pose().orientation().x();
//                 LOG(trace) << "OnRequestOdometry[" << _id << "] \tOrientation.y = " << _request.odom().pose().orientation().y();
//                 LOG(trace) << "OnRequestOdometry[" << _id << "] \tOrientation.z = " << _request.odom().pose().orientation().z();
//                 LOG(trace) << "OnRequestOdometry[" << _id << "] \tOrientation.w = " << _request.odom().pose().orientation().w();

//                 _response.mutable_odom()->mutable_pose()->mutable_position()->set_x(_robotData->basePos.x);
//                 _response.mutable_odom()->mutable_pose()->mutable_position()->set_y(_robotData->basePos.y);
//                 _response.mutable_odom()->mutable_pose()->mutable_position()->set_z(_robotData->basePos.z);
//                 _response.mutable_odom()->mutable_pose()->mutable_orientation()->set_x(_robotData->baseRot.x);
//                 _response.mutable_odom()->mutable_pose()->mutable_orientation()->set_y(_robotData->baseRot.y);
//                 _response.mutable_odom()->mutable_pose()->mutable_orientation()->set_z(_robotData->baseRot.z);
//                 _response.mutable_odom()->mutable_pose()->mutable_orientation()->set_w(_robotData->baseRot.w);
//                 for (int j=0; j<12; j++)
//                     _response.set_joint_pos(j, _robotData->jointPos[j]);
//                 for (int i=0; i<4; i++) {
//                     _response.mutable_foot_pos(i)->set_x(_robotData->footPos[i].x);
//                     _response.mutable_foot_pos(i)->set_y(_robotData->footPos[i].y);
//                     _response.mutable_foot_pos(i)->set_z(_robotData->footPos[i].z);
//                 }
//                 _response.mutable_contact()->set_a1(_robotData->footContact[0]);
//                 _response.mutable_contact()->set_a2(_robotData->footContact[1]);
//                 _response.mutable_contact()->set_a3(_robotData->footContact[2]);
//                 _response.mutable_contact()->set_a4(_robotData->footContact[3]);

//                 _call_state = CallState::WAIT_FINISH;
//                 _responder.Finish(_response, grpc::Status::OK, this);
//             }
//         }
//         else if (_call_state == CallState::WAIT_FINISH) {
//             LOG(debug) << "OnRequestOdometry[" << _id << "] Finalize service.";
//             // _call_state = CallState::FINISHED;
//             // _server->RemoveSession(_id);
//             return false;
//         }
//         else {
//             LOG(err) << "OnRequestOdometry[" << _id << "] Invalid session status (" << static_cast<int>(_call_state) << ")";
//             GPR_ASSERT(false && "Invalid Session Status.");
//             return false;
//         }
//     }
//     else {
//         if (_call_state == CallState::WAIT_CONNECT) {
//             LOG(err) << "OnRequestOdometry[" << _id << "] Session has been shut down before receiving a matching request.";
//             return false;
//         }
//         else {
//             std::lock_guard<std::mutex> lock(_proc_mtx);
//             _responder.Finish(_response, grpc::Status::CANCELLED, this);
//             _call_state = CallState::WAIT_FINISH;
//         }
//     }
//     return true;
// }
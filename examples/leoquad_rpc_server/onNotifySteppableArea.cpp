// #include "onNotifySteppableArea.h"
// #include <dtCore/src/dtLog/dtLog.h>

// OnNotifySteppableArea::OnNotifySteppableArea(dt::DAQ::ServiceListenerGrpc* server, grpc::Service* service, grpc::ServerCompletionQueue* cq, void* udata)
// : dt::DAQ::ServiceListenerGrpc::Session(server, service, cq, udata), _responder(&_ctx), _robotData((RobotData*)udata) {
//     _call_state = CallState::WAIT_CONNECT;
//     (static_cast<ServiceType*>(_service))->RequestNotifySteppableArea(&(_ctx), &_request, &_responder, _cq, _cq, this);
//     LOG(debug) << "OnNotifySteppableArea[" << _id << "] Wait for new service call...";
// }
// OnNotifySteppableArea::~OnNotifySteppableArea() {
//     // LOG(debug) << "OnNotifySteppableArea session deleted."; // Do not output log here. It might be after LOG system has been destroyed.
// }
// bool OnNotifySteppableArea::OnCompletionEvent(bool ok) {
//     if (_call_state == CallState::FINISHED) {
//         return false;
//     }
//     else if (ok) {
//         if (_call_state == CallState::WAIT_CONNECT) {
//             LOG(debug) << "OnNotifySteppableArea[" << _id << "] NEW service call.";

//             _server->template AddSession<OnNotifySteppableArea>((void*)_robotData);
//             {
//                 std::lock_guard<std::mutex> lock(_proc_mtx);

//                 LOG(trace) << "OnNotifySteppableArea[" << _id << "] \tSteppable area count = " << _request.area().steppables_count();

//                 _response.set_rtn(0);
//                 _response.set_msg("success");

//                 _call_state = CallState::WAIT_FINISH;
//                 _responder.Finish(_response, grpc::Status::OK, this);
//             }
//         }
//         else if (_call_state == CallState::WAIT_FINISH) {
//             LOG(debug) << "OnNotifySteppableArea[" << _id << "] Finalize service.";
//             // _call_state = CallState::FINISHED;
//             // _server->RemoveSession(_id);
//             return false;
//         }
//         else {
//             LOG(err) << "OnNotifySteppableArea[" << _id << "] Invalid session status (" << static_cast<int>(_call_state) << ")";
//             GPR_ASSERT(false && "Invalid Session Status.");
//             return false;
//         }
//     }
//     else {
//         if (_call_state == CallState::WAIT_CONNECT) {
//             LOG(err) << "OnNotifySteppableArea[" << _id << "] Session has been shut down before receiving a matching request.";
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
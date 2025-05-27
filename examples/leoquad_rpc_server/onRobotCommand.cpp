#include "onRobotCommand.h"
#include <dtCore/src/dtLog/dtLog.h>

OnRobotCommand::OnRobotCommand(dt::DAQ::ServiceListenerGrpc *server, grpc::Service *service, grpc::ServerCompletionQueue *cq, void *udata)
    : dt::DAQ::ServiceListenerGrpc::Session(server, service, cq, udata), _responder(&_ctx), _robotData((RobotData *)udata)
{
    _call_state = CallState::WAIT_CONNECT;
    (static_cast<ServiceType *>(_service))->RequestSubscribeRobotCommand(&(_ctx), &_responder, _cq, _cq, this);
    LOG(debug) << "OnRobotCommand[" << _id << "] Waiting for new service call.";
}

OnRobotCommand::~OnRobotCommand()
{
}

bool OnRobotCommand::OnCompletionEvent(bool ok)
{
    if (_call_state == CallState::FINISHED)
    {
        return true;
    }
    else if (_call_state == CallState::WAIT_FINISH)
    {
        return false;
    }
    else if (ok)
    {

        if (_call_state == CallState::WAIT_CONNECT)
        {
            LOG(debug) << "OnRobotCommand[" << _id << "] NEW service call.";
            // add another service listener
            _server->template AddSession<OnRobotCommand>((void *)_robotData);
            // process incomming service call
            {
                std::lock_guard<std::mutex> lock(_proc_mtx);
                _responder.Read(&_request, (void *)this);
                _call_state = CallState::WAIT_READ_DONE;
            }
        }
        else if (_call_state == CallState::WAIT_READ_DONE)
        {
            std::lock_guard<std::mutex> lock(_proc_mtx);

#ifdef _WIN32
            FILETIME ft;
            GetSystemTimeAsFileTime(&ft);
            UINT64 ticks = (((UINT64)ft.dwHighDateTime) << 32) | ft.dwLowDateTime;
            _robotData->robotCommand.nav.recv_time.tv_sec = (INT64)((ticks / 10000000) - 11644473600LL);
            _robotData->robotCommand.nav.recv_time.tv_nsec = (INT32)((ticks % 10000000) * 100);
#else
            struct timespec tp;
            int err = clock_gettime(CLOCK_REALTIME, &tp);
            if (err)
                LOG(err).format("clock_gettime returns error({:})", err);
            _robotData->robotCommand.nav.recv_time.tv_sec = tp.tv_sec;
            _robotData->robotCommand.nav.recv_time.tv_nsec = tp.tv_nsec;
#endif

            if (_request.command().nav().has_se2_target_vel())
            {
                _robotData->robotCommand.nav.cmd_type = RobotCommand::NavCommand::NavCommandType_SE2Traj;
                _robotData->robotCommand.nav.end_time.tv_sec = _request.command().nav().se2_target_vel().end_time().seconds();
                _robotData->robotCommand.nav.end_time.tv_nsec = _request.command().nav().se2_target_vel().end_time().nanos();
                _robotData->robotCommand.nav.vel.vx = _request.command().nav().se2_target_vel().vel().linear().x();
                _robotData->robotCommand.nav.vel.vy = _request.command().nav().se2_target_vel().vel().linear().y();
                _robotData->robotCommand.nav.vel.w = _request.command().nav().se2_target_vel().vel().angular();
                LOG(trace) << "OnRobotCommand[" << _id << "] \tvx = " << _robotData->robotCommand.nav.vel.vx;
                LOG(trace) << "OnRobotCommand[" << _id << "] \tvy = " << _robotData->robotCommand.nav.vel.vy;
                LOG(trace) << "OnRobotCommand[" << _id << "] \tw  = " << _robotData->robotCommand.nav.vel.w;
            }
            else if (_request.command().nav().has_se2_target_pose())
            {
                _robotData->robotCommand.nav.cmd_type = RobotCommand::NavCommand::NavCommandType_SE2Pose;
                _robotData->robotCommand.nav.end_time.tv_sec = _request.command().nav().se2_target_pose().end_time().seconds();
                _robotData->robotCommand.nav.end_time.tv_nsec = _request.command().nav().se2_target_pose().end_time().nanos();
                _robotData->robotCommand.nav.pose.x = _request.command().nav().se2_target_pose().pose().position().x();
                _robotData->robotCommand.nav.pose.y = _request.command().nav().se2_target_pose().pose().position().y();
                _robotData->robotCommand.nav.pose.th = _request.command().nav().se2_target_pose().pose().heading();
                LOG(trace) << "OnRobotCommand[" << _id << "] \tx = " << _robotData->robotCommand.nav.pose.x;
                LOG(trace) << "OnRobotCommand[" << _id << "] \ty = " << _robotData->robotCommand.nav.pose.y;
                LOG(trace) << "OnRobotCommand[" << _id << "] \tth = " << _robotData->robotCommand.nav.pose.th;
            }
            else if (_request.command().nav().has_se2_target_traj())
            {
                _robotData->robotCommand.nav.cmd_type = RobotCommand::NavCommand::NavCommandType_SE2Vel;
                _robotData->robotCommand.nav.end_time.tv_sec = _request.command().nav().se2_target_traj().end_time().seconds();
                _robotData->robotCommand.nav.end_time.tv_nsec = _request.command().nav().se2_target_traj().end_time().nanos();

                int nop = _request.command().nav().se2_target_traj().trajectory().points().size();
                if (nop > NAVCOMMAND_TRAJ_MAX) nop = NAVCOMMAND_TRAJ_MAX;
                for (int i = 0; i < nop; i++)
                {
                    _robotData->robotCommand.nav.traj.points[i].pose.x = _request.command().nav().se2_target_traj().trajectory().points(i).pose().position().x();
                    _robotData->robotCommand.nav.traj.points[i].pose.y = _request.command().nav().se2_target_traj().trajectory().points(i).pose().position().y();
                    _robotData->robotCommand.nav.traj.points[i].pose.th = _request.command().nav().se2_target_traj().trajectory().points(i).pose().heading();
                    _robotData->robotCommand.nav.traj.points[i].duration = _request.command().nav().se2_target_traj().trajectory().points(i).duration();
                }
                _robotData->robotCommand.nav.traj.nop = nop;
                _robotData->robotCommand.nav.traj.ref_time.tv_sec = _request.command().nav().se2_target_traj().trajectory().reference_time().seconds();
                _robotData->robotCommand.nav.traj.ref_time.tv_nsec = _request.command().nav().se2_target_traj().trajectory().reference_time().nanos();

                if (nop > 0)
                {
                    LOG(trace) << "OnRobotCommand[" << _id << "] \tx[" << nop - 1 << "] = " << _robotData->robotCommand.nav.traj.points[nop - 1].pose.x;
                    LOG(trace) << "OnRobotCommand[" << _id << "] \ty[" << nop - 1 << "] = " << _robotData->robotCommand.nav.traj.points[nop - 1].pose.y;
                    LOG(trace) << "OnRobotCommand[" << _id << "] \tth[" << nop - 1 << "] = " << _robotData->robotCommand.nav.traj.points[nop - 1].pose.th;
                }
                else
                {
                    LOG(warn) << "OnRobotCommand[" << _id << "] \tnumber of trajectory points = " << nop;
                }
            }

            _robotData->robotCommandMsgSeq++;

            _responder.Read(&_request, (void *)this);
            _call_state = CallState::WAIT_READ_DONE;
        }
    }
    else
    {
        if (_call_state == CallState::WAIT_CONNECT)
        {
            LOG(err) << "OnRobotCommand[" << _id << "] Session has been shut down before receiving a matching request.";
            return false;
        }
        else
        {
            std::lock_guard<std::mutex> lock(_proc_mtx);
            _responder.Finish(_response, grpc::Status::CANCELLED, this);
            _call_state = CallState::WAIT_FINISH;
        }
    }
    return true;
}
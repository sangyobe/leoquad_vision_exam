#include "onLocalGridmap.h"
#include <dtCore/src/dtLog/dtLog.h>

OnLocalGridmap::OnLocalGridmap(dt::DAQ::ServiceListenerGrpc *server, grpc::Service *service, grpc::ServerCompletionQueue *cq, void *udata)
    : dt::DAQ::ServiceListenerGrpc::Session(server, service, cq, udata), _responder(&_ctx), _robotData((RobotData *)udata)
{
    _call_state = CallState::WAIT_CONNECT;
    (static_cast<ServiceType *>(_service))->RequestSubscribeLocalGridmap(&(_ctx), &_responder, _cq, _cq, this);
    LOG(debug) << "OnLocalGridmap[" << _id << "] Waiting for new service call.";
}

OnLocalGridmap::~OnLocalGridmap()
{
}

bool OnLocalGridmap::OnCompletionEvent(bool ok)
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
            LOG(debug) << "OnLocalGridmap[" << _id << "] NEW service call.";
            // add another service listener
            _server->template AddSession<OnLocalGridmap>((void *)_robotData);
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

            _robotData->gridmap.offset.x = _request.grid().pose_offset().position().x();
            _robotData->gridmap.offset.y = _request.grid().pose_offset().position().y();
            _robotData->gridmap.resolution = _request.grid().cell_size().a1();
            _robotData->gridmap.dim_x = _request.grid().grid_dim().a1();
            _robotData->gridmap.dim_y = _request.grid().grid_dim().a2();
            if (_request.grid().has_grid_center())
            {
                _robotData->gridmap.center.x = _request.grid().grid_center().position().x();
                _robotData->gridmap.center.y = _request.grid().grid_center().position().y();
            }
            else
            {
                _robotData->gridmap.center.x = (double)_robotData->gridmap.dim_x * _robotData->gridmap.resolution * 0.5 + 1e-3;
                _robotData->gridmap.center.y = (double)_robotData->gridmap.dim_y * _robotData->gridmap.resolution * 0.5 + 1e-3;
            }

            uint32_t row_count = _robotData->gridmap.dim_x;
            uint32_t col_count = _robotData->gridmap.dim_y;

            for (const dtproto::nav_msgs::Grid_Layer &layer : _request.grid().layers())
            {
                if (layer.layer_id() == "hmap")
                {
                    const double *data = (const double *)(layer.data().c_str());
                    for (int irow = 0; irow < row_count; irow++)
                    {
                        for (int icol = 0; icol < col_count; icol++)
                        {
                            _robotData->gridmap.hmap[irow][icol] = data[irow * col_count + icol];
                        }
                    }
                }
                else if (layer.layer_id() == "steppability")
                {
                    const uint8_t *data = (const uint8_t *)(layer.data().c_str());
                    for (int irow = 0; irow < row_count; irow++)
                    {
                        for (int icol = 0; icol < col_count; icol++)
                        {
                            _robotData->gridmap.steppability[irow][icol] = data[irow * col_count + icol];
                            _robotData->gridmap.costmap[irow][icol] = (_robotData->gridmap.steppability[irow][icol] < 0.5 ? 300.0 : 0.0);
                        }
                    }
                }
            }

            _robotData->gridmapMsgSeq++;

            _responder.Read(&_request, (void *)this);
            _call_state = CallState::WAIT_READ_DONE;
        }
    }
    else
    {
        if (_call_state == CallState::WAIT_CONNECT)
        {
            LOG(err) << "OnLocalGridmap[" << _id << "] Session has been shut down before receiving a matching request.";
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
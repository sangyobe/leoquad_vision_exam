#include "pubGridmap.h"
#include <dtCore/src/dtLog/dtLog.h>

PubGridmap::PubGridmap(ServiceType::Stub *stub, grpc::CompletionQueue *cq, void *udata)
    : dt::DAQ::ServiceCallerGrpc<ServiceType>::Call(stub, cq, udata), _gridmapData((Gridmap *)udata)
{
    _request.mutable_grid()->set_grid_id("gridmap");
    _request.mutable_grid()->set_frame_id("odom");
    _request.mutable_grid()->mutable_grid_dim()->set_a1(_gridmapData->dim_x);
    _request.mutable_grid()->mutable_grid_dim()->set_a2(_gridmapData->dim_y);
    _request.mutable_grid()->mutable_cell_size()->set_a1(_gridmapData->resolution);
    _request.mutable_grid()->mutable_cell_size()->set_a2(_gridmapData->resolution);
    _request.mutable_grid()->add_layers(); // height-map layer
    _request.mutable_grid()->mutable_layers(0)->set_layer_id("hmap");
    _request.mutable_grid()->mutable_layers(0)->set_cell_format(dtproto::nav_msgs::Grid_Layer_CellFormat_FLOAT64);
    _request.mutable_grid()->add_layers(); // steppability layer
    _request.mutable_grid()->mutable_layers(1)->set_layer_id("steppability");
    _request.mutable_grid()->mutable_layers(1)->set_cell_format(dtproto::nav_msgs::Grid_Layer_CellFormat_UINT8);

    LOG(debug) << "PubGridmap[" << _id << "] NEW call.";
    _writer = _stub->PrepareAsyncSubscribeLocalGridmap(&(this->_ctx), &_response, this->_cq);
    _writer->StartCall((void *)this);
    this->_call_state = CallState::WAIT_CONNECT;
}

PubGridmap::~PubGridmap()
{
    // LOG(debug) << "PubGridmap[" << _id << "] Delete call."; // Do not output log
    // here. It might be after LOG system has been destroyed.
}

bool PubGridmap::Publish(const Gridmap &gridmap)
{
    if (this->_call_state != CallState::READY_TO_WRITE)
        return false;

    LOG(debug) << "PubGridmap[" << _id << "] publish a message.";
    std::lock_guard<std::mutex> lock(this->_proc_mtx);
    {
        struct timespec tp;
        int err = clock_gettime(CLOCK_REALTIME, &tp);

        // set message header
        _request.mutable_header()->set_seq(_msgSeq++);
        _request.mutable_header()->mutable_time_stamp()->set_seconds(tp.tv_sec);
        _request.mutable_header()->mutable_time_stamp()->set_nanos(tp.tv_nsec);
        // set message body
        _request.mutable_grid()->mutable_acquisition_time()->set_seconds(tp.tv_sec);
        _request.mutable_grid()->mutable_acquisition_time()->set_nanos(tp.tv_nsec);
        _request.mutable_grid()->mutable_pose_offset()->mutable_position()->set_x(gridmap.offset.x);
        _request.mutable_grid()->mutable_pose_offset()->mutable_position()->set_y(gridmap.offset.y);
        _request.mutable_grid()->mutable_pose_offset()->mutable_position()->set_z(0.0);
        _request.mutable_grid()->mutable_grid_center()->mutable_position()->set_x(gridmap.center.x);
        _request.mutable_grid()->mutable_grid_center()->mutable_position()->set_y(gridmap.center.y);
        _request.mutable_grid()->mutable_grid_center()->mutable_position()->set_z(0.0);
        _request.mutable_grid()->mutable_layers(0)->set_data(&gridmap.hmap[0][0], gridmap.dim_x * gridmap.dim_y * sizeof(gridmap.hmap[0][0]));
        _request.mutable_grid()->mutable_layers(1)->set_data(&gridmap.steppability[0][0], gridmap.dim_x * gridmap.dim_y * sizeof(gridmap.steppability[0][0]));

        _call_state = CallState::WAIT_WRITE_DONE;
        _writer->Write(_request, (void *)this);
    }

    return true;
}

bool PubGridmap::OnCompletionEvent(bool ok)
{
    if (this->_call_state == CallState::WAIT_FINISH)
    {
        switch (this->_status.error_code())
        {
        case grpc::OK:
        {
            LOG(debug) << "PubGridmap[" << _id << "] Complete !!!";
        }
        break;

        case grpc::CANCELLED:
        {
            LOG(warn) << "PubGridmap[" << _id << "] Cancelled !!!";
        }
        break;

        default:
        {
            LOG(err) << "PubGridmap[" << _id << "] Failed !!!";
        }
        break;
        }
        return false;
    }
    else if (ok)
    {

        if (this->_call_state == CallState::WAIT_CONNECT)
        {
            LOG(debug) << "PubGridmap[" << _id << "] Start call.";
            std::lock_guard<std::mutex> lock(this->_proc_mtx);
            _call_state = CallState::READY_TO_WRITE;
        }
        else if (this->_call_state == CallState::WAIT_WRITE_DONE)
        {
            std::lock_guard<std::mutex> lock(_proc_mtx);
            _call_state = CallState::READY_TO_WRITE;
        }
        else if (this->_call_state == CallState::WAIT_RESPONSE)
        {
            LOG(debug) << "PubGridmap[" << _id << "] Get response...";
            std::lock_guard<std::mutex> lock(this->_proc_mtx);
            {
                _call_state = CallState::WAIT_FINISH;
                _writer->Finish(&(this->_status), (void *)this);
            }
        }
        else
        {
            LOG(err) << "PubGridmap[" << _id << "] Invalid call state (" << static_cast<int>(_call_state) << ")";
            GPR_ASSERT(false && "Invalid Call State.");
            return false;
        }
    }
    else
    {
        std::lock_guard<std::mutex> lock(this->_proc_mtx);
        {
            _call_state = CallState::WAIT_FINISH;
            _writer->Finish(&(this->_status), (void *)this);
        }
    }

    return true;
}
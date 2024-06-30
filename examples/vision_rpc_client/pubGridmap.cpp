#include "pubGridmap.h"
#include <dtCore/src/dtLog/dtLog.h>

PubGridmap::PubGridmap(ServiceType::Stub *stub, grpc::CompletionQueue *cq, void *udata)
    : dtCore::dtServiceCallerGrpc<ServiceType>::Call(stub, cq, udata), _gridmapData((GridmapData *)udata)
{
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

bool PubGridmap::Publish(const GridmapData &gridmap)
{
    if (this->_call_state != CallState::READY_TO_WRITE)
        return false;

    LOG(debug) << "PubGridmap[" << _id << "] publish a message.";
    std::lock_guard<std::mutex> lock(this->_proc_mtx);
    {
        _request.mutable_grid()->set_grid_id("hmap");
        _request.mutable_grid()->mutable_acquisition_time()->set_seconds(0);
        _request.mutable_grid()->mutable_acquisition_time()->set_nanos(0);
        _request.mutable_grid()->set_frame_id("odom");
        _request.mutable_grid()->mutable_pose_offset()->mutable_position()->set_x(gridmap.offset.x);
        _request.mutable_grid()->mutable_pose_offset()->mutable_position()->set_y(gridmap.offset.y);
        _request.mutable_grid()->mutable_pose_offset()->mutable_position()->set_z(0.0);
        _request.mutable_grid()->set_row_count(120);
        _request.mutable_grid()->set_col_count(120);
        _request.mutable_grid()->set_cell_stride(4);
        _request.mutable_grid()->set_cell_format(dtproto::nav_msgs::Grid_CellFormat::Grid_CellFormat_FLOAT32);
        _request.mutable_grid()->set_cell_size(0.05);
        _request.mutable_grid()->set_data(&gridmap.hmap[0][0], 120 * 120 * sizeof(gridmap.hmap[0][0]));
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
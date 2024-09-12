#include "rpcServer.h"
#include "QuadrupedNav.grpc.pb.h"
#include "onLocalGridmap.h"
#include "onVisualOdom.h"
#include "robotData.h"
#include <dtCore/src/dtLog/dtLog.h>

/////////////////////////////////////////////////////////////////////////
// RpcServer implementation
//
RpcServer::RpcServer(void *robotData)
    : _navServiceListener(std::make_unique<dt::DAQ::ServiceListenerGrpc>(
          std::make_unique<dtproto::quadruped::Nav::AsyncService>(),
          "0.0.0.0:50056")),
      _robotStatePublisher(std::make_unique<dt::DAQ::StatePublisherGrpc<
                               dtproto::robot_msgs::RobotStateTimeStamped>>(
          "RobotState", "0.0.0.0:50053", 0)),
      _gridmapPublisher(
          std::make_unique<
              dt::DAQ::StatePublisherGrpc<dtproto::nav_msgs::GridTimeStamped>>(
              "Gridmap", "0.0.0.0:50057", 0)),
      _imuPublisher(std::make_unique<dt::DAQ::StatePublisherGrpc<
                        dtproto::sensor_msgs::ImuTimeStamped>>(
          "Imu", "0.0.0.0:50054", 0)),
      _odomPublisher(std::make_unique<dt::DAQ::StatePublisherGrpc<
                         dtproto::quadruped::OdomWithJointPosTimeStamped>>(
          "Odom", "0.0.0.0:50055", 0)),
      _robotData((RobotData *)robotData)
{
    _navServiceListener->AddSession<OnVisualOdom>(robotData);
    _navServiceListener->AddSession<OnLocalGridmap>(robotData);

    // pre-allocate and initialize message structure
    _imuMsg.mutable_header()->set_frame_id("map");

    _robotStateMsg.mutable_header()->set_frame_id("base");
    for (int ji = 0; ji < 12; ji++)
    {
        _robotStateMsg.mutable_state()->add_joint_state();
    }

    _gridmapMsg.mutable_grid()->set_grid_id("gridmap");
    _gridmapMsg.mutable_grid()->set_frame_id("map");
    _gridmapMsg.mutable_grid()->mutable_grid_dim()->set_a1(
        _robotData->gridmap.dim_x);
    _gridmapMsg.mutable_grid()->mutable_grid_dim()->set_a2(
        _robotData->gridmap.dim_y);
    _gridmapMsg.mutable_grid()->add_layers(); // height-map layer
    _gridmapMsg.mutable_grid()->mutable_layers(0)->set_layer_id("hmap");
    _gridmapMsg.mutable_grid()->mutable_layers(0)->set_cell_format(
        dtproto::nav_msgs::Grid_Layer_CellFormat_FLOAT64);
    _gridmapMsg.mutable_grid()->add_layers(); // foot-step cost layer
    _gridmapMsg.mutable_grid()->mutable_layers(1)->set_layer_id("costmap");
    _gridmapMsg.mutable_grid()->mutable_layers(1)->set_cell_format(
        dtproto::nav_msgs::Grid_Layer_CellFormat_FLOAT64);

    _odomMsg.mutable_header()->set_frame_id("map");
    for (int ji = 0; ji < 12; ji++)
    {
        _odomMsg.add_joint_pos(0.0);
    }
    for (int fi = 0; fi < 4; fi++)
    {
        _odomMsg.add_foot_pos();
    }
}

RpcServer::~RpcServer() { _navServiceListener->Stop(); }

void RpcServer::Run()
{
    _dataUpdater = std::thread([this] {
        _runUpdater.store(true);
        double t_ = 0.0;
        double dt_ = 0.1;
        uint32_t prevVisualOdomMsgSeq = 0;
        uint32_t prevGridmapMsgSeq = 0;
        double prevStatTimeSec = t_;
        while (_runUpdater.load())
        {
            struct timespec tp;
            int err = clock_gettime(CLOCK_REALTIME, &tp);

            //
            // Publish RobotState
            //
            // set message header
            _robotStateMsg.mutable_header()->set_seq(0);
            _robotStateMsg.mutable_header()->mutable_time_stamp()->set_seconds(
                tp.tv_sec);
            _robotStateMsg.mutable_header()->mutable_time_stamp()->set_nanos(
                tp.tv_nsec);
            // set message body
            // robot position in {V-odom}
            _robotStateMsg.mutable_state()
                ->mutable_base_pose()
                ->mutable_position()
                ->set_x(_robotData->visualOdomPos.x);
            _robotStateMsg.mutable_state()
                ->mutable_base_pose()
                ->mutable_position()
                ->set_y(_robotData->visualOdomPos.y);
            _robotStateMsg.mutable_state()
                ->mutable_base_pose()
                ->mutable_position()
                ->set_z(_robotData->visualOdomPos.z);
            // robot orientation in {V-odom}
            _robotStateMsg.mutable_state()
                ->mutable_base_pose()
                ->mutable_orientation()
                ->set_w(_robotData->visualOdomRot.w);
            _robotStateMsg.mutable_state()
                ->mutable_base_pose()
                ->mutable_orientation()
                ->set_x(_robotData->visualOdomRot.x);
            _robotStateMsg.mutable_state()
                ->mutable_base_pose()
                ->mutable_orientation()
                ->set_y(_robotData->visualOdomRot.y);
            _robotStateMsg.mutable_state()
                ->mutable_base_pose()
                ->mutable_orientation()
                ->set_z(_robotData->visualOdomRot.z);
            // joint position
            for (int ji = 0; ji < 12; ji++)
            {
                _robotStateMsg.mutable_state()
                    ->mutable_joint_state(ji)
                    ->set_position(_robotData->jointPos[ji]);
            }
            // publish
            _robotStatePublisher->Publish(_robotStateMsg);

            //
            // Publish Gridmap
            //
            // set message header
            _gridmapMsg.mutable_header()->set_seq(0);
            _gridmapMsg.mutable_header()->mutable_time_stamp()->set_seconds(
                tp.tv_sec);
            _gridmapMsg.mutable_header()->mutable_time_stamp()->set_nanos(
                tp.tv_nsec);
            // set message body
            _gridmapMsg.mutable_grid()->mutable_acquisition_time()->set_seconds(
                tp.tv_sec);
            _gridmapMsg.mutable_grid()->mutable_acquisition_time()->set_nanos(
                tp.tv_nsec);
            _gridmapMsg.mutable_grid()->mutable_cell_size()->set_a1(
                _robotData->gridmap.resolution);
            _gridmapMsg.mutable_grid()->mutable_cell_size()->set_a2(
                _robotData->gridmap.resolution);
            _gridmapMsg.mutable_grid()
                ->mutable_pose_offset()
                ->mutable_position()
                ->set_x(_robotData->gridmap.offset.x);
            _gridmapMsg.mutable_grid()
                ->mutable_pose_offset()
                ->mutable_position()
                ->set_y(_robotData->gridmap.offset.y);
            _gridmapMsg.mutable_grid()
                ->mutable_pose_offset()
                ->mutable_position()
                ->set_z(0.0);
            _gridmapMsg.mutable_grid()
                ->mutable_grid_center()
                ->mutable_position()
                ->set_x(_robotData->gridmap.center.x);
            _gridmapMsg.mutable_grid()
                ->mutable_grid_center()
                ->mutable_position()
                ->set_y(_robotData->gridmap.center.y);
            _gridmapMsg.mutable_grid()
                ->mutable_grid_center()
                ->mutable_position()
                ->set_z(0.0);
            _gridmapMsg.mutable_grid()->mutable_layers(0)->set_data(
                &_robotData->gridmap.hmap[0][0],
                _robotData->gridmap.dim_x * _robotData->gridmap.dim_y *
                    sizeof(_robotData->gridmap.hmap[0][0]));
            _gridmapMsg.mutable_grid()->mutable_layers(1)->set_data(
                &_robotData->gridmap.costmap[0][0],
                _robotData->gridmap.dim_x * _robotData->gridmap.dim_y *
                    sizeof(_robotData->gridmap.costmap[0][0]));
            // publish
            _gridmapPublisher->Publish(_gridmapMsg);

            //
            // Publish IMU
            //
            // set message header
            _imuMsg.mutable_header()->set_seq(0);
            _imuMsg.mutable_header()->mutable_time_stamp()->set_seconds(
                tp.tv_sec);
            _imuMsg.mutable_header()->mutable_time_stamp()->set_nanos(
                tp.tv_nsec);
            // set message body
            _imuMsg.mutable_imu()->mutable_orientation()->set_w(
                _robotData->baseRot.w);
            _imuMsg.mutable_imu()->mutable_orientation()->set_x(
                _robotData->baseRot.x);
            _imuMsg.mutable_imu()->mutable_orientation()->set_y(
                _robotData->baseRot.y);
            _imuMsg.mutable_imu()->mutable_orientation()->set_z(
                _robotData->baseRot.z);
            _imuMsg.mutable_imu()->mutable_angular_velocity()->set_a1(0.0);
            _imuMsg.mutable_imu()->mutable_angular_velocity()->set_a2(0.0);
            _imuMsg.mutable_imu()->mutable_angular_velocity()->set_a3(0.0);
            _imuMsg.mutable_imu()->mutable_linear_acceleration()->set_a1(0.0);
            _imuMsg.mutable_imu()->mutable_linear_acceleration()->set_a2(0.0);
            _imuMsg.mutable_imu()->mutable_linear_acceleration()->set_a3(0.0);
            // publish
            _imuPublisher->Publish(_imuMsg);

            //
            // Publish Odom
            //
            // set message header
            _odomMsg.mutable_header()->set_seq(0);
            _odomMsg.mutable_header()->mutable_time_stamp()->set_seconds(
                tp.tv_sec);
            _odomMsg.mutable_header()->mutable_time_stamp()->set_nanos(
                tp.tv_nsec);
            // set message body
            _odomMsg.mutable_odom()->mutable_pose()->mutable_position()->set_x(
                _robotData->basePos.x);
            _odomMsg.mutable_odom()->mutable_pose()->mutable_position()->set_y(
                _robotData->basePos.y);
            _odomMsg.mutable_odom()->mutable_pose()->mutable_position()->set_z(
                _robotData->basePos.z);
            _odomMsg.mutable_odom()
                ->mutable_pose()
                ->mutable_orientation()
                ->set_w(_robotData->baseRot.w);
            _odomMsg.mutable_odom()
                ->mutable_pose()
                ->mutable_orientation()
                ->set_x(_robotData->baseRot.x);
            _odomMsg.mutable_odom()
                ->mutable_pose()
                ->mutable_orientation()
                ->set_y(_robotData->baseRot.y);
            _odomMsg.mutable_odom()
                ->mutable_pose()
                ->mutable_orientation()
                ->set_z(_robotData->baseRot.z);
            for (int ji = 0; ji < 12; ji++)
            {
                _odomMsg.set_joint_pos(ji, _robotData->jointPos[ji]);
            }
            for (int fi = 0; fi < 4; fi++)
            {
                _odomMsg.mutable_foot_pos(0)->set_x(_robotData->footPos[fi].x);
                _odomMsg.mutable_foot_pos(0)->set_y(_robotData->footPos[fi].y);
                _odomMsg.mutable_foot_pos(0)->set_z(_robotData->footPos[fi].z);
            }
            _odomMsg.mutable_contact()->set_a1(_robotData->footContact[0]);
            _odomMsg.mutable_contact()->set_a2(_robotData->footContact[1]);
            _odomMsg.mutable_contact()->set_a3(_robotData->footContact[2]);
            _odomMsg.mutable_contact()->set_a4(_robotData->footContact[3]);
            // publish
            _odomPublisher->Publish(_odomMsg);

            // Log message rate
            if (prevStatTimeSec + 1.0 < t_)
            {
                double delT = t_ - prevStatTimeSec;
                uint32_t visualOdomMsgCount =
                    _robotData->visualOdomMsgSeq - prevVisualOdomMsgSeq;
                uint32_t gridmapMsgCount =
                    _robotData->gridmapMsgSeq - prevGridmapMsgSeq;
                LOG(info) << "Odometry: " << (double)(visualOdomMsgCount) / delT
                          << "(hz)";
                LOG(info) << "Gridmap: " << (double)(gridmapMsgCount) / delT
                          << "(hz)";
                prevVisualOdomMsgSeq = _robotData->visualOdomMsgSeq;
                prevGridmapMsgSeq = _robotData->gridmapMsgSeq;
                prevStatTimeSec = t_;
            }

            // Advance time
            std::this_thread::sleep_for(
                std::chrono::milliseconds((long)(dt_ * 1000)));
            t_ += dt_;
        }
    });
}

void RpcServer::Stop()
{
    _runUpdater.store(false);
    _dataUpdater.join();

    _navServiceListener->Stop();
}
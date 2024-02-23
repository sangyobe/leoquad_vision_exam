#include "rpcServer.h"
#include "QuadrupedNav.grpc.pb.h"
#include <dtCore/src/dtLog/dtLog.h>
#include "onStreamOdometry.h"
#include "onStreamSteppableArea.h"
#include "onRequestOdometry.h"
#include "onNotifySteppableArea.h"

/////////////////////////////////////////////////////////////////////////
// RpcServer implementation
//
RpcServer::RpcServer(const std::string &server_address, void* robotData)
    : _listener(std::make_unique<dtCore::dtServiceListenerGrpc>(std::make_unique<dtproto::quadruped::Nav::AsyncService>(), server_address))
{
    _listener->AddSession<OnRequestOdometry>(robotData);
    _listener->AddSession<OnNotifySteppableArea>(robotData);
    _listener->AddSession<OnStreamOdometry>(robotData);
    _listener->AddSession<OnStreamSteppableArea>(robotData);
}

RpcServer::~RpcServer()
{
    _listener->Stop();
}

void RpcServer::Run()
{
}

void RpcServer::Stop()
{
    _listener->Stop();
}
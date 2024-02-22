#ifndef __RPCSERVER_H__
#define __RPCSERVER_H__

#include <dtCore/src/dtDAQ/grpc/dtServiceListenerGrpc.hpp>
#include <string>
#include <memory>
class RpcServer
{
public:
    RpcServer(const std::string &server_address);
    virtual ~RpcServer();

public:
    void Run();

private:
    std::unique_ptr<dtCore::dtServiceListenerGrpc> _listener{nullptr};
};

#endif // __RPCSERVER_H__
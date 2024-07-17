#ifndef __RPCSERVER_H__
#define __RPCSERVER_H__

#include <dtCore/src/dtDAQ/grpc/dtServiceListenerGrpc.hpp>
#include <string>
#include <memory>
class RpcServer
{
public:
    RpcServer(const std::string &server_address, void* robotData);
    virtual ~RpcServer();

public:
    void Run();
    void Stop();

private:
    std::unique_ptr<dt::DAQ::ServiceListenerGrpc> _listener{nullptr};
};

#endif // __RPCSERVER_H__
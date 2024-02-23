#ifndef RPCCLIENT_H
#define RPCCLIENT_H

#include <dtCore/src/dtDAQ/grpc/dtServiceCallerGrpc.hpp>
#include <string>
#include "QuadrupedNav.grpc.pb.h"

using ServiceType = dtproto::quadruped::Nav;

class RpcClient : public dtCore::dtServiceCallerGrpc<ServiceType> {
public:
    RpcClient(const std::string &server_address) 
        : dtCore::dtServiceCallerGrpc<ServiceType>(server_address) {}
};

#endif // RPCCLIENT_H
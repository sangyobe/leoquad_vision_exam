#ifndef RPCCLIENT_H
#define RPCCLIENT_H

#include <grpc/grpc.h>
#include <grpcpp/channel.h>
#include <grpcpp/client_context.h>
#include <grpcpp/create_channel.h>
#include <grpcpp/security/credentials.h>

#include <dtProto/Service.grpc.pb.h>
#include "QuadrupedNav.grpc.pb.h"

#include <memory>
#include <string>

class RpcClient {
public:
  RpcClient(std::shared_ptr<grpc::Channel> channel);

  bool ExchangeOdometry();
  bool StreamSteppableArea();
  bool ControlCmd(int cmd_mode, const char* fmt, ...);

private:
  std::shared_ptr<grpc::Channel> channel_;
  std::unique_ptr<dtproto::dtService::Stub> stub_;
};

#endif // RPCCLIENT_H
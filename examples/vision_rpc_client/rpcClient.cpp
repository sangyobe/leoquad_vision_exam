#include "rpcClient.h"

RpcClient::RpcClient(std::shared_ptr<grpc::Channel> channel)
    : channel_(channel)
    , stub_(dtproto::dtService::NewStub(channel)) 
{
}

bool RpcClient::ExchangeOdometry()
{
    return false;
}
bool RpcClient::StreamSteppableArea()
{
    return false;
}

bool RpcClient::ControlCmd(int cmd_mode, const char* fmt, ...) 
{
    grpc::ClientContext context;
    dtproto::robot_msgs::ControlCmd req;
    dtproto::std_msgs::Response res;
    
    req.set_cmd_mode(cmd_mode);
    // req.set_arg(arg);

    grpc::Status status = stub_->Command(&context, req, &res);
    if (!status.ok()) {
        std::cout << "Command rpc failed." << std::endl;
        return false;
    } else {
        std::cout << "rtn : " << res.rtn() << std::endl;
        std::cout << "msg : " << res.msg() << std::endl;
        return true;
    }
}

#ifndef __RPCSUBSCRIBER_H__
#define __RPCSUBSCRIBER_H__

#include <dtCore/src/dtDAQ/grpc/dtStateSubscriberGrpc.hpp>

template <typename MessageType>
class RpcSubscriber : public dtCore::dtStateSubscriberGrpc<MessageType>
{
public:
    RpcSubscriber(const std::string &topic_name, const std::string &server_address)
        : dtCore::dtStateSubscriberGrpc<MessageType>(topic_name, server_address) {}
};

#endif // __RPCSUBSCRIBER_H__
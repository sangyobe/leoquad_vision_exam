// This file is part of dtCore, a C++ library for robotics software
// development.
//
// This library is commercial and cannot be redistributed, and/or modified
// WITHOUT ANY ALLOWANCE OR PERMISSION OF Hyundai Motor Company.

#ifndef __DTCORE_DTDATASINKPBMCAP_H__
#define __DTCORE_DTDATASINKPBMCAP_H__

#include <dtCore/src/dtDAQ/dtDataSinkPB.hpp>
#include <dtCore/src/dtUtils/dtFileHelper.hpp>

#include <google/protobuf/descriptor.h>
#include <google/protobuf/descriptor.pb.h>
#include <string>
#include <queue>
#include <unordered_set>
#include <iostream>
#include <fstream>
#include <chrono>

#define MCAP_COMPRESSION_NO_LZ4
#define MCAP_COMPRESSION_NO_ZSTD
#define MCAP_IMPLEMENTATION
#include <mcap/mcap.hpp>

namespace dtCore {

template<typename T>
class dtDataSinkPBMcap : public dtDataSinkPB<T> {
public:
    dtDataSinkPBMcap(const std::string& topic_name, const std::string& file_basename = "", bool annot_datetime = true) 
    : _topic_name(topic_name)
    {
        std::string filename = file_basename;
        if (annot_datetime)
            filename = annotate_filename_datetime(file_basename);

        std::string dirname;
        std::tie(dirname, std::ignore) = split_by_directory(filename);
        if (!create_dir(dirname)) {
            std::cerr << "Failed to create containing directory (" << dirname << ")." << std::endl;
        }

        auto options = mcap::McapWriterOptions("");
        const auto res = _writer.open(filename, options);
        if (!res.ok()) {
            std::cerr << "Failed to open " << filename << " for writing: " << res.message << std::endl;
        }

        // add schema
        mcap::Schema schema(
            T().GetTypeName(),
            "protobuf", 
            BuildFileDescriptorSet(T::descriptor()).SerializeAsString());
        _writer.addSchema(schema);

        // add channel(topic)
        mcap::Channel channel(topic_name, "protobuf", schema.id);
        _writer.addChannel(channel);
        _channel_id = channel.id;
    }

    void Publish(T& msg) 
    {
        mcap::Timestamp publishTime = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                          std::chrono::system_clock::now().time_since_epoch()).count();

        std::string serialized = msg.SerializeAsString();
        mcap::Message mcap_msg;
        mcap_msg.channelId = _channel_id;
        mcap_msg.sequence = _msg_count++;
        mcap_msg.publishTime = publishTime;
        mcap_msg.logTime = publishTime;
        mcap_msg.data = reinterpret_cast<const std::byte*>(serialized.data());
        mcap_msg.dataSize = serialized.size();
        const auto res = _writer.write(mcap_msg);
        if (!res.ok()) {
            std::cerr << "Failed to write message: " << res.message << "\n";
            _writer.terminate();
            _writer.close();
        }
    }

protected:
    google::protobuf::FileDescriptorSet BuildFileDescriptorSet(const google::protobuf::Descriptor* toplevelDescriptor) 
    {
        google::protobuf::FileDescriptorSet fdSet;
        std::queue<const google::protobuf::FileDescriptor*> toAdd;
        toAdd.push(toplevelDescriptor->file());
        std::unordered_set<std::string> seenDependencies;
        while (!toAdd.empty()) {
            const google::protobuf::FileDescriptor* next = toAdd.front();
            toAdd.pop();
            next->CopyTo(fdSet.add_file());
            for (int i = 0; i < next->dependency_count(); ++i) {
                const auto& dep = next->dependency(i);
                if (seenDependencies.find(dep->name()) == seenDependencies.end()) {
                seenDependencies.insert(dep->name());
                toAdd.push(dep);
                }
            }
        }
        return fdSet;
    }

protected:
    std::string _topic_name;
    mcap::McapWriter _writer;
    mcap::ChannelId _channel_id;
    uint32_t _msg_count{0};
};

}

#endif // __DTCORE_DTDATASINKPBMCAP_H__
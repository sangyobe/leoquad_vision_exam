#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <chrono>
#include "rpcClient.h"
#include "requestOdometryCall.h"
#include "notifySteppableAreaCall.h"
#include "streamOdometryCall.h"
#include "streamSteppableAreaCall.h"
#include "emulOdom.h"
#include "emulSteppables.h"
#include <dtCore/src/dtLog/dtLog.h>

OdomEmulator odomEmul;
SteppablesEmulator steppablesEmul;

int main()
{
    dtCore::dtLog::Initialize("leoquad_vision_rpc_client"); //, "logs/leoquad_vision_rpc_client.txt");
    dtCore::dtLog::SetLogLevel(dtCore::dtLog::LogLevel::trace);

    std::unique_ptr<RpcClient> rpcClient = std::make_unique<RpcClient>("10.0.0.1:50056");

    std::atomic<bool> bRun{true};

    std::thread odom_caller = std::thread([&] () {
        double t_ = 0.0;
        double dt_ = 0.01;

        while (bRun.load()) {

            rpcClient->template StartCall<RequestOdometryCall>((void*)(&odomEmul.odom));
            std::this_thread::sleep_for(std::chrono::milliseconds(long(dt_ * 1000)));
            t_ += dt_;
        }
    });
    std::thread steppableArea_caller = std::thread([&] () {
        double t_ = 0.0;
        double dt_ = 0.05;

        while (bRun.load()) {

            rpcClient->template StartCall<NotifySteppableAreaCall>((void*)(&steppablesEmul.steppables));
            std::this_thread::sleep_for(std::chrono::milliseconds(long(dt_ * 1000)));
            t_ += dt_;
        }
    });
    // rpcClient->template StartCall<StreamOdometryCall>((void*)(&odomEmul.odom));
    // rpcClient->template StartCall<StreamSteppableAreaCall>((void*)(&steppablesEmul.steppables));

    while (bRun.load()) {
        std::cout << "(type \'q\' to quit) >\n";
        std::string cmd;
        std::cin >> cmd;
        if (cmd == "q" || cmd == "quit") {
            bRun = false;
        }
    }
    
    odom_caller.join();
    steppableArea_caller.join();
    dtCore::dtLog::Terminate(); // flush all log messages
    return 0;
}
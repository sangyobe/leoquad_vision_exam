#include <iostream>
#include "rpcServer.h"
#include "quadRobot.h"
#include <dtCore/src/dtLog/dtLog.h>

QuadRobot quadRobot;

int main()
{
    dtCore::dtLog::Initialize("leoquad_vision_rpc_server"); //, "logs/leoquad_vision_rpc_server.txt");
    dtCore::dtLog::SetLogLevel(dtCore::dtLog::LogLevel::trace);

    RpcServer rpcServer("0.0.0.0:50056", (void*)&(quadRobot.robotData));
    rpcServer.Run();

    std::atomic<bool> bRun;
    bRun.store(true);
    while (bRun.load()) {
        std::cout << "(type \'q\' to quit) >\n";
        std::string cmd;
        std::cin >> cmd;
        if (cmd == "q" || cmd == "quit") {
            bRun = false;
        }
    }
    rpcServer.Stop();

    dtCore::dtLog::Terminate(); // flush all log messages
    return 0;
}
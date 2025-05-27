#ifndef __EMUL_ROBOTCOMMAND_H__
#define __EMUL_ROBOTCOMMAND_H__

#include "dataTypes.h"
#include <atomic>
#include <chrono>
#include <cmath>
#include <thread>

class RobotCommandEmulator
{
public:
    RobotCommand cmd;

public:
    RobotCommandEmulator()
    {
        _dataUpdater = std::thread([this] {
            _runUpdater.store(true);
            double t_ = 0.0;
            double dt_ = 0.01;
            while (_runUpdater.load())
            {

                cmd.x += 0.1;
                cmd.y += 0.03;
                cmd.th += 0.1;
                if (cmd.x > 10.0) cmd.x -= 10.0;
                if (cmd.y > 10.0) cmd.y -= 10.0;
                if (cmd.th > 2.0 * 3.14159) cmd.th -= (2.0 * 3.14159);

                std::this_thread::sleep_for(std::chrono::milliseconds((long)(dt_ * 1000)));
                t_ += dt_;
            }
        });
    }
    ~RobotCommandEmulator()
    {
        _runUpdater.store(false);
        _dataUpdater.join();
    }

private:
    std::thread _dataUpdater;
    std::atomic<bool> _runUpdater;
};

#endif // __EMUL_ROBOTCOMMAND_H__
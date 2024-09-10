#ifndef __EMUL_ODOM_H__
#define __EMUL_ODOM_H__

#include "dataTypes.h"
#include <atomic>
#include <chrono>
#include <cmath>
#include <thread>

class OdomEmulator
{
public:
    Odom odom;
    Odom kodom;

public:
    OdomEmulator() 
    {
        _dataUpdater = std::thread([this] {
            _runUpdater.store(true);
            double t_ = 0.0;
            double dt_ = 0.001;
            while (_runUpdater.load()) {
                odom.position.x = cos(M_2_PI * 0.5 * t_);
                odom.position.y = sin(M_2_PI * 0.5 * t_);
                odom.position.z = 0.5;
                odom.orientation.fromEuler(0.0, 0.0, M_2_PI * 0.5 * t_);

                std::this_thread::sleep_for(std::chrono::milliseconds((long)(dt_ * 1000)));
                t_ += dt_;
            }
        });
    }
    ~OdomEmulator()
    {
        _runUpdater.store(false);
        _dataUpdater.join();
    }

private:
    std::thread _dataUpdater;
    std::atomic<bool> _runUpdater;
};

#endif // __EMUL_ODOM_H__
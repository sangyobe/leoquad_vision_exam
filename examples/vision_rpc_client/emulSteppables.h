#ifndef __EMUL_STEPPABLES_H__
#define __EMUL_STEPPABLES_H__

#include "dataTypes.h"
#include <thread>
#include <chrono>
#include <cmath>

typedef struct _steppableArea
{
    Polygon steppableArea[MAX_POLYGON];
    Polygon unsteppableArea[MAX_POLYGON];
    uint32_t steppableAreaCount;
    uint32_t unsteppableAreaCount;
} SteppableArea;

class SteppablesEmulator
{
public:
    SteppableArea steppables;

public:
    SteppablesEmulator() 
    {
        _dataUpdater = std::thread([this] {
            _runUpdater.store(true);
            double t_ = 0.0;
            double dt_ = 0.001;
            while (_runUpdater.load()) {
                

                std::this_thread::sleep_for(std::chrono::milliseconds((long)(dt_ * 1000)));
                t_ += dt_;
            }
        });
    }
    ~SteppablesEmulator()
    {
        _runUpdater.store(false);
        _dataUpdater.join();
    }

private:
    std::thread _dataUpdater;
    std::atomic<bool> _runUpdater;
};

#endif // __EMUL_STEPPABLES_H__
#ifndef __EMUL_GRIDMAP_H__
#define __EMUL_GRIDMAP_H__

#include "dataTypes.h"
#include <atomic>
#include <chrono>
#include <cmath>
#include <thread>

typedef struct _gridmapData
{
    float hmap[120][120];
    Point offset;
} GridmapData;

class GridmapEmulator
{
public:
    GridmapData gridmap;

public:
    GridmapEmulator()
    {
        _dataUpdater = std::thread([this] {
            _runUpdater.store(true);
            double t_ = 0.0;
            double dt_ = 0.1;
            while (_runUpdater.load())
            {
                for (int irow = 0; irow < 120; irow++)
                {
                    for (int icol = 0; icol < 120; icol++)
                    {
                        // gridmap.hmap[irow][icol] = -0.04 + 0.2 * std::sin(3.0 * t_ + 5.0 * 0.05 * irow) * 0.05 * icol;
                        gridmap.hmap[irow][icol] = 0.01 * (float)(irow + icol);
                    }
                }

                std::this_thread::sleep_for(std::chrono::milliseconds((long)(dt_ * 1000)));
                t_ += dt_;
            }
        });
    }
    ~GridmapEmulator()
    {
        _runUpdater.store(false);
        _dataUpdater.join();
    }

private:
    std::thread _dataUpdater;
    std::atomic<bool> _runUpdater;
};

#endif // __EMUL_GRIDMAP_H__
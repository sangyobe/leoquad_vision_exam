#ifndef __EMUL_GRIDMAP_H__
#define __EMUL_GRIDMAP_H__

#include "dataTypes.h"
#include <atomic>
#include <chrono>
#include <cmath>
#include <thread>

class GridmapEmulator
{
public:
    Gridmap gridmap;

public:
    GridmapEmulator()
    {
        gridmap.resolution = 0.03;
        gridmap.dim_x = 120;
        gridmap.dim_y = 120;
        gridmap.center.x = 0.0;
        gridmap.center.y = 0.0;
        gridmap.offset.x = -1.8;
        gridmap.offset.y = -1.8;

        _dataUpdater = std::thread([this] {
            _runUpdater.store(true);
            double t_ = 0.0;
            double dt_ = 0.1;
            while (_runUpdater.load())
            {
                for (int irow = 0; irow < gridmap.dim_x; irow++)
                {
                    for (int icol = 0; icol < gridmap.dim_y; icol++)
                    {
                        // gridmap.hmap[irow][icol] = -0.04 + 0.2 * std::sin(3.0 * t_ + 5.0 * 0.05 * irow) * 0.05 * icol;
                        gridmap.hmap[irow][icol] = 0.01 * (float)(irow + icol);
                        gridmap.steppability[irow][icol] = ((irow + icol) % 10 ? 1 : 0);
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
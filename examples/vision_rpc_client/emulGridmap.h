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
        gridmap.resolution = GRIDMAP_RES;
        gridmap.dim_x = GRIDMAP_DIM_X;
        gridmap.dim_y = GRIDMAP_DIM_Y;
        gridmap.center.x = (double)(int)(0.5 * GRIDMAP_RES * 1e3 * GRIDMAP_DIM_X + 0.5) * 1e-3;
        gridmap.center.y = (double)(int)(0.5 * GRIDMAP_RES * 1e3 * GRIDMAP_DIM_Y + 0.5) * 1e-3;
        gridmap.offset.x = -(0.5 * GRIDMAP_RES * GRIDMAP_DIM_X);
        gridmap.offset.y = -(0.5 * GRIDMAP_RES * GRIDMAP_DIM_Y);

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
                        gridmap.hmap[irow][icol] = -0.04 + 0.2 * std::sin(2.0 * t_ + 0.1 * irow) * std::sin(2.0 * t_ + 0.1 * icol);
                        // gridmap.hmap[irow][icol] = 0.01 * (float)(irow + icol);
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
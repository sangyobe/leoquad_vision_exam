#ifndef __EMUL_STEPPABLES_H__
#define __EMUL_STEPPABLES_H__

#include "dataTypes.h"
#include <thread>
#include <chrono>
#include <cmath>
#include "yaml-cpp/yaml.h"

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
        YAML::Node rootNode = YAML::LoadFile("examples/vision_rpc_client/data/stair_240215.yaml");
        char plg_name[64];
        uint32_t plg_count = 0;
        uint32_t vtx_count = 0;
        while (true) {
            snprintf(plg_name, 64, "polygon%d", (plg_count+1));
            if (!rootNode["polygons"][plg_name].IsDefined())
                break;

            steppables.steppableArea[plg_count].center.x = rootNode["polygons"][plg_name]["center"][0].as<double>();
            steppables.steppableArea[plg_count].center.y = rootNode["polygons"][plg_name]["center"][1].as<double>();
            steppables.steppableArea[plg_count].center.z = rootNode["polygons"][plg_name]["center"][2].as<double>();

            steppables.steppableArea[plg_count].normal.x = rootNode["polygons"][plg_name]["normal"][0].as<double>();
            steppables.steppableArea[plg_count].normal.y = rootNode["polygons"][plg_name]["normal"][1].as<double>();
            steppables.steppableArea[plg_count].normal.z = rootNode["polygons"][plg_name]["normal"][2].as<double>();

            vtx_count = rootNode["polygons"][plg_name]["points"].size();
            if (vtx_count > MAX_VERTEX)
                vtx_count = MAX_VERTEX;
            for (int i=0; i<vtx_count; i++) {
                steppables.steppableArea[plg_count].vertex[i].x = rootNode["polygons"][plg_name]["points"][i][0].as<double>();
                steppables.steppableArea[plg_count].vertex[i].y = rootNode["polygons"][plg_name]["points"][i][1].as<double>();
                steppables.steppableArea[plg_count].vertex[i].z = rootNode["polygons"][plg_name]["points"][i][2].as<double>();
            }
            steppables.steppableArea[plg_count].vertex_count = vtx_count;

            plg_count++;
            if (plg_count >= MAX_POLYGON)
                break;
        }
        steppables.steppableAreaCount = plg_count;
        steppables.unsteppableAreaCount = 0;

        _dataUpdater = std::thread([this] {
            _runUpdater.store(true);
            double t_ = 0.0;
            double dt_ = 0.001;
            while (_runUpdater.load()) {
                
                steppables.steppableAreaCount = (((int)t_) % 5) + 1;

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
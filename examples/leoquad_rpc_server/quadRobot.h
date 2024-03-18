#ifndef QUADROBOT_H
#define QUADROBOT_H

#include "robotData.h"
#include <thread>
#include <chrono>

class QuadRobot
{
public:
    RobotData robotData;

public:
    QuadRobot() 
    {
        _dataUpdater = std::thread([this] {
            _runUpdater.store(true);
            double t_ = 0.0;
            double dt_ = 0.001;
            while (_runUpdater.load()) {
                robotData.basePos.x = sin(t_);
                robotData.basePos.y = cos(t_);

                robotData.baseRot.fromEuler(0.0, 0.0, t_);

                for (int j = 0; j < 12; j++) {
                    robotData.jointPos[j] = sin(t_ + j*(1.0/12.0)) * sin(t_ + j*(1.0/12.0));
                }

                for (int i = 0; i < 4; i++) {
                    robotData.footPos[i].x = sin(2.0 * t_);
                    robotData.footPos[i].y = cos(2.0 *t_);
                    robotData.footPos[i].z = 0.4 + 0.1 * sin(2.0 *t_);
                    robotData.footContact[i] = (((int)(t_) % 2) > 0 ? true : false);
                }
                
                std::this_thread::sleep_for(std::chrono::milliseconds((long)(dt_ * 1000)));
                t_ += dt_;
            }
        });
    }
    ~QuadRobot()
    {
        _runUpdater.store(false);
        _dataUpdater.join();
    }

private:
    std::thread _dataUpdater;
    std::atomic_bool _runUpdater;
};

#endif // QUADROBOT_H
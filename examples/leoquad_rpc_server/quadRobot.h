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
                robotData.basePos.x = 0.0; // sin(t_);
                robotData.basePos.y = 0.0; // cos(t_);
                robotData.basePos.z = 0.45;
                robotData.baseRot.fromEuler(0.0, 0.0, t_);

                robotData.jointPos[0] = 0.0;
                robotData.jointPos[1] = 0.7 * sin(3.14 * t_) * sin(3.14 * t_);
                robotData.jointPos[2] = -2.0 * robotData.jointPos[1];

                robotData.jointPos[3] = 0.0;
                robotData.jointPos[4] = 0.7 * cos(3.14 * t_) * cos(3.14 * t_);
                robotData.jointPos[5] = -2.0 * robotData.jointPos[4];

                robotData.jointPos[6] = 0.0;
                robotData.jointPos[7] = 0.7 * cos(3.14 * t_) * cos(3.14 * t_);
                robotData.jointPos[8] = -2.0 * robotData.jointPos[7];

                robotData.jointPos[9] = 0.0;
                robotData.jointPos[10] = 0.7 * sin(3.14 * t_) * sin(3.14 * t_);
                robotData.jointPos[11] = -2.0 * robotData.jointPos[10];

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
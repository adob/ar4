#pragma once

#include "robot/robot.h"
#include <QList>

#include "lib/base.h"

namespace robot {
    using namespace lib;

    struct CameraConfig {
        String device_path;
    };

    struct TaskConfig {
        String dataset_dir;
        robot::Pose starting_pose;

        QList<CameraConfig> cameras;
    };
}
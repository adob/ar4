#pragma once

#include <climits>
#include <list>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <qlist.h>

#include "lib/time.h"
#include "arm.h"
#include "camera.h"
#include "gello.h"
#include "robot/smoother.h"
#include "task.h"
#include "user_input.h"

namespace robot {

    using namespace lib;

    struct Observation {
        // cv::Mat     cam_side;
        // cv::Mat     cam_wrist;
        QList<cv::Mat> cams;
        robot::Pose current_pose;
        robot::Pose target_pose;
    };

    struct RealEnv {

        const int max_steps               = INT_MAX;
        const time::duration control_freq = time::hz(50);

        RealEnv();

        robot::Pose start_pose;

        // CameraFeed cam_side;
        // CameraFeed cam_wrist;
        TaskConfig        task_config;
        QList<std::shared_ptr<CameraFeed>> cameras;
        AR4Gello          gello;
        AR4Arm            arm;

        robot::MotionSmoother smoother;
        
        std::list<Observation> observations;

        void start(error &err);

        bool run(UserInput &user_input, error &err);

        bool prepare(UserInput &user_input, error &err);
        void step(error &err);

        void shutdown();



        void move_to_start_position(error &err);

        // void move_arm(robot::Pose const& pose, error&);
    } ;
}
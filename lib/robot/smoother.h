#pragma once

#include "robot.h"
#include "arm.h"

#include "lib/base.h"
#include "lib/time.h"

namespace robot {
    using namespace lib;

    struct JointSmoother {
        float64 pos1;
        float64 pos2;
        bool first = true;

        MoveJointRequest step(float64 target, float64 currpos, time::duration time_delta, JointLimits const& limits, float64 time_multiple);
        void reset();
    } ;

    struct JointSmoother2 {
        float64 pos1;
        float64 pos2;

        MoveJointRequest step(float64 target, float64 currpos, time::duration full_time, time::duration half_time, JointLimits const& limits);
        void reset();
    } ;

    struct MotionSmoother {
        time::duration time_delta;
        JointSmoother j1, j2, j3, j4, j5, j6;
        float64 time_multiple = 2;

        MotionSmoother(time::duration time_delta) : time_delta(time_delta) {}

        void move(AR4Arm &arm, Pose const& target_pose, Pose const& curr_pose, error &err);
        void move(AR4Arm &arm, robot::JointsConfiguration jcfg, Pose const& current_pose, error &err);
        void reset();
    } ;

    struct MotionSmoother2 {
        JointSmoother2 j1, j2, j3, j4, j5, j6;

        time::monotime time1;
        time::monotime time2;
        
        int cnt = 0;

        void move(AR4Arm &arm, Pose const& target_pose, Pose const& curr_pose, error &err);
        void reset();
    } ;
}
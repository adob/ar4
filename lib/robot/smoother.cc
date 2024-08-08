#include "smoother.h"

#include "arm.h"
#include "lib/math.h"
#include "lib/print.h"
#include "robot/robot.h"

using namespace lib;
using namespace robot;

const bool use_time = true;

MoveJointRequest JointSmoother::step(float64 target, float64 currpos, time::duration time_delta, JointLimits const& limits, float64 time_multiple) {
    pos1 = pos2;
    pos2 = target;

    float64 delta_distance = pos2 - pos1;

    if (use_time) { 
    
        return {
            .enable                   = true,
            .position_degrees         = pos2,
            .speed_degrees_per_second = limits.max_speed,
            // .speed_out_degrees_per_second = limits.max_speed,
            .acceleration             = limits.acc_multiplier * limits.max_speed,
            // .deceleration             = 999'999,
            .target_time              = time_multiple*time_delta.seconds(),
        };
    }
    


    // if (ts < 2) {
    //     if (ts == 0) {
    //         pos1 = target;
    //     }
    //     ts++;
    // }

    if (first) {
        first = false;
        return {};
    }

    // overall speed
    // float64 speed_overall = math::dist(currpos, pos2) / (2*time_delta.seconds());

    // speed on second half
    float64 speed2 = math::dist(pos1, pos2) / (2*time_delta.seconds());

    // move to pos2 at speed overall speed
    // with out_speed set to speed2
    float64 normal_out_speed = 1;
    // float64 speed = math::min(limits.max_speed, speed_overall);
    float64 speed = math::min(limits.max_speed, speed2);
    
    return {
        .enable                       = true,
        .position_degrees             = pos2,
        .speed_degrees_per_second     = speed,
        // .speed_out_degrees_per_second = speed,
        // .speed_out_degrees_per_second = math::max(normal_out_speed, speed2),
        // .speed_out_degrees_per_second = limits.max_speed,
        .acceleration                 = limits.acc_multiplier * speed,
        // .acceleration = limits.acc,
    };
}

void JointSmoother::reset() {
    first = true;
}

MoveJointRequest JointSmoother2::step(float64 target, float64 currpos, time::duration full_time, time::duration half_time, JointLimits const& limits) {
    pos1 = pos2;
    pos2 = target;

    // overall speed
    float64 speed_overall = math::dist(currpos, pos2) / (full_time.seconds());

    // speed on second half
    float64 speed2 = math::dist(pos1, pos2) / half_time.seconds();

    // move to pos2 at speed overall speed
    // with out_speed set to speed2
    float64 normal_out_speed = 1;
    float64 speed = math::min(limits.max_speed, speed_overall);

    return {
        .enable                       = true,
        .position_degrees             = pos2,
        .speed_degrees_per_second     = speed,
        // .speed_degrees_per_second = limits.max_speed,
        // .speed_out_degrees_per_second = math::max(normal_out_speed, speed2),
        // .speed_out_degrees_per_second = limits.max_speed,
        .acceleration                 = limits.acc_multiplier * speed,
        // .acceleration = limits.acc,
    };
}

void JointSmoother2::reset() {}

void MotionSmoother::move(AR4Arm &arm, Pose const& target_pose, Pose const& current_pose, error &err) {
    MoveJointsAbsRequest req = {
        .j1 = j1.step(target_pose[0], current_pose[0], time_delta, arm.j1_limits, time_multiple),
        .j2 = j2.step(target_pose[1], current_pose[1], time_delta, arm.j2_limits, time_multiple),
        .j3 = j3.step(target_pose[2], current_pose[2], time_delta, arm.j3_limits, time_multiple),
        .j4 = j4.step(target_pose[3], current_pose[3], time_delta, arm.j4_limits, time_multiple),
        .j5 = j5.step(target_pose[4], current_pose[4], time_delta, arm.j5_limits, time_multiple),
        .j6 = j6.step(target_pose[5], current_pose[5], time_delta, arm.j6_limits, time_multiple),
    };

    arm.move_joints(req, err);
}

void MotionSmoother::move(AR4Arm &arm, robot::JointsConfiguration jcfg, Pose const& current_pose, error &err) {
    MoveJointsAbsRequest req = {
        .j1 = j1.step(jcfg.j1, current_pose[0], time_delta, arm.j1_limits, time_multiple),
        .j2 = j2.step(jcfg.j2, current_pose[1], time_delta, arm.j2_limits, time_multiple),
        .j3 = j3.step(jcfg.j3, current_pose[2], time_delta, arm.j3_limits, time_multiple),
        .j4 = j4.step(jcfg.j4, current_pose[3], time_delta, arm.j4_limits, time_multiple),
        .j5 = j5.step(jcfg.j5, current_pose[4], time_delta, arm.j5_limits, time_multiple),
        .j6 = j6.step(jcfg.j6, current_pose[5], time_delta, arm.j6_limits, time_multiple),
    };
    
    req.has_alt_wrist = jcfg.has_alt_wrist;
    req.j4b = jcfg.j4b;
    req.j5b = jcfg.j5b;
    req.j6b = jcfg.j6b;

    arm.move_joints(req, err);
}

void MotionSmoother::reset() {
    j1.reset();
    j2.reset();
    j3.reset();
    j4.reset();
    j5.reset();
    j6.reset();
}

void MotionSmoother2::move(AR4Arm &arm, Pose const& target_pose, Pose const& current_pose, error &err) {
    time::monotime time0 = time1;
    time1 = time2;
    time2 = time::mono();
    
    if (cnt < 2) {
        cnt++;
    }

    time::duration full_time = time2 - time0;
    time::duration half_time = time2 - time1;

    MoveJointsAbsRequest req = {
        .j1 = j1.step(target_pose[0], current_pose[0], full_time, half_time, arm.j1_limits),
        .j2 = j2.step(target_pose[1], current_pose[1], full_time, half_time, arm.j2_limits),
        .j3 = j3.step(target_pose[2], current_pose[2], full_time, half_time, arm.j3_limits),
        .j4 = j4.step(target_pose[3], current_pose[3], full_time, half_time, arm.j4_limits),
        .j5 = j5.step(target_pose[4], current_pose[4], full_time, half_time, arm.j5_limits),
        .j6 = j6.step(target_pose[5], current_pose[5], full_time, half_time, arm.j6_limits),
    };

    arm.move_joints(req, err);
}

void MotionSmoother2::reset() {
    cnt = 0;
    j1.reset();
    j2.reset();
    j3.reset();
    j4.reset();
    j5.reset();
    j6.reset();
}
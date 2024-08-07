#include <fcntl.h>
#include <math.h>

#include "robot.h"
#include "interface/interface.h"

#include "lib/io.h"
#include "lib/print.h"

using namespace lib;
using namespace robot;

void Pose::fmt(io::OStream &out, error&) const {
    fmt::fprintf(out, "{%f %f %f %f %f %f}", data[0], data[1], data[2], data[3], data[4], data[5]);
}

RobotCfg robot::ar4 {
    j1: {
        origin_offset: 170,
        invert_direction: true,

        //home_offset_degrees:   -392 / (16'000 / 360.0),
        home_offset_degrees: 1,
        theta_offset_degrees: 0,
        // theta_offset_degrees: 90,

        alpha: -M_PI / 2, // -90
        a: 64.20,
        d: 169.77,
        
        min_angle: -170,
        max_angle: 170,
    },

    j2: {
        origin_offset: 42,

        home_offset_degrees: -2,
        theta_offset_degrees: -90,
        scale_factor: 1.0 / (90.85/90.0),
        alpha: 0,
        a: 305.00,
        d: 0,
        
        min_angle: -42,
        max_angle:  90,
    },

    j3: {
        origin_offset: 52,
        invert_direction: true,

        home_offset_degrees: 3.9,
        //home_offset_degrees: -93 / (20'000 / 360.0) + 5,
        theta_offset_degrees: 180,
        alpha: M_PI / 2, // 90
        a: 0,
        d: 0,
        
        min_angle: -99,
        max_angle: 52, //99,
    },

    j4: {
        origin_offset: 165,

        home_offset_degrees: 4.8,
        theta_offset_degrees: 0,
        alpha: -M_PI / 2, // -90
        a: 0,
        d: 222.63,
        
        min_angle: -165,
        max_angle: 165,
    },

    j5: {
        origin_offset: 105,

        home_offset_degrees: 10,// -24 + 3,
        theta_offset_degrees: 0,
        alpha: M_PI / 2, // 90
        a: 0,
        d: 0,
        
        min_angle: -105,
        max_angle: 105,
    },

    j6: {
        origin_offset: 155,
        invert_direction: true,

        home_offset_degrees: -1,
        theta_offset_degrees: 0,
        alpha: 0,
        a: 0,
        d: 36.25,
        
        min_angle: -155,
        max_angle: 155,
    }
};

static float64 fix_position(float64 target, JointCfg const& cfg) {
    target *= cfg.scale_factor;

    if (cfg.invert_direction) {
        target = -target;
    }

    target += cfg.origin_offset;
    target += cfg.home_offset_degrees;

    return target;
}

static float64 unfix_position(float64 p, JointCfg const& cfg) {
    p -= cfg.home_offset_degrees;
    p -= cfg.origin_offset;;

    if (cfg.invert_direction) {
        p = -p;
    }

    p /= cfg.scale_factor;

    return p;
}

void robot::robot_move_abs(RobotCfg const& cfg, Client &client, MoveJointsAbsRequest const& req, error &err) {
    MoveJointsAbsRequest outreq = req;

    outreq.has_alt_wrist = req.has_alt_wrist;

    if (req.j1.enable && !std::isnan(req.j1.position_degrees)) {
        outreq.j1.position_degrees = fix_position(req.j1.position_degrees, cfg.j1);
    }

    if (req.j2.enable && !std::isnan(req.j2.position_degrees)) {
        outreq.j2.position_degrees = fix_position(req.j2.position_degrees, cfg.j2);
    }

    if (req.j3.enable && !std::isnan(req.j3.position_degrees)) {
        outreq.j3.position_degrees = fix_position(req.j3.position_degrees, cfg.j3);
    }

    if (req.j4.enable && !std::isnan(req.j4.position_degrees)) {
        outreq.j4.position_degrees = fix_position(req.j4.position_degrees, cfg.j4);
        outreq.j4b = fix_position(req.j4b, cfg.j4);
    }

    if (req.j5.enable && !std::isnan(req.j5.position_degrees)) {
        outreq.j5.position_degrees = fix_position(req.j5.position_degrees, cfg.j5);
        outreq.j5b = fix_position(req.j5b, cfg.j5);
    }

    if (req.j6.enable && !std::isnan(req.j6.position_degrees)) {
        outreq.j6.position_degrees = fix_position(req.j6.position_degrees, cfg.j6);
        outreq.j6b = fix_position(req.j6b, cfg.j6);
        //print "j6", outreq.j6.position_degrees, outreq.j4b;
    }

    client.move_joints_abs(outreq, err);
}

void robot::move_to_pose(RobotCfg const& cfg, Client &client, robot::JointsConfiguration const& target_pose, bool await, bool fast, error &err) {
    const float64 m = 1.0;

    robot::MoveJointsAbsRequest req;
    req.j1.enable = true;
    req.j1.speed_degrees_per_second *= !fast ? 1 : 4*m;
    req.j1.acceleration *= !fast ? 1 : 4*4*m;
    req.j1.position_degrees = target_pose.j1;

    req.j2.enable = true;
    req.j2.speed_degrees_per_second *= !fast ? 1 : 4*m;
    req.j2.acceleration *= !fast ? 1 : 4*4*m;
    req.j2.position_degrees = target_pose.j2;

    req.j3.enable = true;
    req.j3.speed_degrees_per_second *= !fast ? 1 : 4*m;
    req.j3.acceleration *= !fast ? 1 : 4*4*m;
    req.j3.position_degrees = target_pose.j3;

    req.j4.enable = true;
    req.j4.speed_degrees_per_second *= !fast ? 4*m : 8*m;
    req.j4.acceleration *= !fast ? 4*4*m : 8*8*m;
    req.j4.position_degrees = target_pose.j4;

    req.j5.enable = true;
    req.j5.speed_degrees_per_second *= !fast ? 4*m : 8*m;
    req.j5.acceleration *= !fast ? 4*4*m : 8*8*m;
    req.j5.position_degrees = target_pose.j5;

    req.j6.enable = true;
    req.j6.speed_degrees_per_second *= !fast ? 4*m : 8*m;
    req.j6.acceleration *= !fast ? 4*4*m : 8*8*m;
    req.j6.position_degrees = target_pose.j6;

    req.has_alt_wrist = target_pose.has_alt_wrist;
    req.j4b = target_pose.j4b;
    req.j5b = target_pose.j5b;
    req.j6b = target_pose.j6b;

    req.await = await;

    robot_move_abs(cfg, client, req, err);
}

Pose robot::get_pose(RobotCfg const& cfg, StateResponse const& state) {
    return {
        unfix_position(state.j1_angle, cfg.j1),
        unfix_position(state.j2_angle, cfg.j2),
        unfix_position(state.j3_angle, cfg.j3),
        unfix_position(state.j4_angle, cfg.j4),
        unfix_position(state.j5_angle, cfg.j5),
        unfix_position(state.j6_angle, cfg.j6),
    };
}

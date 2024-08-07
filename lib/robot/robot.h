#pragma once

#include "lib/base.h"
#include "lib/io.h"

#include "client.h"
#include "interface/interface.h"

namespace robot {
    using namespace lib;

    struct Pose : Array<float64, 6> {
        void fmt(io::OStream &, error&) const;
    };

    struct JointCfg {
        double origin_offset = 0;
        bool invert_direction = false;

        double home_offset_degrees = 0;
        double theta_offset_degrees = 0;
        float64 scale_factor = 1.0;

        double alpha;
        double a;
        double d;
        
        float64 min_angle = -180;
        float64 max_angle = 180;
    };

    struct JointsConfiguration {
        float64 j1 = 0;
        float64 j2 = 0;
        float64 j3 = 0;
        float64 j4 = 0;
        float64 j5 = 0;
        float64 j6 = 0;

        bool has_alt_wrist = false;
        float64 j4b = 0;
        float64 j5b = 0;
        float64 j6b = 0;
    } ;

    struct RobotCfg {
        JointCfg j1, j2, j3, j4, j5, j6;

        Array<JointCfg*, 6> joints = {{&j1, &j2, &j3, &j4, &j5, &j6}};
    } ;

    extern RobotCfg ar4;

    void robot_move_abs(RobotCfg const& cfg, Client &client, MoveJointsAbsRequest const& req, error &err);
    void move_to_pose(RobotCfg const& cfg, Client &client, JointsConfiguration const& pose, bool await, bool fast, error &err);
    Pose get_pose(RobotCfg const& cfg, StateResponse const& state);
}

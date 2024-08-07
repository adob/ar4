#pragma once

#include <memory>

#include "lib/os/file.h"
#include "robot.h"
#include "serialrpc/client.h"

namespace robot {
    using namespace lib;

    struct JointLimits {
        float64 max_speed = 25;
        float64 acc = 1;
        float64 acc_multiplier = 1;
    } ;

    struct AR4Arm {

        JointLimits j1_limits = {
            .max_speed = 25*4,
            .acc       = 4*(25*4),
            .acc_multiplier = 4,
        };
        JointLimits j2_limits = {
            .max_speed = 25*4,
            .acc       = 4*(25*4),
            .acc_multiplier = 4,
        };
        JointLimits j3_limits = {
            .max_speed = 25*6,
            .acc       = 6*(25*4),
            .acc_multiplier = 6,
        };
        JointLimits j4_limits = {
            .max_speed = 25*8,
            .acc       = 8*(25*8),
            .acc_multiplier = 8,
        };
        JointLimits j5_limits = {
            .max_speed = 25*8,
            .acc       = 8*(25*8),
            .acc_multiplier = 8,
        };
        JointLimits j6_limits = {
            .max_speed = 25*16,
            .acc       = 8*(25*16),
            .acc_multiplier = 8,
        };


        std::shared_ptr<os::File> conn;
        robot::Client             client;

        void connect(str device_path, error &err);
        void disconnect();
        void move_to_start_position(error &err);
        void move_joints(robot::MoveJointsAbsRequest const& req, error &err);
        void move_joints_rel(robot::Pose const& joints, bool await, error &err);
        void move_cart(robot::Pose const&pose, bool await, error &err);
        void move_abs(robot::Pose const& pose, bool await, error &err);
        void move_abs_fast(robot::Pose const& pose, bool await, error& err);
        void move_linear(robot::Pose const& cart_pose, error& err);
        void move_linear_rel(robot::Pose const& rel_cart, error& err);

        robot::Pose get_pose(error&);
        robot::Pose get_cart_pose(error&);
    };
}
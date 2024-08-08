#pragma once

#include <kdl/frames.hpp>
#include <visp3/core/vpHomogeneousMatrix.h>

#include "robot.h"
#include "lib/base.h"
#include "lib/array.h"
#include "lib/math/matrix.h"
#include "lib/types.h"


namespace robot {
    using namespace lib;

    struct DHParams {
        float64 theta;
        float64 alpha;
        float64 a;
        float64 r;
    };

    Pose forward_kinematics(RobotCfg const& cfg, Pose const& pose);
    math::Matrix4x4 forward_kinematics_mat(RobotCfg const& cfg, Pose const& pose);
    JointsConfiguration inverse_kinematics(RobotCfg const& cfg, Pose const& desired_pose);
    JointsConfiguration inverse_kinematics(RobotCfg const& cfg, float64 x, float64 y, float64 z, math::Matrix3x3 const& rotm);
    JointsConfiguration inverse_kinematics_mat(RobotCfg const& cfg, math::Matrix4x4 const& m);
    robot::Pose inverse_kinematics_mat(RobotCfg const& cfg, math::Matrix4x4 const& m, robot::Pose const &current_joints);
    void rotm2eul_zyx(math::Matrix3x3 const& m, float64 *z, float64 *y, float64 *x);

    Pose reverse_kinematics_wrist(RobotCfg const& cfg,  float64 x, float64 y, float64 z);
    void ik_wrist(math::Matrix3x3 const& m36, JointsConfiguration *joints);
    
    math::Matrix4x4 eul2mat(Pose const& pose);
    robot::Pose     mat2eul(math::Matrix4x4 const& m);
    
    vpHomogeneousMatrix to_vp(math::Matrix4x4 const& m);
    math::Matrix4x4 from_vp(vpHomogeneousMatrix const& m);
    
    math::Matrix3x3 extract_rotm(math::Matrix4x4 const& m);

    math::Matrix4x4 frame2mat(KDL::Frame const& f);
    KDL::Frame mat2frame(math::Matrix4x4 const& m);
}

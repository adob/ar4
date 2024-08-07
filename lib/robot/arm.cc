#include "arm.h"

#include <kdl/path_line.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/velocityprofile_trap.hpp>

#include "robot/client.h"
#include "robot/kinematics.h"

#include <memory>

#include "lib/os.h"
#include "lib/time.h"
#include "robot/robot.h"

#include "lib/print.h"
#include "robot/smoother.h"

using namespace robot;

void AR4Arm::connect(str device_path, error &err) {
    conn = std::make_shared<os::File>(os::open(device_path, O_RDWR, err));
    // printf("FILE %lx %lx\n", &(*conn));
    if (err) {
        return;
    }

    std::shared_ptr<serialrpc::Client> srpc = make_shared<serialrpc::Client>(conn);
    client = robot::Client(srpc);
     
}

void AR4Arm::move_to_start_position(error &err) {
    print "arm moving to start position";
    robot::robot_move_abs(robot::ar4, client, {
            .j1 = {.enable = true},
            .j2 = {.enable = true},
            .j3 = {.enable = true},
            .j4 = {.enable = true},
            .j5 = {.enable = true},
            .j6 = {.enable = true},
            .await = true,
        }, err);
}

void dump_joint_cmd(str name, robot::MoveJointRequest const& req, float64 alt) {
    if (req.enable) {
        print "move %s pos %s; speed %s; acc %s; speedin %s; speedout %s; alt %s" % name,
            req.position_degrees, req.speed_degrees_per_second, req.acceleration, 
            req.speed_in_degrees_per_second, req.speed_out_degrees_per_second, alt;
    }
}


void AR4Arm::move_joints(robot::MoveJointsAbsRequest const& req, error &err) {
    dump_joint_cmd("j1", req.j1, 0);
    dump_joint_cmd("j2", req.j2, 0);
    dump_joint_cmd("j3", req.j3, 0);
    dump_joint_cmd("j4", req.j4, req.j4b);
    dump_joint_cmd("j5", req.j5, req.j5b);
    dump_joint_cmd("j6", req.j6, req.j6b);
    robot::robot_move_abs(robot::ar4, client, req, err);
}

void AR4Arm::move_joints_rel(robot::Pose const& joints, bool await, error& err) {
    robot::Pose curr_joints = get_pose(err);
    if (err) {
        return;
    }
    
    robot::MoveJointsAbsRequest req;
    
    if (joints[0] != 0) {
        req.j1.enable           = true;
        req.j1.position_degrees = curr_joints[0] + joints[0];
    }

    if (joints[1] != 0) {
        req.j2.enable = true;
        req.j2.position_degrees = curr_joints[1] + joints[1];
    }

    if (joints[2] != 0) {
        req.j3.enable = true;
        req.j3.position_degrees = curr_joints[2] + joints[2];
    }

    if (joints[3] != 0) {
        req.j4.enable = true;
        req.j4.position_degrees = curr_joints[3] + joints[3];
    }

    if (joints[4] != 0) {
        req.j5.enable = true;
        req.j5.position_degrees = curr_joints[4] + joints[4];
    }

    if (joints[5] != 0) {
        req.j6.enable = true;
        req.j6.position_degrees = curr_joints[5] + joints[5];
    }
    
    req.await = await;
    
    move_joints(req, err);
}

void AR4Arm::move_abs(robot::Pose const& pose, bool await, error& err) {
    robot::robot_move_abs(robot::ar4, client, {
            .j1 = {.enable = true, .position_degrees = pose[0] },
            .j2 = {.enable = true, .position_degrees = pose[1] },
            .j3 = {.enable = true, .position_degrees = pose[2] /*/, .speed_degrees_per_second = robot::DefaultSpeedDegreesPerSecond*4, .acceleration = robot::DefaultSpeedDegreesPerSecond*4*4*/ },
            .j4 = {.enable = true, .position_degrees = pose[3], .speed_degrees_per_second = robot::DefaultSpeedDegreesPerSecond*4, .acceleration = robot::DefaultSpeedDegreesPerSecond*4*4 },
            .j5 = {.enable = true, .position_degrees = pose[4], .speed_degrees_per_second = robot::DefaultSpeedDegreesPerSecond*4, .acceleration = robot::DefaultSpeedDegreesPerSecond*4*4 },
            .j6 = {.enable = true, .position_degrees = pose[5], .speed_degrees_per_second = robot::DefaultSpeedDegreesPerSecond*4, .acceleration = robot::DefaultSpeedDegreesPerSecond*4*4 },
            .await = await,
        }, err);
}

void AR4Arm::move_abs_fast(robot::Pose const& pose, bool await, error& err)
{
    robot::robot_move_abs(robot::ar4, client, {
            .j1 = {.enable = true, .position_degrees = pose[0], .speed_degrees_per_second = robot::DefaultSpeedDegreesPerSecond*4, .acceleration = robot::DefaultSpeedDegreesPerSecond*4*4 },
            .j2 = {.enable = true, .position_degrees = pose[1], .speed_degrees_per_second = robot::DefaultSpeedDegreesPerSecond*4, .acceleration = robot::DefaultSpeedDegreesPerSecond*4*4 },
            .j3 = {.enable = true, .position_degrees = pose[2], .speed_degrees_per_second = robot::DefaultSpeedDegreesPerSecond*4, .acceleration = robot::DefaultSpeedDegreesPerSecond*4*4 },
            .j4 = {.enable = true, .position_degrees = pose[3], .speed_degrees_per_second = robot::DefaultSpeedDegreesPerSecond*8, .acceleration = robot::DefaultSpeedDegreesPerSecond*4*8 },
            .j5 = {.enable = true, .position_degrees = pose[4], .speed_degrees_per_second = robot::DefaultSpeedDegreesPerSecond*8, .acceleration = robot::DefaultSpeedDegreesPerSecond*4*8 },
            .j6 = {.enable = true, .position_degrees = pose[5], .speed_degrees_per_second = robot::DefaultSpeedDegreesPerSecond*8, .acceleration = robot::DefaultSpeedDegreesPerSecond*4*8 },
            .await = await,
        }, err);
}

void AR4Arm::move_cart(robot::Pose const& pose, bool await, error &err) {
    robot::JointsConfiguration target_pose = robot::inverse_kinematics(robot::ar4, pose);
    robot::move_to_pose(robot::ar4, client, target_pose, await, false, err);
}

void AR4Arm::move_linear(robot::Pose const& cart_pose, error& err) {
    float64 translation_speed = 100; // 100 mm / sec
    float64 translation_acc = 100;
    float64 rot_speed_rad = 22.5 * KDL::deg2rad;
    
    robot::Pose start_pose_cart = get_cart_pose(err);
    if (err) {
        return;
    }

    KDL::Frame start_frame = robot::mat2frame(robot::eul2mat(start_pose_cart));
    KDL::Frame end_frame = robot::mat2frame(robot::eul2mat(cart_pose));
    KDL::RotationalInterpolation_SingleAxis rot_iterp;
    
    float64 eqradius = translation_speed / rot_speed_rad;

    KDL::Path_Line path_line(start_frame, end_frame, &rot_iterp, eqradius, false);
    float64 path_length = path_line.PathLength();

    KDL::VelocityProfile_Trap vel_profile(translation_speed, translation_acc);
    vel_profile.SetProfile(/*pos1=*/0, /*pos2=*/path_length);

    KDL::Trajectory_Segment traj_seg(&path_line, &vel_profile, /*duration=*/1.0, false);

    float64 t = 0;
    float64 delta_t = 0.01;
    time::duration control_freq = 10 * time::millisecond;
    time::LoopTimer loop_timer(control_freq);

    robot::MotionSmoother smoother(control_freq);
    for (;;) {
        loop_timer.delay();

        float64 progress = vel_profile.Pos(t);

        KDL::Frame desired_frame = path_line.Pos(progress);
        math::Matrix4x4 mat = robot::frame2mat(desired_frame);

        robot::JointsConfiguration jcfg = robot::inverse_kinematics_mat(robot::ar4, mat);
        
        robot::Pose curr_pose = get_pose(err);
        if (err) {
            return;
        }
        
        smoother.move(*this, jcfg, curr_pose, err);
        if (err) {
            return;
        }

        // print "mat", t, "\n", mat;

        if (progress >= path_length) {
            break;
        }
        t += delta_t;
    }
    
    this->move_cart(cart_pose, true, err);
    if (err) {
        return;
    }
}

void AR4Arm::move_linear_rel(robot::Pose const& rel_cart, error& err) {
    robot::Pose pose = get_cart_pose(err);
    if (err) {
        return;
    }
    
    for (int i = 0; i < 6; i++) {
        pose[i] += rel_cart[i];
    }
    
    move_linear(pose, err);
}

robot::Pose AR4Arm::get_pose(error &err) {
    robot::StateResponse resp = client.get_state(err);
    if (err) {
        return {};
    }
    return robot::get_pose(robot::ar4, resp);
}

robot::Pose AR4Arm::get_cart_pose(error &err) {
    robot::Pose pose = get_pose(err);
    if (err) {
        return {};
    }
    
    return robot::forward_kinematics(robot::ar4, pose);
}

void AR4Arm::disconnect() {
    conn->close(error::ignore());
}
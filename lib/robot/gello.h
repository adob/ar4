#pragma once

#include "DynamixelSDK/c++/include/dynamixel_sdk/dynamixel_sdk.h"

#include "robot/robot.h"

#include "lib/async.h"
#include "lib/base.h"
#include "user_input.h"

namespace robot {
    using namespace lib;

    struct AR4Gello {
        dynamixel::PortHandler   *port_handler   = nil;
        dynamixel::PacketHandler *packet_handler = nil;

        const float64 alpha = 0.9;

        const int32 j1_offset = -4096/2;
        const int32 j2_offset = -int32((90.0/360) * 4096);
        const int32 j3_offset = -int32((0.0/360) * 4096);
        const int32 j4_offset = -int32((90.0/360) * 4096);
        const int32 j5_offset = -int32((135.0/360) * 4096);
        const int32 j6_offset = -int32((0.0/360) * 4096);

        robot::Pose base_pose;

        void connect(str device_path, error &err);
        bool start(UserInput *user_input, robot::Pose const& pose, error &err);
        robot::Pose get_current_pose(error &err);
        void stop();


        void reboot(error &err);
        void torque_enable_all(bool enable, error &err);
        void torque_enable_joint(uint8 dynamixel_id, bool enable, error &err);
        void set_moving_theshold(int32 amt, error &err);
        void move_joint(uint8 joint, float64 deg, error &err);
        void set_speed(uint8 joint, float64 speed_degrees_per_second, error &err);

        void write1(uint8 dynamixel_id, uint16 addr, uint8 data, error &err);
        robot::Pose read_joints(error &err);

        sync::Mutex mtx;
        robot::Pose current_pose;
        async::Future<void> read_done;
        bool running = false;
        error read_err;

        void read_loop(sync::WaitGroup &ready);
    } ;
}
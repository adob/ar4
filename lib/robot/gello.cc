#include "gello.h"

#include <cmath>

#include "robot/kinematics.h"
#include "robot/robot.h"
#include "tts/tts.h"

#include "lib/time.h"
#include "lib/print.h"
#include "user_input.h"

using namespace robot;

static bool check_comm_error(int error_code, error &err) {
    switch (error_code) {
        case COMM_SUCCESS:
            return true;

        case COMM_PORT_BUSY:
            err("port busy");
            return false;
        case COMM_TX_FAIL:
            err("tx fail");
            return false;
        case COMM_RX_FAIL:
            err("rx fail");
            return false;
        case COMM_TX_ERROR:
            err("tx error");
            return false;
        case COMM_RX_WAITING:
            err("rx waiting");
            return false;
        case COMM_RX_TIMEOUT:
            err("rx timeout");
            return false;
        case COMM_RX_CORRUPT:
            err("rx corrupt");
            return false;
        case COMM_NOT_AVAILABLE:
            err("not available");
            return false;
        default:
            err("comm error");
            return false;
        }
}

static float64 dist(float64 a, float64 b) {
    return std::abs(a - b);
}

void AR4Gello::write1(uint8 dynamixel_id, uint16 addr, uint8 data, error &err) {
    uint8 error = 0;

    int result = packet_handler->write1ByteTxRx(port_handler, dynamixel_id, 
            addr, 
            data /* extended position control mode*/,
            &error);
    check_comm_error(error, err);
    if (err) {
        return;
    }
    if (error) {
        err("cmd error %d", error);
    }
}

void AR4Gello::connect(str device_path, error &err) {
    if (port_handler) {
        delete port_handler;
    }
    port_handler = dynamixel::PortHandler::getPortHandler(device_path.c_str());

    if (packet_handler) {
        delete packet_handler;
    }
    packet_handler = dynamixel::PacketHandler::getPacketHandler(2.0);

    bool ok = port_handler->openPort();
    if (!ok) {
        err("AR4Gello: open port failed");
        return;
    }

    const int baud_rate = 57600;
    ok = port_handler->setBaudRate(baud_rate);
    if (!ok) {
        err("AR4Gello: set baud rate failed");
        return;
    }

    reboot(err);
    if (err) {
        return;
    }

    const uint16 AddrOperatingMode = 11;
    for (uint8 dynamixel_id = 1; dynamixel_id <= 6; dynamixel_id++) {
 
        write1(dynamixel_id, AddrOperatingMode, 4 /* extended position control mode*/, err);
        if (err) {
            return;
        }
    }
    // write1(6, AddrOperatingMode, 3 /* position control mode*/, err);

    set_moving_theshold(0, err);
    if (err) {
        return;
    }
    
    for (int i = 1; i <= 6; i++) {
        set_speed(uint8(i), 25, err);
        if (err) {
            return;
        }
    }
}

void AR4Gello::reboot(error &err) {
    for (uint8 dynamixel_id = 1; dynamixel_id <= 6; dynamixel_id++) {
        uint8 error = 0;
        int result = packet_handler->reboot(port_handler, dynamixel_id, &error);
        check_comm_error(result, err);
        if (err) {
            return;
        }
        if (error) {
            err("reboot error: %d", error);
            return;
        }
    }

        for (uint8 dynamixel_id = 1; dynamixel_id <= 6; dynamixel_id++) {
        retry:    
        uint8 error = 0;
        int result = packet_handler->ping(port_handler, dynamixel_id, &error);
        if (result == COMM_RX_TIMEOUT) {
            goto retry;
        }
        check_comm_error(result, err);
        if (err) {
            return;
        }
        if (error) {
            err("ping error");
            return;
        }
        }
}

void AR4Gello::torque_enable_all(bool enable, error &err) {
    for (uint8 dynamixel_id = 1; dynamixel_id <= 6; dynamixel_id++) {
        torque_enable_joint(dynamixel_id, enable, err);
        if (err) {
            return;
        }
    }
}

void AR4Gello::torque_enable_joint(uint8 dynamixel_id, bool enable, error &err) {
    const uint16 AddrTorqueEnable = 64;

    uint8 packet_error = 0;

    int result = packet_handler->write1ByteTxRx(
        port_handler, 
        dynamixel_id,
        AddrTorqueEnable, 
        enable ? 1 : 0,
        &packet_error);

    if (result != COMM_SUCCESS) {
        err("torque enable comm error: %d", result);
    }
    
    if (packet_error) {
        err("torque_enable packet error: %d", packet_error);
        return;
    }
}

void AR4Gello::set_moving_theshold(int32 amt, error &err) {
    const uint16 AddrMovingThreshold = 24;
    uint8 error = 0;

    for (uint8 dynamixel_id = 1; dynamixel_id <= 6; dynamixel_id++) {
        int result = packet_handler->write4ByteTxRx(port_handler, dynamixel_id, AddrMovingThreshold, amt, &error);
        check_comm_error(result, err);
        if (err) {
            return;
        }
        if (error) {
            err("set_moving_theshold: insturction error: %d", error);
            return;
        }
    }
}

void AR4Gello::move_joint(uint8 joint, float64 deg, error &err) {
    const uint16 AddrGoalPosition = 116;
    const uint16 LenGoalPosition = 4;

    dynamixel::GroupSyncWrite sync_write(
        port_handler, 
        packet_handler,
        AddrGoalPosition,
        LenGoalPosition);

    int32 goal_position = std::round((deg / 360.0) * 4096);
    if (joint == 1) {
        goal_position -= j1_offset;
    } else if (joint == 2) {
        goal_position -= j2_offset;
    } else if (joint == 3) {
        goal_position = -goal_position;
        goal_position -= j3_offset;
    } else if (joint == 4) {
        goal_position -= j4_offset;
    } else if (joint == 5) {
        goal_position = -goal_position;
        goal_position -= j5_offset;
    } else if (joint == 6) {
        goal_position -= j6_offset;
    } else {
        err("unkown joint: %d", joint);
        return;
    }
    // print "move_joint %d to %d" % joint, goal_position;

    bool ok = sync_write.addParam(joint, (uint8*) &goal_position);
    if (!ok) {
        err("failed to set param");
        return;
    }

    int result = sync_write.txPacket();
    if (result != COMM_SUCCESS) {
        err("comm failure %d", result);
    }
}

void AR4Gello::set_speed(uint8 joint, float64 speed_degrees_per_second, error &err) {
    if (speed_degrees_per_second <= 0) {
        err("speed is negative or zero");
        return;
    }
    
    const uint64 AddrProfileVelocity = 112;
    
    dynamixel::GroupSyncWrite sync_write(
        port_handler, 
        packet_handler,
        AddrProfileVelocity,
        4);

    // 	unit 0.229 [rev/min]
    uint32 vel = uint16(std::round(speed_degrees_per_second / 360.0 * 60.0 / 0.229));
    if (vel == 0) {
        vel = 1;
    }
    print "set_speed", vel;

    bool ok = sync_write.addParam(joint, (uint8*)&vel);
    if (!ok) {
        err("failed to set param");
        return;
    }
    
    int result = sync_write.txPacket();
    if (result != COMM_SUCCESS) {
        err("comm failure %d", result);
    }
}

robot::Pose AR4Gello::read_joints(error &err) {
    const uint16 AddrPresentPosition = 132;
    const uint16 LenPresentPosition = 4;

    dynamixel::GroupSyncRead sync_read(port_handler, packet_handler, AddrPresentPosition, LenPresentPosition);
    sync_read.addParam(1);
    sync_read.addParam(2);
    sync_read.addParam(3);
    sync_read.addParam(4);
    sync_read.addParam(5);
    sync_read.addParam(6);

    int r = sync_read.txRxPacket();
    if (r != COMM_SUCCESS) {
        err("comm failure");
        return {};
    }

    for (int i = 1; i <= 6; i++) {
        if (!sync_read.isAvailable(i, AddrPresentPosition, LenPresentPosition)) {
            err("motor %d data not available", i);
            return {};
        }
    }

    int32 j1_pos = (int32) sync_read.getData(1, AddrPresentPosition, LenPresentPosition);
    float64 j1_deg = (j1_pos+j1_offset) / 4096.0 * 360;
    //float64 j1_deg = std::clamp((j1_pos+j1_offset) / 4096.0 * 360, -180.0, 180.0);

    int32 j2_pos = (int32) sync_read.getData(2, AddrPresentPosition, LenPresentPosition);
    float64 j2_deg = (j2_pos+j2_offset) / 4096.0 * 360;
    // float64 j2_deg = std::clamp((j2_pos+j2_offset) / 4096.0 * 360, -180.0, 180.0);

    int32 j3_pos = (int32) sync_read.getData(3, AddrPresentPosition, LenPresentPosition);
    // float64 j3_deg = std::clamp(-(j3_pos+j3_offset) / 4096.0 * 360, -180.0, 180.0);
    float64 j3_deg = (j3_pos+j3_offset) / 4096.0 * 360;

    int32 j4_pos = (int32) sync_read.getData(4, AddrPresentPosition, LenPresentPosition);
    // float64 j4_deg = std::clamp((j4_pos+j4_offset) / 4096.0 * 360, -180.0, 180.0);
    float64 j4_deg = (j4_pos+j4_offset) / 4096.0 * 360;

    int32 j5_pos = (int32) sync_read.getData(5, AddrPresentPosition, LenPresentPosition);
    // float64 j5_deg = std::clamp(-(j5_pos+j5_offset) / 4096.0 * 360, -180.0, 180.0);
    float64 j5_deg = (j5_pos+j5_offset) / 4096.0 * 360;

    int32 j6_pos = (int32) sync_read.getData(6, AddrPresentPosition, LenPresentPosition);
    // float64 j6_deg = std::clamp((j6_pos+j6_offset) / 4096.0 * 360, -180.0, 180.0);
    float64 j6_deg = (j6_pos+j6_offset) / 4096.0 * 360;

    // print "raw j2", j2_pos;
    // print "pos j1 %d -> %d; j2 %d -> %d; j3 %d -> %d; j4 %d -> %d; j5 %d -> %d; j6 %d -> %d" % j1_pos, j1_deg, j2_pos, j2_deg, j3_pos, j3_deg, j4_pos, j4_deg, j5_pos, j5_deg, j6_pos, j6_deg;
    return {j1_deg, j2_deg, -j3_deg, j4_deg, -j5_deg, j6_deg};
    // print "pos %d->%d", j1_deg, j2_deg, j3_deg, j4_deg, j5_deg, j6_deg;
}

void AR4Gello::read_loop(sync::WaitGroup &ready) {
    error err;
    bool first = true;

    for (time::LoopTimer loop_timer(time::hz(100));;) {
        loop_timer.delay();

        if (!running) {
            break;
        }

        robot::Pose pose = read_joints(err);
        for (int i = 0; i < 6; i++) {
            pose[i] = pose[i] - base_pose[i];
        }

        sync::Lock lock(mtx);
        if (err) {
            read_err = err;
            print "read_loop got error", err;
            break;
        }
        
        if (first) {
            first = false;
            current_pose = pose;
            ready.done();
            continue;
        }

        robot::Pose newpose;
        for (int i = 0; i < 6; i++) {
            if (dist(newpose[i], current_pose[i]) == 1) {
                newpose[i] = current_pose[i];
            } else {
                newpose[i] = pose[i];
            }
        }
        current_pose = newpose;
    }
}

bool AR4Gello::start(UserInput *user_input, robot::Pose const& start_pose, error &err) {
    if (running) {
        stop();
    }

    robot::Pose current_pose = read_joints(err);
    float64 j6_curr = current_pose[5];
    float64 j6_target = start_pose[5];
    // float64 j6_diff = j6_target - j6_curr;

    if (j6_curr - j6_target < 0 && j6_curr - j6_target > -180) {
        j6_target = 360 + j6_target;
    }

    print "j6 current position is", j6_curr;
    print "j6 target is", j6_target;

    tts::say("move to start position");    

    if (false) {
        for (time::LoopTimer loop_timer(time::hz(10));;) {
            loop_timer.delay();
            if (user_input && !user_input->get_foot_pedal()) {
                return false;
            }

            robot::Pose pose = read_joints(err);
            if (err) {
                return false;
            }
            const float64 max = 20;

            float64 dist1 = dist(pose[0], start_pose[0]);
            float64 dist2 = dist(pose[1], start_pose[1]);
            float64 dist3 = dist(pose[2], start_pose[2]);
            float64 dist4 = dist(pose[3], start_pose[3]);
            float64 dist5 = dist(pose[4], start_pose[4]);
            float64 dist6 = dist(pose[5], start_pose[5]);

            if (dist1 <= max && dist2 <= max && dist3 <= max && dist4 <= max && dist5 <= max && dist6 <= max) {
                tts::say("WAIT");
                break;
            }

            print "dists", dist1, dist2, dist3, dist4, dist5, dist6;
            print "pose", pose;
        }
    }

    torque_enable_all(true, err);
    if (err) {
        return false;
    }

    for (time::LoopTimer loop_timer(time::hz(10));;) {
        loop_timer.delay();

        if (user_input && !user_input->get_foot_pedal()) {
            torque_enable_all(false, err);
            return false;
        }

        move_joint(1, start_pose[0], error::panic);
        move_joint(2, start_pose[1], error::panic);
        move_joint(3, start_pose[2], error::panic);
        move_joint(4, start_pose[3], error::panic);
        move_joint(5, start_pose[4], error::panic);
        move_joint(6, start_pose[5], error::panic);

        robot::Pose pose = read_joints(err);
        if (err) {
            torque_enable_all(false, error::ignore());
            return false;
        }
        const float64 max = 5;

        float64 dist1 = dist(pose[0], start_pose[0]);
        float64 dist2 = dist(pose[1], start_pose[1]);
        float64 dist3 = dist(pose[2], start_pose[2]);
        float64 dist4 = dist(pose[3], start_pose[3]);
        float64 dist5 = dist(pose[4], start_pose[4]);
        float64 dist6 = dist(pose[5], start_pose[5]);

        if (dist1 <= max && dist2 <= max && dist3 <= max && dist4 <= max && dist5 <= max && dist6 <= max) {
            break;
        }

        print "dists", dist1, dist2, dist3, dist4, dist5, dist6;
        print "pose", pose;
    }

    torque_enable_all(false, err);
    if (err) {
        return false;
    }

    base_pose = read_joints(err);
    base_pose[0] -= start_pose[0];
    base_pose[1] -= start_pose[1];
    base_pose[2] -= start_pose[2];
    base_pose[3] -= start_pose[3];
    base_pose[4] -= start_pose[4];
    base_pose[5] -= start_pose[5];

    print "base_pose", base_pose;

    running = true;
    sync::WaitGroup ready(1);
    read_done = async::go([&] { read_loop(ready); });
    ready.wait();
    tts::say("BEGIN");
    return true;
}

robot::Pose AR4Gello::get_current_pose(error &err) {
    sync::Lock lock(mtx);
    if (read_err) {
        // print "get_current_pose read_err", read_err;
        err(read_err);
    }
    return current_pose;
}

void AR4Gello::stop() {
    {
        sync::Lock lock(mtx);
        running = false;
    }
    read_done.await();
}
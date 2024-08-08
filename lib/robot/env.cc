#include "env.h"

#include "camera.h"
#include "task.h"
#include "tts/tts.h"
#include "lib/print.h"
#include "user_input.h"
#include <memory>

using namespace robot;

static const bool dry_run = false;

RealEnv::RealEnv() 
    : smoother(control_freq) {}

void RealEnv::start(error &err) {
    // cam_side.open("side", "/dev/v4l/by-id/usb-046d_C922_Pro_Stream_Webcam_EB2FE25F-video-index0", 1280, 720, 30, 15, err);
    // if (err) {
    //     return;
    // }

    // cam_wrist.open("wrist", "/dev/v4l/by-id/usb-046d_Logitech_Webcam_C930e_81B986FE-video-index0", 1280, 720, 30, 35, err);
    // if (err) {
    //     return;
    // }

    // cam_side.start_async();
    // cam_wrist.start_async();

    // print "waiting for webcams...";
    // cam_side.await_ready();
    // cam_wrist.await_ready();

    // gello
    
    // CameraFeed feed1;
    // CameraFeed feed2;
    // feed2 = std::move(feed1);
    
    int id = 1;
    print "starting webcams...";
    for (CameraConfig &cfg : task_config.cameras) {
        std::shared_ptr<CameraFeed> &cam = cameras.emplace_back(std::make_shared<CameraFeed>());
        cam->open(fmt::sprintf("cam%d", id), cfg.device_path, 1920, 1080, 30, -1, err);
        if (err) {
            return;
        }
        id++;
        
        cam->start_async();
    }

    print "waiting for webcams...";
    for (std::shared_ptr<CameraFeed> &cam : cameras) {
        cam->await_ready();
    }
    
    print "connecting to gello...";
    str gello_device_path = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT8ISNE2-if00-port0";
    gello.connect(gello_device_path, err);
    if (err) {
        return;
    }

    // arm
    str ar4_device_path = "/dev/serial/by-id/usb-Teensyduino_USB_Serial_13756360-if00";
    arm.connect(ar4_device_path, err);
    if (err) {
        return;
    }

    // observations.reserve(usize(max_steps));
}

bool RealEnv::run(UserInput &user_input, error &err) {
    bool ok = prepare(user_input, err);
    if (err) {
        return false;
    }

    if (!ok) {
        tts::say("canceled");
        return false;
    }

    time::LoopTimer loop_timer(control_freq);
    loop_timer.start();

    for (int i = 0; i < max_steps; i++) {
        print "step", i;

        step(err);
        if (err) {
            return false;
        }

        if (!user_input.get_foot_pedal()) {
            tts::say("END");
            return true;
        }

        time::duration delay = loop_timer.delay();
        if (delay > control_freq) {
            print "delay was", delay.nsecs / 1e6;
            err("loop took too long; delay was %v", delay);
            return false;
        }
    }

    tts::say("END");
    return true;
}

bool RealEnv::prepare(UserInput &user_input, error &err) {
    observations.clear();

    tts::say("waiting for pedal");
    user_input.await_foot_pedal();

    move_to_start_position(err);
    if (err) {
        return false;
    }
    
    bool ok = gello.start(&user_input, start_pose, err);
    if (!ok || err) {
        return false;
    }

    smoother.reset();
    return true;
}

void RealEnv::move_to_start_position(error &err) {
    print "arm moving to start position";
    robot::robot_move_abs(robot::ar4, arm.client, {
            .j1 = {.enable = true, .position_degrees = start_pose[0]},
            .j2 = {.enable = true, .position_degrees = start_pose[1]},
            .j3 = {.enable = true, .position_degrees = start_pose[2]},
            .j4 = {.enable = true, .position_degrees = start_pose[3]},
            .j5 = {.enable = true, .position_degrees = start_pose[4]},
            .j6 = {.enable = true, .position_degrees = start_pose[5]},
            .await = true,
        }, err);
}

void RealEnv::step(error &err) {
    time::monotime start = time::mono();
    robot::Pose curr_pose   = arm.get_pose(err);
    if (err) {
        return;
    }
    print "arm get pose", (time::mono() - start).nsecs / 1e6;
    start = time::mono();

    robot::Pose target_pose = gello.get_current_pose(err);
    if (err) {
        return;
    }
    print "gello pose", (time::mono() - start).nsecs / 1e6;

    start = time::mono();
    Observation ob = {
        // .cam_side     = cam_side.current_frame()->frame_raw.clone(),
        // .cam_wrist    = cam_wrist.current_frame()->frame_raw.clone(),
        .current_pose = curr_pose,
        .target_pose  = target_pose,
    };
    for (std::shared_ptr<CameraFeed> &cam : cameras) {
        ob.cams.append( cam->current_frame()->frame_raw.clone() );
    }
    print "frame clone time", (time::mono() - start).nsecs / 1e6;

    start = time::mono();
    smoother.move(arm, target_pose, curr_pose, err);
    if (err) {
        return;
    }
    print "arm move", (time::mono() - start).nsecs / 1e6;

    observations.push_back(ob);
};

// void RealEnv::move_arm(robot::Pose const& pose, error &err) {
//     robot::MoveJointsAbsRequest req {};
//     const float64 m = 4.0;

//     req.j1 = {
//         .enable = true,
//         .position_degrees = pose[0],
//         .speed_degrees_per_second = 25*m,

//         // .speed_in_degrees_per_second = 0.1,
//         // .speed_out_degrees_per_second = 0.1,

//         // .acceleration_mulitpiler = m,
//     };
//     req.j2 = {
//         .enable = true,
//         .position_degrees = pose[1],
//         .speed_degrees_per_second = 25*m,

//         // .speed_in_degrees_per_second = 0.1,
//         // .speed_out_degrees_per_second = 0.1,

//         .acceleration_mulitplier = 1,
//     };
//     req.j3 = {
//         .enable = true,
//         .position_degrees = pose[2],
//         .speed_degrees_per_second = 25*m,

//         // .speed_in_degrees_per_second = 0.1,
//         // .speed_out_degrees_per_second = 0.1,

//         // .acceleration_mulitpiler = m,
//     };
//     req.j4 = {
//         .enable = true,
//         .position_degrees = pose[3],
//         .speed_degrees_per_second = 25*2*m,
//         .acceleration_mulitplier = /*2**/m,
//     };
//     req.j5 = {
//         .enable = true,
//         .position_degrees = pose[4],
//         .speed_degrees_per_second = 25*2*m,
//         .acceleration_mulitplier = /*2**/m,
//     };
//     req.j6 = {
//         .enable = true,
//         .position_degrees = pose[5],
//         .speed_degrees_per_second = 25*4*m,
//         .acceleration_mulitplier = 2*m,
//     };
    
//     if (!dry_run) {
//         arm.move_joints(req, err);
//         if (err) {
//             return;
//         }
//     }
// }

 void RealEnv::shutdown() {
    // cam_side.stop();
    // cam_wrist.stop();

    gello.stop();
    arm.disconnect();
}
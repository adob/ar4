#include <QCommandLineParser>
#include <QCoreApplication>
#include <qcommandlineparser.h>
#include <qcontainerfwd.h>

#include "robot/arm.h"
#include "robot/kinematics.h"

#include "lib/debug/debug.h"
#include "lib/print.h"

using namespace lib;

static void home_cmd(robot::AR4Arm &arm, QStringList args) {
    robot::HomeRequest req;

    if (args.length() == 0) {
        eprint "no joints specified";
        return;
    }

    for (QString const& arg : args) {
        if (arg == "j1") {
            req.j1 = true;
        } else if (arg == "j2") {
            req.j2 = true;
        } else if (arg == "j3") {
            req.j3 = true;
        } else if (arg == "j4") {
            req.j4 = true;
        } else if (arg == "j5") {
            req.j5 = true;
        } else if (arg == "j6") {
            req.j6 = true;
        } else {
            eprint "unkown joint", arg;
            return;
        }
    }

    arm.client.home(req, error::panic);
}

static void autohome_cmd(robot::AR4Arm &arm) {
    arm.client.home({ .j1 = true}, error::panic);
    robot::robot_move_abs(robot::ar4, arm.client, {
        .j1 = {.enable = true, .position_degrees = 90},
    }, error::panic);

    arm.client.home({ .j2 = true, .j3 = true}, error::panic);
    robot::robot_move_abs(robot::ar4, arm.client, {
        .j2 = {.enable = true},
        .j3 = {.enable = true},
        .await = true,
    }, error::panic);


    arm.client.home({ .j4 = true }, error::panic);
    robot::robot_move_abs(robot::ar4, arm.client, {
        .j4 = {.enable = true},
        .await = true,
    }, error::panic);

    arm.client.home({ .j6 = true }, error::panic);

    arm.client.home({ .j5 = true }, error::panic);
    robot::robot_move_abs(robot::ar4, arm.client, {
        .j5 = {.enable = true},
        .j6 = {.enable = true},
        .await = true,
    }, error::panic);
}

static void status_cmd(robot::AR4Arm &arm) {
    arm.client.log_state(error::panic);
    robot::StateResponse resp = arm.client.get_state(error::panic);

    robot::Pose pose = robot::get_pose(robot::ar4, resp);
    print "joint angles", pose;

    robot::Pose cartesian = robot::forward_kinematics(robot::ar4, pose);
    print "cartesian position", cartesian;
}

static void move_rel_cmd(robot::AR4Arm &arm, QStringList args)  {
    if (args.length() % 2 != 0) {
        eprint "usage: moverel <JOINT> <FLOAT>";
        return;
    }

    robot::Pose p;
    for (size i = 0; i < args.length() - 1; i += 2) {
        QString joint = args[i];
        float64 val = args[i + 1].toDouble();

        if (joint == "j1") {
            p[0] = val;
        } else if (joint == "j2") {
            p[1] = val;
        } else if (joint == "j3") {
            p[2] = val;
        } else if (joint == "j4") {
            p[3] = val;
        } else if (joint == "j5") {
            p[4] = val;
        } else if (joint == "j6") {
            p[5] = val;
        }
    }
    
    print "moverel", p;
    arm.move_joints_rel(p, true, error::panic);
}

void move_abs_cmd(robot::AR4Arm &arm, QStringList args) {
    if (args.length() % 2 != 0) {
        eprint "usage: moveabs <JOINT> <FLOAT>";
        return;
    }

    float64 speed = 25;
    float64 acceleration = 25;

    robot::MoveJointsAbsRequest req {};
    for (size i = 0; i < args.size() - 1; i += 2) {
        QString joint = args[i];
        float64 val = args[i+1].toDouble();

        if (joint == "j1") {
            req.j1 = {
                .enable = true,
                .position_degrees = val,
                .speed_degrees_per_second = speed,
                .acceleration = acceleration,
            };
        } else if (joint == "j2") {
            req.j2 = {
                .enable = true,
                .position_degrees = val,
                .speed_degrees_per_second = speed,
                .acceleration = acceleration,
            };
        } else if (joint == "j3") {
            req.j3 = {
                .enable = true,
                .position_degrees = val,
                .speed_degrees_per_second = speed,
                .acceleration = acceleration,
            };
        } else if (joint == "j4") {
            req.j4 = {
                .enable = true,
                .position_degrees = val,
                .speed_degrees_per_second = speed,
                .acceleration = acceleration,
            };
        } else if (joint == "j5") {
            req.j5 = {
                .enable = true,
                .position_degrees = val,
                .speed_degrees_per_second = speed,
                .acceleration = acceleration,
            };
        } else if (joint == "j6") {
            req.j6 = {
                .enable = true,
                .position_degrees = val,
                .speed_degrees_per_second = speed,
                .acceleration = acceleration,
            };
        } else if (joint == "j1s") {
            req.j1.speed_degrees_per_second = val;
        } else if (joint == "j1a") {
            req.j1.acceleration = val;
        } else if (joint == "m") {
            speed *= val;
            acceleration *= val;
        } else {
            eprint "unkown joint", joint;
            return;
        }
    }

    req.await = true;

    print "pos1", req.j1.position_degrees;

    print "sending move command";
    robot::robot_move_abs(robot::ar4, arm.client, req, error::panic);
    print "done";
}

void move_lin_cmd(robot::AR4Arm &arm, QStringList args) {
    if (args.length() != 6) {
        eprint "expected 6 args";
        return;
    }

    float64 x = args[0].toDouble();
    float64 y = args[1].toDouble();
    float64 z = args[2].toDouble();
    float64 rz = args[3].toDouble();
    float64 ry = args[4].toDouble();
    float64 rx = args[5].toDouble();

    robot::Pose target_xyz = { x, y, z, rz, ry, rx };

    arm.move_linear(target_xyz, error::panic);
}

int main(int argc, char *argv[]) {
    debug::init();

    QCommandLineParser parser;
    parser.setOptionsAfterPositionalArgumentsMode(QCommandLineParser::ParseAsPositionalArguments);
    parser.addOption({
        "ar4", 
        "Path to use for robot connection [default = /dev/serial/by-id/usb-Teensyduino_USB_Serial_13756360-if00]", 
        "ar4", 
        "/dev/serial/by-id/usb-Teensyduino_USB_Serial_13756360-if00"});

    parser.addPositionalArgument("command", "The command to execute");
    parser.setApplicationDescription(
        "\nValid commands:\n"
        "  home [j1] [j2] [j3] [j4] [j5] [j6]\n"
        "  autohome\n"
        "  status\n"
        "  moverel <JOINT> <FLOAT> [<JOINT> <FLOAT>]*; position is expressed in degrees\n"
        "  moveabs <JOINT> <FLOAT> [<JOINT> <FLOAT>]*; position is expressed in degrees\n"
        "  movelin x y z rz ry rx; linear (in a straigh line) movement in Cartesian space");

    QCoreApplication app(argc, argv);
    parser.process(app);

    
    if (parser.positionalArguments().size() == 0) {
        eprint parser.helpText();
        return 1;
    }

    String ar4_device = parser.value("ar4").toStdString();
    print "using device path", ar4_device;

    robot::AR4Arm arm;
    arm.connect(ar4_device, error::panic);
    
    String cmd       = parser.positionalArguments()[0].toStdString();
    QStringList args = parser.positionalArguments().sliced(1);

    if (cmd == "home") {
        home_cmd(arm, args);
    } else if (cmd == "autohome") {
        autohome_cmd(arm);
    } else if (cmd == "status") {
        status_cmd(arm);
    } else if (cmd == "moverel") {
        move_rel_cmd(arm, args);
    } else if (cmd == "moveabs") {
        move_abs_cmd(arm, args);
    } else if (cmd == "movelin")  {
        move_lin_cmd(arm, args);
    } else {
        eprint parser.helpText();
        return 1;
    }
}
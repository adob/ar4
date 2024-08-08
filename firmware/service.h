#pragma once
#include "interface/interface.h"
#include "lib/base.h"
#include "serialrpc/server.h"

#include "robot.h"

namespace robot {
    using namespace lib;

    struct Service : serialrpc::Service {
        Robot &robot;

        Service(Robot &robot): robot(robot) {}

        SumResponse sum(SumRequest const&, error &err);
        void blink(BlinkRequest const&, error &err);
        void move_joints_rel(MoveJointsRelRequest const&, error&);
        void move_joints_abs(MoveJointsAbsRequest const&, error&);
        void log_state(error&);
        StateResponse get_state(error&);
        void home(HomeRequest const& req, error &);
        void set_debug(SetDebugReqeust const&, error&);
        void test_cmd(TestCmdRequest const&, error&);
        void emergency_stop();

        void handle(serialrpc::CallCtx &ctx, int method_id, str msg, error &err) override;
        void start() override;
    } ;
}

#pragma once
#include "interface/interface.h"
#include "serialrpc/client.h"

namespace robot {
    using namespace lib;

    struct Client {
        std::shared_ptr<serialrpc::Client> rpc;

        SumResponse sum(SumRequest const& req, error &err);
        void blink(int duration_ms, error &err);
        void move_joints_rel(MoveJointsRelRequest const& req, error &err);
        void move_joints_abs(MoveJointsAbsRequest const& req, error &err);
        
        void log_state(error &err);
        StateResponse get_state(error &err);
        void set_debug(SetDebugReqeust const&, error&);
        void test_cmd(TestCmdRequest const&, error&);

        void home(HomeRequest const& req, error &err);
    } ;
}

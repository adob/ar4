#include "client.h"

#include "interface/interface.h"
#include "lib/print.h"

using namespace lib;
using namespace robot;

SumResponse Client::sum(SumRequest const& req, error &err) {
    SumResponse resp;
    rpc->call(Sum, req, resp, err);
    return resp;
}

void Client::blink(int duration_ms, error &err) {
    BlinkRequest req { duration_ms };
    rpc->call(Blink, req, nil, err);
    return;
}

void Client::move_joints_rel(MoveJointsRelRequest const& req, error &err) {
    rpc->call(MoveJointsRel, req, nil, err);
    return;
}

void Client::move_joints_abs(MoveJointsAbsRequest const& req, error &err) {
    rpc->call(MoveJointsAbs, req, nil, err);
    return;
}

void Client::log_state(error &err) {
  rpc->call(LogState, nil, nil, err);
}

StateResponse Client::get_state(error &err) {
    StateResponse resp;
    rpc->call(GetState, nil, resp, err);
    return resp;
}

void Client::set_debug(SetDebugReqeust const& req, error &err) {
    rpc->call(SetDebug, req, nil, err);
}

void Client::test_cmd(TestCmdRequest const& req, error &err) {
    rpc->call(TestCmd, req, nil, err);
}

void Client::home(HomeRequest const& req, error &err) {
    rpc->call(Home, req, nil, err);
}


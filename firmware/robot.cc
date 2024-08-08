#include "robot.h"
#include <core_pins.h>
#include <cmath>

#include "interface/interface.h"
#include "math.h"
#include "lib/print.h"

using namespace lib;
using namespace robot;

Robot::Robot() {}

void Robot::init() {
    timer.attach(j1.stepper);
    timer.attach(j2.stepper);
    timer.attach(j3.stepper);
    timer.attach(j4.stepper);
    timer.attach(j5.stepper);
    timer.attach(j6.stepper);
    timer.start();
}

void Robot::move_joints_rel(MoveJointsRelRequest const& req) {
    if (req.j1 != 0) {
        j1.move_rel(req.j1);
    }

    if (req.j2 != 0) {
        j2.move_rel(req.j2);
    }

    if (req.j3 != 0) {
        j3.move_rel(req.j3);
    }

    if (req.j4 != 0) {
        j4.move_rel(req.j4);
    }

    if (req.j5 != 0) {
        j5.move_rel(req.j5);
    }

    if (req.j6 != 0) {
        j6.move_rel(req.j6);
    }
}

static MoveJointCommand get_move_command(MoveJointRequest const& req, Motor const& motor) {
    MoveJointCommand cmd;
    if (!req.enable || std::isnan(req.position_degrees)) {
        cmd.step_target = -1;
        return cmd;
    }

    cmd.step_target = motor.pos2step(req.position_degrees);
    cmd.speed = req.speed_degrees_per_second * motor.steps_per_degree;
    cmd.speed_in = req.speed_in_degrees_per_second * motor.steps_per_degree;
    cmd.speed_out = req.speed_out_degrees_per_second * motor.steps_per_degree;
    cmd.acceleration = req.acceleration * motor.steps_per_degree;
    cmd.deceleration = req.deceleration * motor.steps_per_degree;

    return cmd;
}

static bool use_alt(int j4a_step, int j4b_step, int j5a_step, int j5b_step, int j6a_step, int j6b_step, Robot &robot) {
    if (j4b_step > robot.j4.max_step) {
        return false;
    }

    if (j5b_step > robot.j5.max_step) {
        return false;
    }

    if (j6b_step > robot.j6.max_step) {
        return false;
    }

    if (j4a_step > robot.j4.max_step || j5a_step < 0) {
        return true;
    }

    if (j5a_step > robot.j5.max_step || j5a_step < 0) {
        return true;
    }

    if (j6a_step > robot.j6.max_step || j6a_step < 0) {
        return true;
    }

    int j4_cur = robot.j4.get_position();
    int dist_a = abs(j4_cur - j4a_step);
    int dist_b = abs(j4_cur - j4b_step);
    return dist_b < dist_a;
}

void Robot::move_joints_abs(MoveJointsAbsRequest const& req) {
    if (req.j1.enable && !std::isnan(req.j1.position_degrees)) {
        j1.move_abs(req.j1);
    }

    if (req.j2.enable && !std::isnan(req.j2.position_degrees)) {
        j2.move_abs(req.j2);
    }

    if (req.j3.enable && !std::isnan(req.j3.position_degrees)) {
        j3.move_abs(req.j3);
    }

    MoveJointCommand j4_cmd = get_move_command(req.j4, j4);
    MoveJointCommand j5_cmd = get_move_command(req.j5, j5);
    MoveJointCommand j6_cmd = get_move_command(req.j6, j6);
    if (req.has_alt_wrist) {
        int j4a_step = j4_cmd.step_target;
        int j4b_step = !std::isnan(req.j4b) ? j4.pos2step(req.j4b) : -1;
        // print "j4a_step", j4a_step;
        // print "j4b_step", j4b_step;

        int j5a_step = j5_cmd.step_target;
        int j5b_step = !std::isnan(req.j5b) ? j5.pos2step(req.j5b) : -1;
        // print "j5a_step", j5a_step;
        // print "j5b_step", j5b_step;

        int j6a_step = j6_cmd.step_target;
        int j6b_step = !std::isnan(req.j6b) ? j6.pos2step(req.j6b) : -1;
        // print "j6a_step", j6a_step;
        // print "j6b_step", j6b_step;

        if (use_alt(j4a_step, j4b_step, j5a_step, j5b_step, j6a_step, j6b_step, *this)) {
            // print "using alt";
            j4_cmd.step_target = j4b_step;
            j5_cmd.step_target = j5b_step;
            j6_cmd.step_target = j6b_step;
        }
    }

    if (j4_cmd.step_target != -1) {
        j4.move_to(j4_cmd.step_target, j4_cmd.speed, j4_cmd.acceleration, j4_cmd.deceleration, j4_cmd.speed_in, j4_cmd.speed_out, req.j4.target_time);
    }

    if (j5_cmd.step_target != -1) {
        j5.move_to(j5_cmd.step_target, j5_cmd.speed, j5_cmd.acceleration, j5_cmd.deceleration, j5_cmd.speed_in, j5_cmd.speed_out, req.j5.target_time);
    }

    if (j6_cmd.step_target != -1) {
        j6.move_to(j6_cmd.step_target, j6_cmd.speed, j6_cmd.acceleration, j6_cmd.deceleration, j6_cmd.speed_in, j6_cmd.speed_out, req.j6.target_time);
    }

    if (req.await) {
        j1.stepper.await();
        j2.stepper.await();
        j3.stepper.await();
        j4.stepper.await();
        j5.stepper.await();
        j6.stepper.await();
    }
}


void Robot::emergency_stop() {
    j1.stepper.emergency_stop();
    j2.stepper.emergency_stop();
    j3.stepper.emergency_stop();
    j4.stepper.emergency_stop();
    j5.stepper.emergency_stop();
    j6.stepper.emergency_stop();
}

#include "motor.h"
#include <core_pins.h>
#include <cmath>

#include "robot/interface.h"
#include "lib/print.h"

using namespace lib;
using namespace robot;

int MotorCfg::pos2step(float64 pos) const {
    return (int) std::round(steps_per_degree * pos);
}

static int32 get_min_step(int limit_angle, int steps_per_degree, bool reverse_rotation) {
    if (reverse_rotation) {
        return -limit_angle*steps_per_degree;
    }
    
    return 0;
}

static int32 get_max_step(int limit_angle, int steps_per_degree, bool reverse_rotation)
{
    if (reverse_rotation) {
        return 0;
    }

    return limit_angle*steps_per_degree;
}

Motor::Motor(MotorCfg const& cfg)
    : MotorCfg(cfg)
    , stepper(cfg.step_pin, cfg.dir_pin, cfg.encoder_pin1, cfg.encoder_pin2, cfg.microsteps, 
        get_max_step(cfg.limit_angle, cfg.steps_per_degree, cfg.inverse_rotation),
        get_min_step(cfg.limit_angle, cfg.steps_per_degree, cfg.inverse_rotation),
        steps_per_degree)
{
    if (endstop_pin >= 0) {
        pinMode(endstop_pin, INPUT);
    }

    max_step = int(limit_angle * steps_per_degree);

    // if (inverse_rotation) {
    //     stepper.setInverseRotation(true);
    // }
}

int Motor::get_position() {
    int pos = stepper.getPosition();
    if (inverse_rotation) {
        return -pos;
    }

    return pos;
}

float64 Motor::get_angle() {
    return get_position() / steps_per_degree;
}

void Motor::move_rel(float64 delta_degrees) {
    int steps = (int) std::round(delta_degrees * steps_per_degree);
    move_rel_steps(steps);
}

void Motor::move_rel_steps(int steps) {
    if (!homed) {
        print "about to move %s %d steps" % name, steps;

        if (inverse_rotation) {
            steps = -steps;
        }
        stepper.moveRel(steps);
        return;
    }

    int curpos = get_position();
    int destpos = curpos + steps;
    if (destpos < 0) {
        print "clamping destpos to 0, was", destpos;
        destpos = 0;

    } else if (destpos > max_step) {
        print "clamping destpos to max_step (%d), was %d" % max_step, destpos;
        destpos = max_step;
    }

    print "about to move %s %d steps, new angle will be %d" % name, (destpos - curpos),
        destpos / steps_per_degree;

    if (inverse_rotation) {
        destpos = -destpos;
    }
    print "moveAbs(%d)" % destpos;
    stepper.moveAbs(destpos);
}

bool check_endstop(int pin) {
    for (int i = 0; i < 10; i++) {
        if (digitalRead(pin) == LOW) {
            return false;
        }
        delay(1);
    }
    return true;
}

void Motor::move_to(int step_target, int speed, int acceleration, int deceleration, int speed_in, int speed_out, float64 target_time) {
    if (!homed) {
        print "ignoring move request, not homed";
        return;
    }

    if (step_target < 0) {
        print "clamping step_target to 0, was", step_target;
        step_target = 0;
    }

    if (step_target > max_step) {
        print "clamping step_target to max_step (%d), was %d" % max_step, step_target;
        step_target = max_step;
    }

    if (inverse_rotation) {
        step_target = -step_target;
    }

    print "move_abs %s step_target %d; speed %v; acceleration %v; speed_in %d; speed_out %d " % name, step_target, speed, acceleration, speed_in, speed_out;
    stepper.moveAbsAsync(step_target, speed, acceleration, deceleration, speed_in, speed_out, target_time);
}

void Motor::move_abs(MoveJointRequest const& req) {
    float64 delta_target = req.position_degrees;

    if (!homed) {
        print "ignoring move request, not homed";
        return;
    }

    int step_target = pos2step(delta_target);
    if (step_target < 0) {
        print "clamping step_target to 0, was", step_target;
        step_target = 0;
    }
    if (step_target > max_step) {
        print "clamping step_target to max_step (%d), was %d" % max_step, step_target;
        step_target = max_step;
    }
    if (inverse_rotation) {
        step_target = -step_target;
    }

    float64 speed = req.speed_degrees_per_second * steps_per_degree;
    float64 speed_in = req.speed_in_degrees_per_second * steps_per_degree;
    float64 speed_out = req.speed_out_degrees_per_second * steps_per_degree;
    float64 acceleration = req.acceleration * steps_per_degree;
    float64 deceleration = req.deceleration * steps_per_degree;


     print "move_abs %s delta_target %d; step_target %d; speed %v; acceleration %v; speed_in %v; speed_out %v" % name, delta_target, step_target, speed, acceleration, speed_in, speed_out;
     stepper.moveAbsAsync(step_target, speed, acceleration, deceleration, speed_in, speed_out, req.target_time);
}

bool Motor::home_drive(int max_steps, int speed) {
    for (;;) {
        endstop_reached = false;

        stepper.moveUntil(max_steps, [&] { return homing_callback(); }, speed);

        if (!endstop_reached) {
            print "WARN: did not reach endstop within step limit";
            return false;
        }

        if (!check_endstop(endstop_pin)) {
            print "WARN: endstop bounce, trying again";
            continue;
        }

        return true;
    }

    print "WARN: tried too many times;";
    return false;
}

void Motor::home() {
    int max_steps = (int) std::round(365 * steps_per_degree);
    if (!inverse_rotation) {
        max_steps = -max_steps;
    }

    homed           = false;

    // int start_position = stepper.getPosition();

    print "driving home...";
    backoff();
    bool ok = home_drive(max_steps);
    if (!ok) {
        return;
    }

    // backoff
    for (int i = 0; i < 3; i++) {
        print "backing off...";
        // backoff();
        int steps_5deg = (int)std::round(5 * steps_per_degree);
        move_rel_steps(steps_5deg);
        int32 backoff_position = stepper.current_state->current_position;
        print "backoff position", backoff_position;
        
        // move_rel(5);

        // print "delaying 0.5s...";
        //delay(500);

        // drive in again
        print "driving home again...";
        ok = home_drive(max_steps, 500);
        if (!ok) {
            return;
        }
        int32 homed_position = stepper.current_state->current_position;
        int32 dist = homed_position - backoff_position;
        print "travel dist", dist;
    }
    
    homed = true;
    //int end_position = stepper.getPosition();
    //int moved_steps = abs(end_position - start_position);
    //print "move steps", moved_steps;
    stepper.setPosition(0);
}

void Motor::backoff() {
    int max_steps = (int) std::round(10 * steps_per_degree);
    if (!inverse_rotation) {
        max_steps = -max_steps;
    }

    stepper.moveUntil(-max_steps, [&] { return backoff_callback(); });
}

// called in ISR
bool Motor::homing_callback() {

    int true_count = 0;
    for (int i = 0; i < 100; i++) {

        bool endstop = digitalReadFast(endstop_pin) == HIGH;
        if (endstop) {
            true_count++;
        }
    }

    if (true_count >= 80) {
        endstop_reached = true;
        stepper.setPosition(0);
        return false;
    }

    return true;
}

bool Motor::backoff_callback() {
    int true_count = 0;
    for (int i = 0; i < 100; i++) {
        bool endstop = digitalReadFast(endstop_pin) == LOW;
        if (endstop) {
            true_count++;
        }
    }

    if (true_count >= 80) {
        return false;
    }

    return true;
}

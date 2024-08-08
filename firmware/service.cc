#include <Arduino.h>
#include <core_pins.h>

#include "service.h"
#include "interface/interface.h"
#include "stepper.h"
#include "lib/print.h"

using namespace lib;
using namespace robot;

void Service::start() {
    robot.init();
}

void Service::handle(serialrpc::CallCtx &ctx, int method_id, str msg, error &err) {
    MethodID id = MethodID(method_id);

    switch (id) {
        case Sum:
            handle_method(ctx, *this, &Service::sum, msg, err);
            break;

        case Blink:
            handle_method(ctx, *this, &Service::blink, msg, err);
            break;

        case MoveJointsRel:
            handle_method(ctx, *this, &Service::move_joints_rel, msg, err);
            break;

        case MoveJointsAbs:
            handle_method(ctx, *this, &Service::move_joints_abs, msg, err);
            break;

        case LogState:
            handle_method(ctx, *this, &Service::log_state, err);
            break;

        case GetState:
            handle_method(ctx, *this, &Service::get_state, err);
            break;

        case Home:
            handle_method(ctx, *this, &Service::home, msg, err);
            break;

        case SetDebug:
            handle_method(ctx, *this, &Service::set_debug, msg, err);
            break;
            
        case TestCmd:
            handle_method(ctx, *this, &Service::test_cmd, msg, err);
    }
}

SumResponse Service::sum(SumRequest const &req, error&) {
    print "Service::sum()";
    return {
        answer: req.left + req.right,
    };
}

void Service::blink(BlinkRequest const& req, error&) {
    unsigned long start = millis();

    pinMode(LED_BUILTIN, OUTPUT);
    for (;;) {

        digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
        delay(1000);                      // wait for a second
        digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
        delay(1000);

        unsigned long elapsed = millis() - start;
        if (elapsed >= (unsigned long) req.duration_ms) {
            break;
        }

    }
}

void Service::move_joints_rel(MoveJointsRelRequest const& req, error&) {
    print "Service::move_joints_rel begin";

    robot.move_joints_rel(req);

    print "Service::move_joints_rel done";
}

void Service::move_joints_abs(MoveJointsAbsRequest const& req, error&) {
    robot.move_joints_abs(req);
}

void Service::emergency_stop() {
    robot.emergency_stop();
}


void Service::log_state(error&) {
    for (Motor *motor : robot.motors) {
        print "%s homed:    %d" % motor->name, motor->homed;
        print "%s position: %d" % motor->name, motor->get_position();
        print "%s max_step: %d" % motor->name, motor->max_step;
        print "%s angle:    %f" % motor->name, motor->get_angle();
        print "%s endstop:  %v" % motor->name, (bool) digitalRead(motor->endstop_pin);

        print "%s nonce:             %s" % motor->name, motor->stepper.isr_data.nonce;
        print "%s reset_pin:         %s" % motor->name, motor->stepper.isr_data.reset_pin;
        print "%s current_position:  %s" % motor->name, motor->stepper.isr_data.current_position;
        print "%s current_direction: %s" % motor->name, motor->stepper.isr_data.current_direction;
        print "%s speed:             %s" % motor->name, motor->stepper.isr_data.speed;
        print "%s speed_squared:     %s" % motor->name, motor->stepper.isr_data.speed_squared;
        print "%s target_speed:      %s" % motor->name, motor->stepper.isr_data.target_speed;
        print "%s target_tick:       %s" % motor->name, motor->stepper.isr_data.target_tick;
        print "%s tick:              %s" % motor->name, motor->stepper.isr_data.tick;
        print "%s step_count:        %s" % motor->name, motor->stepper.isr_data.step_count;
        print "%s step:              %s" % motor->name, motor->stepper.isr_data.step;
        print "%s acceleration_2x:   %s" % motor->name, motor->stepper.isr_data.acceleration_2x;
        // print "%s direction_change:  %s" % motor->name, motor->stepper.isr_data.direction_change;
        print "%s deceleration_start: %s" % motor->name, motor->stepper.isr_data.deceleration_start;
        print "%s acceleration_end:  %s" % motor->name, motor->stepper.isr_data.acceleration_end;
        print "%s max_delay:         %s" % motor->name, motor->stepper.max_delay;

        if (motor->stepper.encoder.enabled) {
             print "%s encoder:           %s" % motor->name, motor->stepper.encoder.read();
        }

        print "";
    }
}

StateResponse Service::get_state(error&) {
    return StateResponse {
        j1_homed: robot.j1.homed,
        j1_angle: robot.j1.get_angle(),

        j2_homed: robot.j2.homed,
        j2_angle: robot.j2.get_angle(),

        j3_homed: robot.j3.homed,
        j3_angle: robot.j3.get_angle(),

        j4_homed: robot.j4.homed,
        j4_angle: robot.j4.get_angle(),

        j5_homed: robot.j5.homed,
        j5_angle: robot.j5.get_angle(),

        j6_homed: robot.j6.homed,
        j6_angle: robot.j6.get_angle(),
    };
}

void Service::home(HomeRequest const& req, error &) {
    if (req.j2) {
        robot.j2.home();
    }

    if (req.j1) {
        robot.j1.home();
    }

    if (req.j3) {
        robot.j3.home();
    }

    if (req.j4) {
        robot.j4.home();
    }

    if (req.j5) {
        robot.j5.home();
    }

    if (req.j6) {
        robot.j6.home();
    }
}

// extern bool debug;
void Service::set_debug(SetDebugReqeust const& req, error&)  {
    Stepper::debug = req.debug;
}

void Service::test_cmd(TestCmdRequest const&, error&) {
    int step_pin = robot.j6.step_pin;
    
    int32 start_pos = robot.j6.stepper.get_real_position();
    print "test_cmd start_pos", start_pos;
    
    ulong start_us = micros();
    
    digitalWriteFast(step_pin, HIGH);
    delayMicroseconds(3);
    digitalWriteFast(step_pin, LOW);
    delayMicroseconds(3);
    int32 curr_pos;
    ulong delay;

    uint i = 0;
    for (;;) {
        curr_pos = robot.j6.stepper.get_real_position();
        if (curr_pos != start_pos) {
            break;
        }
        delay = micros() - start_us;
        if (delay > 1'000'000) {
            print "delay too long";
            return;
        }
        i++;
    }
    
    print "j6 delay %dus; count %d; final_pos %d" % delay, i, curr_pos;
    
}
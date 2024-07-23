#pragma once

#include "lib/base.h"
#include "robot/interface.h"
#include "stepper.h"
#include "encoder.h"


namespace robot {
    using namespace lib;
    
    struct MotorCfg {
        str name;

        int step_pin    = -1;
        int dir_pin     = -1;
        int endstop_pin = -1;

        int encoder_pin1 = -1;
        int encoder_pin2 = -1;

        int microsteps = 1;
        double steps_per_degree = -1;

        double limit_angle = 0;
        bool inverse_rotation = false;

        int pos2step(float64 pos) const;
    } ;

    struct Motor : MotorCfg {
        Stepper stepper;

        bool homed = false;
        int max_step;

        Motor(MotorCfg const&);

        void move_rel(float64 degrees);
        void move_rel_steps(int steps);
        void move_abs(MoveJointRequest const& req);
        void move_to(int step, int speed, int acceleration, int deceleration, int speed_in, int speed_out, float64 target_time);
        void home();

        int get_position();
        float64 get_angle();

      private:
        volatile bool endstop_reached;
        void backoff();
        bool homing_callback();
        bool backoff_callback();
        bool home_drive(int max_steps, int speed = 0);
    } ;
}

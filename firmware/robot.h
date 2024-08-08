#pragma once

#include "lib/base.h"
#include "lib/array.h"
#include "interface/interface.h"
#include "motor.h"
#include "stepper_timer.h"

namespace robot {
    using namespace lib;

    struct MoveJointCommand {
        int step_target = 0;
        // int step_target_alt = -1;
        float64 speed = 0;
        float64 speed_in = 0;
        float64 speed_out = 0;
        float64 acceleration = 0;
        float64 deceleration = 0;
    } ;

    struct MoveJointsCommand {
        MoveJointCommand j1, j2, j3, j4, j5, j6;
    } ;

    struct Robot {

        StepperTimer timer;

        // driver
        // https://www.omc-stepperonline.com/download/DM542T.pdf
        Motor j1 {{
            // 17HS15-1684D-HG10-AR4
            name:     "J1",
            step_pin: 0,
            dir_pin:  1,
            endstop_pin: 26,

            encoder_pin1: 14,
            encoder_pin2: 15,

            // step_angle: 1.80,
            // gear_ratio: 10,
            // belt_ratio: 4
            microsteps: 2,

            // driver_peak_current: 1.46A
            // rated_amps : 1.68

            // step_fration = 1 / (step_angle / gear_ratio / belt_ratio / microsteps)
            // steps_per_degree: (1/step_angle) * gear_ratio * belt_ratio * microsteps
            steps_per_degree: 16'000 / 360.0,  // 44.444

            limit_angle: 340,
            inverse_rotation: true,
        }};

         Motor j2 {{
            name:     "J2",

            step_pin: 2,
            dir_pin:  3,
            endstop_pin: 27,

            encoder_pin1: 16,
            encoder_pin2: 17,

            // step_angle: 1.80
            // gear ratio: 50
            microsteps: 2,

            steps_per_degree: 20'000 / 360.0,

            limit_angle: 132,
            inverse_rotation: true,
        }};

        Motor j3 {{
            name:     "J3",

            // 17HS15-1684D-HG50-AR4
            step_pin: 4,
            dir_pin:  5,
            endstop_pin: 28,

            encoder_pin1: 18,
            encoder_pin2: 19,

            // step_angle: 1.80,
            // gear_ratio: 50,
            // belt_ratio: n/a
            microsteps: 2,

            steps_per_degree: 20'000 / 360.0,

            limit_angle: 141 + 10,
        }};

        Motor j4 {{
            name:     "J4",
            // 17HS15-1684D-HG50-AR4
            step_pin: 6,
            dir_pin:  7,
            endstop_pin: 29,

            encoder_pin1: 20,
            encoder_pin2: 21,

            // step_angle: 1.80
            // gear_ratio: 13 + (212/289)
            // belt_ratio: 2.8
            microsteps: 2,

            steps_per_degree: 42.72664359861591,

            limit_angle: 330,
        }};


        Motor j5 = {{
            name:     "J5",

            step_pin: 8,
            dir_pin:  9,
            endstop_pin: 30,

            encoder_pin1: 22,
            encoder_pin2: 23,

            // step_angle: 1.80,
            // gear_ratio: 1,
            microsteps: 4,
            steps_per_degree: 21.86024888,

            limit_angle: 210,
            inverse_rotation: true,
        }};

        Motor j6 = {{
            name:     "J6",

            step_pin: 10,
            dir_pin:  11,
            endstop_pin: 31,

            encoder_pin1: 25,
            encoder_pin2: 24,

            // step_angle: 1.80
            // gear_ratio: 19+38/187
            microsteps: 2,
            steps_per_degree: 1.0 / (1.80 / (19 + 38.0/187) / 2),  // 21.336898395721924

            limit_angle: 310,
        }};

        Array<Motor*, 6> motors = {{&j1, &j2, &j3, &j4, &j5, &j6}};

        Robot();
        void init();
        void move_joints_rel(MoveJointsRelRequest const& req);
        void move_joints_abs(MoveJointsAbsRequest const& req);
        void emergency_stop();
    };
}

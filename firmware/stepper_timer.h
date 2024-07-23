#pragma once

#include "TeensyTimerTool.h"
#include "stepper.h"

namespace robot {
    struct StepperTimer {
        TeensyTimerTool::PeriodicTimer timer;
        Stepper *first = nullptr;
        bool started = false;

        StepperTimer();
        void start();
        void attach(Stepper &stepper);

        void timer_callback();
    } ;
}
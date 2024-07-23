#include "stepper_timer.h"

using namespace lib;
using namespace robot;

StepperTimer::StepperTimer()
    : timer(TeensyTimerTool::TMR1) {

}

void StepperTimer::attach(Stepper &stepper) {
    stepper.timer = this;

    if (!first) {
        first = &stepper;
        return;
    }

    Stepper *last = first;
    while (last->next != nullptr) {
        last = last->next;
    }

    last->next = &stepper;
}

void StepperTimer::start() {
    if (started) {
        return;
    }
    started = true;

    timer.setPrescaler(1);
    TeensyTimerTool::errorCode err = timer.begin([&] { timer_callback(); }, 5);
    if (err != TeensyTimerTool::errorCode::OK) {
        printf("timer.begin error");
        abort();
    }
}

// ISR
void StepperTimer::timer_callback() {
    for (Stepper *stepper = first; stepper != nullptr; stepper = stepper->next) {
        stepper->ISR();
    }
}

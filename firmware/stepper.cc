#include "stepper.h"

#include <Arduino.h>
#include <algorithm>
#include <cmath>
// #include <cmath>

#include "lib/base.h"
#include "lib/print.h"

using namespace lib;
using namespace robot;

const bool use_encoders = true;


volatile bool Stepper::debug = false;

Stepper::Stepper(int step_pin, int dir_pin, int encoder_pin1, int encoder_pin2, int microsteps, 
        int32 max_step, int32 min_step, int32 steps_per_degree) 
    : stepPin(step_pin)
    , dirPin(dir_pin)
    , max_step(max_step)
    , min_step(min_step)
    , steps_per_degree(steps_per_degree)
    , encoder_ratio(1.0f / (20.f / microsteps)) {

    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);

    if (encoder_pin1 >= 0 && encoder_pin2 >= 0) {
        encoder.attach(encoder_pin1, encoder_pin2);
    }
}

void Stepper::moveAbs(int32 target, uint32 v = 0, std::function<bool()> const& condition) {
    moveAbsAsync(target, v, 0, 100, 100, 0, 0, condition);
    while (!pending_action->motion_complete)
    {
        delay(10);
    }
}

void Stepper::moveAbsAsync(int32 target, uint32 v = 0, uint32 a = 0, int32 deceleration, uint32 speed_in, uint32 speed_out, float64 target_time, std::function<bool()> const& condition = {}) {
    //printf("moveAbsAsync target %d\n", (int) target);
    startMoveTo(target, 0, (v == 0 ? vMax : v), (a > 0 ? a : acc), deceleration, speed_in, speed_out, target_time, condition);
}


void Stepper::moveRel(int32 delta, uint32 v = 0, std::function<bool()> const& condition) {
    moveRelAsync(delta, v, condition);
    while (!pending_action->motion_complete)
    {
        delay(10);
    }
}

void Stepper::moveRelAsync(int32 delta, uint32 v = 0, std::function<bool()> const& condition) {
     int32_t current_position = current_state->current_position;
    // printf("moveRelAsync delta=%d current_position=%d abs=%d\n", (int) delta, (int) current_position, (int) (current_position + delta));
    startMoveTo(current_position + delta, 0, (v == 0 ? std::abs(vMax) : v), acc, 0, 100, 100, 0, condition);
}

void Stepper::moveUntil(int32 maxSteps, std::function<bool()> const& condition, uint32 v = 0) {
    moveRel(maxSteps, v, condition);
}

void Stepper::startMoveTo(int32 _s_tgt, int32 v_e, uint32 v_tgt, uint32 a, int32 deceleration, uint32 speed_in, uint32 speed_out, float64 target_time, std::function<bool()> const& condition) {
    if (v_tgt <= 0) {
        v_tgt = 1;
        return;
    }
    
    if (a <= 0) {
        a = 1;
    }
    
    if (deceleration <= 0) {
        deceleration = a;
    }
    
    int target = _s_tgt;
    *pending_action_tmp = {
        .nonce           = pending_action->nonce + 1,
        .is_moving       = true,
        .target_position = target,
        .target_speed    = v_tgt,
        .speed_in        = speed_in,
        .speed_out       = speed_out,
        .acceleration    = a,
        .deceleration    = deceleration,
        .target_time     = target_time,
        .condition       = condition,
    };

    // if (debug) {
    //     printf("startMoveTo nonce %d\n", pendingAction->nonce);
    // }

    std::swap(pending_action, pending_action_tmp);
}

void Stepper::emergency_stop() {
    *pending_action_tmp = {
        .nonce = pending_action->nonce + 1,
        .is_moving = false,
    };
    std::swap(pending_action, pending_action_tmp);
}

int32 Stepper::getPosition() const {
    return get_real_position();
    //return current_state->current_position;
}

void Stepper::setPosition(int32 p) {
    isr_data.current_position = p;
    if (encoder.enabled) {
        encoder.write(p);
    }
    current_state->current_position = p;
}

void Stepper::await() {
    while (current_state->is_moving || (pending_action->is_moving && !pending_action->motion_complete)) {
        delay(10);
    }
}

// struct MotionPlan {
//     int initial_deceleration_distance = 0;
//     int initial_deceleration          = 0;
//     int reverse_direction             = 0;
//     int acceleration_distance         = 0;
//     int coast_distance                = 0;
//     int deceleration_distance         = 0;
//     int acceleration                  = 0;
// };

struct MotionPlan2 {

    int acceleration_distance = 0;
    int acceleration = 0;

    int coast_distance = 0;

    int deceleration_distance = 0;
    int deceleration = 0;

    int direction_change = 0;
};

struct MotionBlock {
    int acceleration_distance = 0;
    int coast_distance = 0;
    int deceleration_distance = 0;
};

static constexpr int signum(int32 v) {
    return (0 < v) - (v < 0);
}

static int sqr(int a) {
    return a*a;
}

static int get_acceleration_distance(int initial_speed, int target_speed, int acceleration) {
    if (initial_speed >= target_speed) {
        
        if (initial_speed > target_speed) printf("WARN: get_acceleration_distance %d %d %d\n", initial_speed, target_speed, acceleration);
        return 0;
        
        // panic("get_acceleration_distance");
    }

    // distance covered is given the following formula:
    //
    //     v^2 - v0^2
    // d = ----------
    //        2a

    return (sqr(target_speed) - sqr(initial_speed)) / (2 * acceleration);
}

static int get_deceleration_distance(int initial_speed, int target_speed, int acceleration){
    if (initial_speed <= target_speed) {
        if (initial_speed < target_speed) printf("WARN: get_deceleration_distance %d %d %d\n", initial_speed, target_speed, acceleration);
        return 0;
        panic("get_deceleration_distance");
    }

    // distance covered is given the following formula:
    //
    //     v0^2 - v^2
    // d = ----------
    //        2a

    return (sqr(initial_speed) - sqr(target_speed)) / (2 * acceleration);
}

static int get_acceleration(int initial_speed, int target_speed, int distance) {
    if (initial_speed >= target_speed) {
        if (initial_speed > target_speed) printf("WARN: get_acceleration %d %d %d\n", initial_speed, target_speed, distance);
        return 0;
        panic("get_acceleration");
    }
    
    //
    //     v^2 - v0^2
    // a = ----------
    //        2d
    return (sqr(target_speed) - sqr(initial_speed)) / (2*distance);
}

static int get_deceleration(int initial_speed, int target_speed, int distance) {
    if (initial_speed <= target_speed) {
        if (initial_speed < target_speed) printf("WARN: get_deceleration %d %d %d\n", initial_speed, target_speed, distance);
        return 0;
        panic("get_deceleration");
    }
    //
    //     v^2 - v0^2
    // a = ----------
    //        2d
    return (sqr(initial_speed) - sqr(target_speed)) / (2 * distance);
}

static MotionBlock make_motion(int acceleration_distance, int travel_distance) {
    if (acceleration_distance > travel_distance/2) {
        acceleration_distance = travel_distance/2;
    }

    return {
        .acceleration_distance = acceleration_distance,
        .coast_distance        = travel_distance - 2*acceleration_distance,
        .deceleration_distance = acceleration_distance,
    };
}

static MotionBlock make_motion(int acceleration_distance, int deceleration_distance, int travel_distance) {
    if (acceleration_distance + deceleration_distance > travel_distance) {
        int overrun = (acceleration_distance + deceleration_distance) - travel_distance;
        int subt = (overrun+1) / 2;
        
        if (subt > acceleration_distance) {
            acceleration_distance = 0;
        } else {
            acceleration_distance -= subt;
        }
        
        if (subt > deceleration_distance) {
            deceleration_distance = 0;
        } else {
            deceleration_distance -= subt;
        }
        
        if (acceleration_distance > travel_distance) {
            acceleration_distance = travel_distance;
        }
        
        if (deceleration_distance > travel_distance) {
            deceleration_distance  = travel_distance;
        }
    }

    return {
        .acceleration_distance = acceleration_distance,
        .coast_distance        = travel_distance - acceleration_distance - deceleration_distance,
        .deceleration_distance = deceleration_distance,
    };
}


static MotionPlan2 plan_motion(
    int initial_position,
    int initial_direction,
    int initial_speed,
    int initial_deceleration,

    int target_position,
    int target_speed,
    int target_acceleration,
    int target_deceleration,

    int speed_in,
    int speed_out,
    
    int travel_available,
    int reverse_out_speed
) {

    int delta_position = target_position - initial_position;
    // if (delta_position == 0) {
    //     return {};
    // }

    int travel_direction = signum(delta_position);
    int travel_distance  = abs(delta_position);
    
    if (target_speed < speed_in) {
        if (Stepper::debug) printf("L415\n");
        speed_in = target_speed;
    }
    
    if (target_speed < speed_out) {
        if (Stepper::debug) printf("L420\n");
        speed_out = target_speed;
    }

    if (initial_speed < speed_in) {
        // if (Stepper::debug) printf("L425\n");
        initial_speed = speed_in;
    }


    if (travel_direction == initial_direction) {
        
        // make sure we can stop in time
        // TODO this check could be move further down
        if (initial_speed > speed_out) {
            int stopping_distance = get_deceleration_distance(initial_speed, speed_out, target_acceleration);
            
            
            if (stopping_distance > travel_distance) {
                if (Stepper::debug) printf("motion plan: STOPPING DISTANCE > TRAVEL_DISTANCE %d %d %d\n", stopping_distance, travel_distance, stopping_distance - travel_distance);
                // we can't stop in time using target_acceleration

                int deceleration          = get_deceleration(initial_speed, speed_out, travel_distance);
                int deceleration_distance = travel_distance;

                int max_deceleration = std::max(initial_deceleration, target_acceleration);
                if (deceleration <= max_deceleration) {
                    if (Stepper::debug) printf("L456 %d %d\n", deceleration, deceleration_distance);
                    // all good, we can stop in time with < max_acceleration
                    return {
                        .deceleration_distance = deceleration_distance,
                        .deceleration          = deceleration,
                        .direction_change      = 0,
                    };
                }
                
                // we can't stop in time; must stop and go back
                // make sure we can stop before hitting travel limit)
                deceleration          = max_deceleration;
                deceleration_distance = get_deceleration_distance(initial_speed, reverse_out_speed, max_deceleration);
                if (deceleration_distance <= travel_available) {
                    if (Stepper::debug) printf("L469 %d %d\n", deceleration, deceleration_distance);
                    // we can stop beforre hitting travel limit while slowing down at less than max deceleration
                    return {
                        .deceleration_distance = deceleration_distance,
                        .deceleration          = deceleration,
                        .direction_change      = travel_direction,
                    };
                }

                deceleration          = get_deceleration(initial_speed, reverse_out_speed, travel_available);
                deceleration_distance = travel_available;
                if (Stepper::debug) printf("L479 %d %d\n", deceleration, deceleration_distance);
                // we will use whatever deceleration we neee to stop before hitting travel limit
                return {
                    .deceleration_distance = deceleration_distance,
                    .deceleration          = deceleration,
                    .direction_change      = travel_direction,
                };
            }
        }
        
        if (initial_speed <= target_speed) {
            // must speed up to reach target speed
            
            // case 1
            //    ------
            //   /      \ 
            // --        \ 
            //            --
            
            //  case 2
            //      ------
            //     /      \ 
            //    /        --
            //   /
            // --
                
            int acceleration_distance = get_acceleration_distance(initial_speed, target_speed, target_acceleration);
            int deceleration_distance = get_deceleration_distance(target_speed, speed_out, target_deceleration);
            if (Stepper::debug) printf("L480 %d %d\n", acceleration_distance, deceleration_distance);
            
            MotionBlock motion = make_motion(acceleration_distance, deceleration_distance, travel_distance);
            return {
                .acceleration_distance = motion.acceleration_distance,
                .acceleration          = target_acceleration,
                
                .coast_distance        = motion.coast_distance,
                
                .deceleration_distance = motion.deceleration_distance,
                .deceleration          = target_deceleration,
                
                .direction_change      = 0,
            };
        }
        
        // must slow down to reach target speed
        //
        // --
        //   \ 
        //    ----
        //        \ 
        //         --
        
        int deceleration_distance1 = get_deceleration_distance(initial_speed, target_speed, target_deceleration);    
        int deceleration_distance2 = get_deceleration_distance(target_speed, speed_out, target_deceleration);
        if (Stepper::debug) printf("L504 %d %d\n", deceleration_distance1, deceleration_distance2);
        
        MotionBlock motion = make_motion(deceleration_distance1, deceleration_distance2, travel_distance);
        return {
            .acceleration_distance = motion.deceleration_distance,
            .acceleration          = -target_acceleration,
            .coast_distance        = motion.coast_distance,
            .deceleration_distance = motion.deceleration_distance,
            .deceleration          = target_deceleration,
            
            .direction_change      = 0,
        };

    } // end if (travel_direction == initial_direction)

    // reverse direction
    if (initial_speed <= reverse_out_speed) {
        return {
            // change direction;
            .direction_change      = travel_direction,
        };
    }
    
    // must slow down to speed_out_reverse
    int deceleration      = std::max(initial_deceleration, target_acceleration);
    int stopping_distance = get_deceleration_distance(initial_speed, reverse_out_speed, deceleration);
        
    if (stopping_distance > travel_available) {
        int deceleration = get_deceleration(initial_speed, reverse_out_speed, travel_available);
        int deceleration_distance = travel_available;
        return {
            .deceleration_distance = deceleration_distance,
            .deceleration          = deceleration,
            .direction_change      = travel_direction,
        };
    }
    
    return {
        .deceleration_distance = stopping_distance,
        .deceleration          = deceleration,
        .direction_change      = travel_direction,
    };
}

static MotionPlan2 acceleration_plan(
    int initial_position,
    int initial_direction,
    int initial_speed,
    int initial_deceleration,
    
    int target_position,
    int max_speed,
    int signed_acceleration,
    int max_acceleration,
    
    int speed_in,
    int speed_out,
    int travel_available,
    int reverse_out_speed
) {
    int delta_position = target_position - initial_position;
    int travel_distance = abs(delta_position);
    int travel_direction = signum(delta_position);
    
    if (signed_acceleration == 0) {
        return {
            .coast_distance = travel_distance,
        };
    }

    int acceleration = std::abs(signed_acceleration);
    if (signum(signed_acceleration) != travel_direction) {
        // deceleartion case
         
        int stopping_distance = get_deceleration_distance(initial_speed, reverse_out_speed, acceleration);
        if (acceleration < max_acceleration) {   
            if (stopping_distance > delta_position) {
                delta_position = stopping_distance;
            }
            
            // all good, we can stop in time
            return {
                .deceleration_distance = delta_position,
                .deceleration          = acceleration,
            };
        }
        
        // will overshoot
        if (stopping_distance <= travel_available) {
            return {
                .deceleration_distance = stopping_distance,
                .deceleration           = acceleration,
            };
        }
        
        int deceleration          = get_deceleration(initial_speed, reverse_out_speed, travel_available);
        int deceleration_distance = travel_available;
        return {
            .deceleration_distance = deceleration_distance,
            .deceleration          = deceleration,
        };
    }
    
    if (acceleration > max_acceleration) {
        acceleration = max_acceleration;
    }
    
    return {
        .acceleration_distance = travel_distance,
        .acceleration = acceleration,
    };
    
    // acceleration case
    
}

template <typename T1, typename T2>
static auto div_round(T1 x, T2 d) {
    if (x > 0) {
        return (x + d/2) / d;
    } else {
        return (x - d/2) / d;
    }
}


int32 Stepper::get_real_position() const {
    if (!encoder.enabled) {
        return isr_data.current_position;
    }
    
    // printf("BEFORE!\n");
    //printf("read %d\n", encoder.encoder.state);
    // printf("X1\n");
    int32 encoder_position_raw = encoder.read();
    // int32 encoder_position_fixed = div_round(encoder_position_raw, 10);
    int32 encoder_position_fixed = std::round(encoder_position_raw * encoder_ratio);
    // printf("counted position %d; encoder position %d; fixed %d\n", isr_data.current_position, encoder_position_raw, encoder_position_fixed);

    return encoder_position_fixed;
}

void Stepper::replan() {
    int travel_available;
    if (isr_data.current_direction > 0) {
        travel_available = max_step - isr_data.current_position;
    } else {
        travel_available = isr_data.current_position - min_step;
    }
    
    MotionPlan2 plan = plan_motion(
            isr_data.current_position,
            isr_data.current_direction,
            isr_data.speed,
            isr_data.deceleartion_2x/2,
            
            isr_data.target_position,
            isr_data.target_speed,
            isr_data.target_acceleration,
            isr_data.target_deceleration,
            isr_data.speed_in,
            isr_data.speed_out,
            
            travel_available,
            steps_per_degree);
        if (debug) printf("motion plan delta=%d pos=%d dir=%d speed=%d cacc=%d target_position=%d target_speed=%d sin=%d sout=%d target_acceleration=%d acc=%d dec=%d accd=%d coast=%d decd=%d\n",
             (int) (isr_data.target_position - isr_data.current_position),
             (int) isr_data.current_position,
             (int) isr_data.current_direction,
             (int) isr_data.speed,
             (int) isr_data.acceleration_2x / 2,
             (int) isr_data.target_position, (int) isr_data.target_speed, (int) isr_data.speed_in, (int) isr_data.speed_out, (int) isr_data.target_acceleration,
             (int) plan.acceleration, plan.deceleration,
             (int) plan.acceleration_distance, (int) plan.coast_distance, (int) plan.deceleration_distance);

        isr_data.acceleration_2x         = 2*plan.acceleration;
        isr_data.deceleartion_2x         = 2*plan.deceleration;

        // isr_data.direction_change   = plan.reverse_direction;
        isr_data.acceleration_end   = plan.acceleration_distance;
        isr_data.deceleration_start = isr_data.acceleration_end + plan.coast_distance;

        isr_data.step       = 0;
        isr_data.step_count = plan.acceleration_distance + plan.coast_distance + plan.deceleration_distance;     

        if (isr_data.speed == 0) {
            isr_data.speed         = isr_data.speed_in;
            isr_data.speed_squared = isr_data.speed_in*isr_data.speed_in;
            isr_data.target_tick   = 0;
        }
        
        isr_data.direction_change = plan.direction_change;
}

void Stepper::ISR() {
    if (isr_data.reset_pin) {
        digitalWriteFast(stepPin, LOW);
        isr_data.reset_pin = false;
        return;
    }

    PendingAction &action = *pending_action;
    if (action.nonce != isr_data.nonce) {
        isr_data.nonce = action.nonce;

        if (!action.is_moving) {
            stop();
            action.motion_complete = true;
            return;
        }

        if (use_encoders && isr_data.speed == 0) {
            isr_data.current_position = get_real_position();
        }

        isr_data.target_position     = action.target_position;
        isr_data.target_speed        = action.target_speed;
        isr_data.target_acceleration = action.acceleration,
        isr_data.speed_in            = action.speed_in;
        isr_data.speed_out           = action.speed_out;
        
        if (action.deceleration > 0) {
            isr_data.target_deceleration = action.deceleration;
        } else {
            isr_data.target_deceleration = isr_data.target_acceleration;
        }

        if (action.target_time > 0) {
            int32 delta_position = action.target_position - isr_data.current_position;
            int travel_distance = abs(delta_position);

            int32 speed_overall = travel_distance / action.target_time;
            int speed = std::min(speed_overall, int32(action.target_speed));
            int acc_multip = action.acceleration / action.target_speed;
            int acc = speed* acc_multip;
            
            isr_data.target_speed = speed;
            isr_data.target_acceleration = acc;
            isr_data.target_deceleration = acc;
            
            replan();
            
            
            //      2*(d - v0*t)
            // a = --------------
            //          t^2

            // int dir = isr_data.current_direction < 0 ? -1 : 1;
            // int32 speed = isr_data.speed;
            // if (speed < isr_data.speed_in) {
            //     speed = isr_data.speed_in;
            // }
            // int32 current_velocity = speed * dir;

            // int32 delta_position = action.target_position - isr_data.current_position;

            // float64 a = (2.0 * (delta_position - current_velocity * action.target_time)) / (action.target_time * action.target_time);
            
            // if ((a > 0 && delta_position > 0)  || (a < 0 && delta_position < 0)) {
            //     if (debug) printf("MUST ACCELERATE %f\n", a);
            // } else {
            //     if (debug) printf("MUST DECELERATE %f\n", a);
            // }
            // isr_data.target_acceleration = int(std::abs(a) + 0.5);
            
            // if (action.deceleration <= 0) {
            //     isr_data.target_deceleration = isr_data.target_acceleration;
            // }
            
            // // replan();
            
            // int acc;
            // if (a < 0) {
            //     acc = int(a - 0.5);
            // } else {
            //     acc = int(a + 0.5);
            // }
            // int travel_available;
            // if (isr_data.current_direction > 0) {
            //     travel_available = max_step - isr_data.current_position;
            // } else {
            //     travel_available = isr_data.current_position - min_step;
            // }

            // MotionPlan2 plan = acceleration_plan(
            //     isr_data.current_position,
            //     isr_data.current_direction,
            //     isr_data.speed,
            //     isr_data.deceleartion_2x/2,
                
            //     action.target_position,
            //     /* max_speedf*/ action.target_speed, 
            //     acc,
            //     /* max_acceleration*/ action.acceleration,
            //     action.speed_in,
            //     action.speed_out, 
                
            //     travel_available,
            //     steps_per_degree);
     
            // if (debug) printf("TARGET TIME: acc %f; d %d; v %d; t %f\n", a, delta_position, current_velocity, action.target_time);
            // if (debug)
            //     printf("acceleration_plan delta=%d pos=%d dir=%d speed=%d cacc=%d target_position=%d "
            //            "target_speed=%d sin=%d sout=%d target_acceleration=%d acc=%d dec=%d "
            //            "accd=%d coast=%d decd=%d\n",
            //         (int)(isr_data.target_position - isr_data.current_position),
            //         (int)isr_data.current_position, (int)isr_data.current_direction,
            //         (int)isr_data.speed, (int)isr_data.acceleration_2x / 2,
            //         (int)isr_data.target_position, (int)isr_data.target_speed,
            //         (int)isr_data.speed_in, (int)isr_data.speed_out,
            //         (int)isr_data.target_acceleration, (int)plan.acceleration, plan.deceleration,
            //         (int)plan.acceleration_distance, (int)plan.coast_distance,
            //         (int)plan.deceleration_distance);

            // isr_data.acceleration_2x = 2 * plan.acceleration;
            // isr_data.deceleartion_2x = 2 * plan.deceleration;

            // // isr_data.direction_change   = plan.reverse_direction;
            // isr_data.acceleration_end = plan.acceleration_distance;
            // isr_data.deceleration_start = isr_data.acceleration_end + plan.coast_distance;

            // isr_data.step = 0;
            // isr_data.step_count
            //     = plan.acceleration_distance + plan.coast_distance + plan.deceleration_distance;

            // if (isr_data.speed == 0) {
            //     isr_data.speed = isr_data.speed_in;
            //     isr_data.speed_squared = isr_data.speed_in * isr_data.speed_in;
            //     isr_data.target_tick = 0;
            // }
        } else {
            replan();
        }

            
        
        max_delay = 0;

    } else if (isr_data.speed == 0) {
        return;
    }

    if (isr_data.tick < isr_data.target_tick) {
        isr_data.tick++;
        return;
    }
    isr_data.tick = 0;

    if (action.condition) {
        bool ok = action.condition();
        if (!ok) {
            stop();
            action.motion_complete = true;
            return;
        }
    }

    int32 enc_pos = get_real_position();
    int diff = isr_data.current_position - enc_pos;
    if (isr_data.current_direction < 0) {
        diff = -diff;
    }
    
    if (diff > max_delay) {
        max_delay = diff;
    }

    // bool lagging  = diff > 1;
    // if (lagging && use_encoders) {
    //     if (debug) printf("stepper lagging\n");
    //     digitalWriteFast(stepPin, HIGH);
    //     isr_data.reset_pin = true;
    //     return;
    // }

    // bool overshot = 
    //         isr_data.direction_change == 0 && (
    //             (isr_data.current_direction > 0 && enc_pos > isr_data.target_position) || 
    //             (isr_data.current_direction < 0 && enc_pos < isr_data.target_position)
    //         );
    // if (overshot && use_encoders) {
    //     if (debug) printf("stepper overshot\n");
    //     stop();
    //     action.motion_complete = true;
    //     return;
    // }

    if (isr_data.step < isr_data.acceleration_end) // accelerating
    {
        isr_data.speed          = sqrtf(isr_data.speed_squared);
        isr_data.speed_squared += isr_data.acceleration_2x;

        if (isr_data.speed > isr_data.target_speed) {
            isr_data.speed = isr_data.target_speed;
            if (debug) printf("WARN: speed greater than max speed when accelerating\n");
        }
        if (isr_data.speed < isr_data.speed_in) {
            isr_data.speed = isr_data.speed_in;
            if (debug) printf("WARN: speed less than min speed during acceleration\n");
        }
        update_frequency(isr_data.speed);

        do_step();
        return;
    }

    if (isr_data.step < isr_data.deceleration_start) // constant speed
    {
        isr_data.speed = isr_data.target_speed;
        update_frequency(isr_data.speed);

        do_step();
        return;
    }

    if (isr_data.step < isr_data.step_count) // decelerating
    {
        isr_data.speed_squared -= isr_data.deceleartion_2x;
        isr_data.speed          = sqrtf(isr_data.speed_squared);
        
        // if (isr_data.speed < isr_data.speed_out) {
        //     isr_data.speed = isr_data.speed_out;
        //     if (debug) printf("WARN: speed less than min speed when decelerating: %d\n", (int) isr_data.speed);
        // }
        if (isr_data.speed < steps_per_degree) {
            isr_data.speed = steps_per_degree;
            if (debug) printf("WARN: speed less than min speed when decelerating: %d\n", (int) isr_data.speed);
        }

        update_frequency(isr_data.speed);

        do_step();
        return;
    }

    // direction change
    if (isr_data.direction_change != 0) {
        do_direction_change(isr_data.direction_change);
        replan();
        return;
    }
    // if (isr_data.current_position != isr_data.target_position) {
    //     int delta_position   = isr_data.target_position - isr_data.current_position;
    //     int travel_direction = signum(delta_position);
    //     if (debug) printf("INFO: direction change\n");
        
    //     if (travel_direction == isr_data.current_direction) {
    //         printf("CRITICAL: direction change %d\n", travel_direction);
    //         return;
    //         panic("direction change");
    //     }

    //     do_direction_change(travel_direction);
    //     replan();
    //     return;
    // }

    // target reached
    stop();
    action.motion_complete = true;
    // if (debug) printf("ISR motion complete\n");
}

// ISR context
void Stepper::stop() {
    //isr_data.is_moving      = false;
    isr_data.speed = 0;
    update_state();
}

// ISR context
void Stepper::update_frequency(float steps_per_sec) {
    int32_t tick_count = (int) roundf(200'000 / steps_per_sec);
    isr_data.target_tick = tick_count;
}

// ISR context
void Stepper::do_step() {
    digitalWriteFast(stepPin, HIGH);

    isr_data.step += 1;
    isr_data.current_position += isr_data.current_direction;

    isr_data.reset_pin = true;
    update_state();
}

void Stepper::do_direction_change(int new_direction) {
    isr_data.current_direction = new_direction;
    digitalWriteFast(dirPin, new_direction > 0 ? HIGH : LOW);
    isr_data.reset_pin = true;
    isr_data.speed = 0;
}

// ISR context
void Stepper::update_state() {
    *tmp_state = {
        .is_moving        = isr_data.speed > 0,
        .current_position = isr_data.current_position,
    };

    std::swap(current_state, tmp_state);
}
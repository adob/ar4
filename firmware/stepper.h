#pragma once

#include <functional>
#include "lib/base.h"
#include "encoder.h"

namespace robot {
    using namespace lib;
    struct StepperTimer;

    struct Stepper {
        static volatile bool debug;

        static const int32 min_speed = 100;

        Stepper(
            int stepPin, int dirPin, int encoder_pin1, int encoder_pin2, int microsteps, 
            int32 max_step,
            int32 min_step,
            
            int32 steps_per_degree);

        Encoder encoder;

        const int stepPin, dirPin;
        const int32 max_step, min_step;
        const int32 steps_per_degree;
        
        int32 max_delay;

        void moveAbs(int32 target, uint32 v = 0, std::function<bool()> const& condition = {});
        void moveAbsAsync(int32 target, uint32 v = 0, uint32 a = 0, int32 deceleration = 0, uint32 speed_in = min_speed, uint32 speed_out = min_speed, float64 target_time = 0, std::function<bool()> const& condition = {});

        void moveRel(int32 delta, uint32 v = 0, std::function<bool()> const& condition = {});
        void moveRelAsync(int32 delta, uint32 v = 0, std::function<bool()> const& condition = {});

        // move until pin reads val, or number of steps readches maxSteps
        void moveUntil(int32 maxSteps, std::function<bool()> const& condition, uint32 v = 0);

        void startMoveTo(int32 s_tgt, int32 v_e, uint32 v_max, uint32 a, int32 deceleration, uint32 speed_in, uint32 speed_out, float64 target_time, std::function<bool()> const& condition);

        int32 getPosition() const;
        void setPosition(int32 p);

        void await();

        void emergency_stop();

    // private:
        struct {
            unsigned nonce = 0;
            int32 target_position = 0;

            //bool is_moving = false;
            bool reset_pin = false;

            int32 current_position = 0;
            int current_direction = 0;

            int32 speed = 0;
            int32 speed_in = min_speed;
            int32 speed_out = min_speed;
            int32 speed_squared = 0;
            int32 target_speed = 0;
            
            int32 target_acceleration = 0;
            int32 target_deceleration = 0;

            int32 target_tick = 0;
            int32 tick = 0;

            int32 step_count = 0;
            int32 step = 0;

            // int32 initial_deceleration_2x = 0;
            int32 acceleration_2x         = 0;
            int32 deceleartion_2x         = 0;

            // int32 initial_deceleration_end = 0;
            // int32 direction_change_position = 0;
            // int direction_change = 0;
            int32 deceleration_start = 0;
            int32 acceleration_end = 0;
            
            int direction_change = 0;
        } isr_data;

        struct State {
            bool is_moving = false;
            int32 current_position = 0;
        } state1, state2;

        State *volatile current_state = &state1;
        State *volatile tmp_state = &state2;
        void ISR();
        void stop();
        void update_frequency(float);
        void do_step();
        void do_direction_change(int new_direction);
        void update_state();
        void replan();

        int32 get_real_position() const;
        const float32 encoder_ratio;

        enum class mode_t {
            target,
            rotate,
            stopping,
        };

        struct PendingAction {
            unsigned nonce = 0;
            bool is_moving = false;
            int32 target_position = 0;
            uint32 target_speed = 0;
            uint32 speed_in = min_speed;
            uint32 speed_out = min_speed;
            uint32 acceleration = 0;
            uint32 deceleration = 0;
            float64 target_time = 0;
            bool use_encoders = true;
            std::function<bool()> condition;

            volatile bool motion_complete = false;
        } pending_action1 = {}, pending_action2 = {};

        PendingAction *volatile pending_action = &pending_action1;
        PendingAction *volatile pending_action_tmp = &pending_action2;

        StepperTimer *timer = nullptr;
        Stepper* next = nullptr; // linked list of steppers, maintained by StepperTimer

        // Stepper& setMaxSpeed(int32 speed);          // steps/s
                                                       // StepperBase& setVStart(int32 vIn);              // steps/s
                                                       // StepperBase& setVStop(int32 vIn);               // steps/s
        // Stepper& setAcceleration(uint32 _a);         // steps/s^2
                                                       //

        // Stepper& setInverseRotation(bool b); // Change polarity of the dir pulse

        // void moveAsync();

        // 
        


        // void rotateAsync(int32 v = 0);
        // void stopAsync();
        // 


        int32 vMax = vMaxDefault;
        uint32 acc  = aDefault;
        // uint32 avMax;

        // std::function<bool()> condition;

        // static constexpr int32 vMaxMax        = 100'000; // largest speed possible (steps/s)
        // static constexpr uint32 aMax          = 999'999; // speed up to 500kHz within 1 s (steps/s^2)
        static constexpr uint32 vMaxDefault   = 1'000;   // should work with every motor (1 rev/sec in 1/4-step mode)
        // static constexpr uint32 vStartDefault = 100;     // start speed
        // static constexpr uint32 vStopDefault  = 100;     // stop speed
        static constexpr uint32 aDefault      = 1'000;   // reasonably low (~1s for reaching the default speed)

    };
}
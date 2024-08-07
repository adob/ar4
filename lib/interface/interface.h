#pragma once

#include "lib/base.h"


namespace robot {
    using namespace lib;

    enum MethodID: int {
        Sum = 1,
        Blink,
        MoveJointsRel,
        MoveJointsAbs,
        LogState,
        Home,
        GetState,
        SetDebug,
        TestCmd
    } ;

    struct SetDebugReqeust {
        bool debug = false;

        template <typename Archive>
        void serialize(Archive &ar) {
            ar(debug);
        }
    } ;

    struct SumRequest {
        int32 left  = 0;
        int32 right = 0;

        template <typename Archive>
        void serialize(Archive &ar) {
            ar(left, right);
        }
    };

    struct SumResponse {
        int32 answer;

        template <typename Archive>
        void serialize(Archive &ar) {
            ar(answer);
        }
    };

    struct BlinkRequest {
        int32 duration_ms;

        template <typename Archive>
        void serialize(Archive &ar) {
            ar(duration_ms);
        }
    } ;

    struct MoveJointsRelRequest {
        float64 j1 = 0;
        float64 j2 = 0;
        float64 j3 = 0;
        float64 j4 = 0;
        float64 j5 = 0;
        float64 j6 = 0;

        template <typename Archive>
        void serialize(Archive &ar) {
            ar(j1, j2, j3, j4, j5, j6);
        }
    } ;

    const float64 DefaultSpeedDegreesPerSecond = 25;

    struct MoveJointRequest {
        bool enable = false;
        float64 position_degrees = 0;
        float64 speed_degrees_per_second = DefaultSpeedDegreesPerSecond;
        float64 speed_in_degrees_per_second = 1;
        float64 speed_out_degrees_per_second = 1;
        float64 acceleration = DefaultSpeedDegreesPerSecond;
        float64 deceleration = 0;
        float64 target_time = 0;

        template <typename Archive>
        void serialize(Archive &ar) {
            ar(enable, position_degrees, speed_degrees_per_second, speed_in_degrees_per_second, speed_out_degrees_per_second, acceleration, deceleration, target_time);
        }
    } ;

    struct MoveJointsAbsRequest {
        MoveJointRequest j1, j2, j3, j4, j5, j6;

        bool has_alt_wrist = false;
        float64 j4b, j5b, j6b;
        bool await = false;

        template <typename Archive>
        void serialize(Archive &ar) {
            ar(j1, j2, j3, j4, j5, j6, has_alt_wrist, j4b, j5b, j6b, await);
        }
    } ;

    struct HomeRequest {
        bool j1 = false;
        bool j2 = false;
        bool j3 = false;
        bool j4 = false;
        bool j5 = false;
        bool j6 = false;;

        template <typename Archive>
        void serialize(Archive &ar) {
            ar(j1, j2, j3, j4, j5, j6);
        }
    } ;

    struct StateResponse {
        bool j1_homed;
        float64 j1_angle;

        bool j2_homed;
        float64 j2_angle;

        bool j3_homed;
        float64 j3_angle;

        bool j4_homed;
        float64 j4_angle;

        bool j5_homed;
        float64 j5_angle;

        bool j6_homed;
        float64 j6_angle;

        template <typename Archive>
        void serialize(Archive &ar) {
            ar(
                j1_homed, j1_angle,
                j2_homed, j2_angle,
                j3_homed, j3_angle,
                j4_homed, j4_angle,
                j5_homed, j5_angle,
                j6_homed, j6_angle);
        }
    } ;
    
    struct TestCmdRequest {
        template <typename Archive>
        void serialize(Archive &ar) {
            ar();
        }
    } ;
}

#pragma once

#include "lib/sync.h"


namespace robot {
    using namespace lib;

    struct UserInput {
        sync::Mutex mtx;
        sync::Cond  has_key_cond;

        int last_key = 0;
        
        bool foot_pedal_down = false;
        sync::Cond foot_pedal_cond;

        void handle_key_down(int key);
        void handle_key_up(int key);

        int poll_key();
        int wait_key();
        bool get_foot_pedal();
        void await_foot_pedal();
    };
}
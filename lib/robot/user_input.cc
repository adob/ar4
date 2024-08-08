#include "user_input.h"

#include <Qt>

#include "lib/print.h"

using namespace robot;

void UserInput::handle_key_down(int key) {
    sync::Lock lock(mtx);

    if (key == Qt::Key_B && !foot_pedal_down) {
        print "foot pedal DOWN";
        foot_pedal_down = true;
        foot_pedal_cond.signal();
        return;
    }

    last_key = key;
    has_key_cond.signal();
}

void UserInput::handle_key_up(int key) {
    sync::Lock lock(mtx);

    if (key == Qt::Key_B && foot_pedal_down) {
        print "foot pedal UP";
        foot_pedal_down = false;
    }
}

int UserInput::poll_key() {
    sync::Lock lock(mtx);
    int key = last_key;
    last_key = 0;
    return key;
}

int UserInput::wait_key() {
    sync::Lock lock(mtx);

    print "UserInput::waik_key()";
    has_key_cond.wait(mtx);
    return last_key;
}

bool UserInput::get_foot_pedal() {
    sync::Lock lock(mtx);
    return foot_pedal_down;
}

void UserInput::await_foot_pedal() {
    sync::Lock lock(mtx);
    if (foot_pedal_down) {
        return;
    }
    foot_pedal_cond.wait(mtx);
}
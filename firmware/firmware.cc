#include <Arduino.h>
#include "robot.h"
#include "service.h"
#include "serial.h"

#include "lib/print.h"

using namespace lib;

void setup() {    
    serial_io.accept();
    print "firmware started";

    robot::Robot ctrl;
    robot::Service robot_service(ctrl);
    serialrpc::Server rpc_server(robot_service);

    robot_service.start();

    for (;;) {
        error err;

        while (Serial.available() == 0) {
            // continue
        }
        
        print "STARTING CONNECTION", Serial.available();
        rpc_server.serve(serial_io, err);
        print "SERVE RETURNED ERROR", err;
        robot_service.emergency_stop();
        serial_io.accept();
    }
    
}

void loop() {
    abort();
}

extern "C" {
    void abort(void) {
        pinMode(LED_BUILTIN, OUTPUT);
        
        for (;;) {
            printf("ABORTED\n");
                        
            digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
            delay(1000);                      // wait for a second
            digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
            delay(1000);                      // wait for a second
        }
    }
}

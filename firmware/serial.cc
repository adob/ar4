#include <Arduino.h>

#include "serial.h"

#include "lib/print.h"

SerialIO serial_io;

byte serial_read_byte() {
    uint8 i;
    
    for (;;) {
        usize n = usb_serial_read(&i, 1);
        
        if (n == 1) {
            return i;
        }
    }
}

void serial_write(str s) {
    Serial.write(s.data, s.len);
}

size SerialIO::direct_read(buf buffer, error &err) {
  retry:
    int avail;
    for (;;) {
        avail = Serial.available();
        if (avail > 0) {
            break;
        }
        if (!Serial.dtr()) {
            print "Serial has disconnected", Serial.dtr();
            err(io::EOF);
            return 0;
        }
        yield();
    }

    size amt = avail;
    if (amt > len(buffer)) {
        amt = len(buffer);
    }
    

    amt = usb_serial_read(buffer.data, amt);
    if (amt == 0) {
        goto retry;
    }

    
    return amt;
}

size SerialIO::direct_write(str data, error &err) {
    // if (!Serial) {
    //     err(io::EOF);
    // }

    serial_write(data);
    Serial.send_now();
    return data.len;
}

void SerialIO::accept() {
    reset();

    // for (;;) {
    //     if (Serial) {
    //         break;
    //     }
    // }
}
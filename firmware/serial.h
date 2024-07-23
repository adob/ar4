#pragma once
#include "lib/base.h"
#include "lib/io.h"

using namespace lib;

struct SerialIO : io::StaticBuffered<512, 512> {
    void accept();

    ssize direct_read(buf, error&) override;
    ssize direct_write(str, error&) override;
};

extern SerialIO serial_io;

byte serial_read_byte();
void serial_write(str s);


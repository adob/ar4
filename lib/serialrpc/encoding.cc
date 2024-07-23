#include "encoding.h"
#include "lib/io.h"

using namespace lib;
using namespace serialrpc;

void Archiver::write(int32 n) {
    varint::write_sint32(out, n, err);
}

void UnArchiver::read(int32 &n) {
    n = varint::read_sint32(in, err);
}

void Archiver::write(float64 n) {
    str s((byte*) &n, sizeof(n));
    out.write(s, err);
}

void UnArchiver::read(float64 &n) {
    buf b((byte*) &n, sizeof(n));
    io::read_full(in, b, err);
}


void Archiver::write(bool b) {
    out.write(b ? 1 : 0, err);
}

void UnArchiver::read(bool &b) {
    b = in.read_byte(err) != 0;
}


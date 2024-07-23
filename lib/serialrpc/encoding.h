#pragma once

#include "lib/base.h"
#include "lib/io.h"
#include "lib/varint.h"

namespace serialrpc {
    using namespace lib;

    struct Archiver {
        io::OStream &out;
        error &err;

        template <typename ...Args>
        void operator () (Args &&...args) {
            // https://www.foonathan.net/2020/05/fold-tricks/
            (void) ( (write(args), bool(err)) || ... );

            //(*this << ... << args);
        }

        void write(int32);
        void write(float64 n);
        void write(bool b);

        template <typename T>
        void write(T &t) {
            t.serialize(*this);
        }

    private:
        void write_fixed(str);
    };

    struct UnArchiver {
        io::IStream &in;
        error &err;

        template <typename ...Args>
        void operator () (Args &&...args) {
            (void) ( (read(args), bool(err)) || ... );
        }

        void read(int32 &n);
        void read(float64 &n);
        void read(bool &b);

        template <typename T>
        void read(T &t) {
            t.serialize(*this);
        }
    };

    template <typename T>
    str serialize(buf buffer, T const& t, error &err) {
        io::BufStream out(buffer);

        Archiver ar { out, err };
        const_cast<T&>(t).serialize(ar);

        return out.bytes();

    }

    template <typename T>
    void deserialize(str buffer, T &t, error &err) {
        io::StrStream in(buffer);

        UnArchiver ar { in, err };
        t.serialize(ar);
    }
}

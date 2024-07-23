#pragma once
#ifndef ARDUINO

#include <cstddef>
#include <type_traits>
#include "encoding.h"

#include "lib/io.h"
#include "lib/array.h"
#include "lib/sync.h"


namespace serialrpc {
    using namespace lib;

    struct Call {
        uint method_id;
    } ;

    struct Client {
        std::shared_ptr<io::Stream> iostream;

        Array<byte, 1024> writebuf;
        Array<byte, 1024> readbuf;

        Client(std::shared_ptr<io::Stream> const& stream);

        str call(uint method_id, str data, error &err);
        void start(error &err);
        void close(error &err);

        template <typename T1, typename T2>
        void call(uint method_id, T1 &data, T2 &result, error &err) {
            if constexpr (!std::is_same_v<T1, const std::nullptr_t>) {
                str binary = serialize(writebuf, data, err);
                if (err) {
                    return;
                }
                str retdata = call(method_id, binary, err);
                if (err) {
                    return;
                }

                if constexpr (!std::is_same_v<T2, const std::nullptr_t>) {
                    deserialize(retdata, result, err);
                }
            } else {
                str retdata = call(method_id, {}, err);
                if (err) {
                    return;
                }

                if constexpr (!std::is_same_v<T2, const std::nullptr_t>) {
                    deserialize(retdata, result, err);
                }
            }
        }

        ~Client();

    private:
        sync::Mutex call_mutex;
        sync::Mutex cond_mutex;
        sync::Cond  call_cond;
        bool started = false;
        bool has_reply = false;

        str pending_reply;
        deferror *pending_error;

        std::future<void> receiver;

        void input(error&);
        str read_body(error&);
        void handle_reply(bool is_ok, error &err);
        void handle_error(deferror &errp);
    };

}

#endif
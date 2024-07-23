#pragma once

#include "lib/base.h"
#include "lib/io.h"
#include "lib/array.h"

#include "rpc.h"
#include "encoding.h"

namespace serialrpc {
    using namespace lib;
    struct CallCtx;

    template <typename T,typename Req, typename Resp>
    using MemberFunc = Resp (T::*)(Req const&, error&);

     template <typename T, typename Resp>
    using MemberFuncNoReq = Resp (T::*)(error&);

    struct Service {
        virtual void start() {};
        virtual void handle(CallCtx &ctx, int method_id, str data, error &err) = 0;

        virtual ~Service() {}

      protected:
        template <typename T, typename Req, typename Resp>
        void handle_method(
            CallCtx &ctx,
            T &t,
            MemberFunc<T, Req, Resp> handler, str msg, error &err);

          template <typename T, typename Resp>
          void handle_method(
            CallCtx &ctx,
            T &t,
            MemberFuncNoReq<T, Resp> handler, error &err);
    };

    struct Server {
        Service &service;
        Array<byte, 1024> writebuf;
        Array<byte, 1024> readbuf;

        Server(Service &service);

        void serve(io::Stream &conn, error &err);

    private:
        void handle_rpc(io::Stream&, error&);
        void handle_request(io::Stream&, error&);
        void handle_goodbye(io::Stream&, error&);
    };

    struct CallCtx {
        Server &server;
        io::Stream &conn;

        bool handled = false;
        error err;

        void reply(str data);
        void send_code(ServerMessageType code);
        void send_error(str msg);

        template <typename T>
        void reply(T &msg) {
            str data = serialize(server.writebuf, msg, error::panic);
            reply(data);
        }
    };

    template <typename T, typename Req, typename Resp>
    void Service::handle_method(
            CallCtx &ctx,
            T &t,
            MemberFunc<T, Req, Resp> handler,
            str msg,
            error &err) {

        Req req;
        deserialize(msg, req, err);
        if (err) {
            ctx.send_code(serialrpc::BadMessage);
            return;
        };

        if constexpr (!std::is_void_v<Resp>) {
            Resp resp = (t.*handler)(req, err);
            if (err) {
                return;
            }
            ctx.reply(resp);
        } else {
            (t.*handler)(req, err);
            ctx.reply({});
        }
    }

    template <typename T, typename Resp>
    void Service::handle_method(
            CallCtx &ctx,
            T &t,
            MemberFuncNoReq<T, Resp> handler,
            error &err) {


        if constexpr (!std::is_void_v<Resp>) {
            Resp resp = (t.*handler)(err);
            if (err) {
                return;
            }
            ctx.reply(resp);
        } else {
            (t.*handler)(err);
            ctx.reply({});
        }
    }
}

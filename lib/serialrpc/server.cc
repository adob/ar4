#include "server.h"
#include "rpc.h"
#include "internal.h"
#include "lib/print.h"
#include "lib/varint.h"

using namespace serialrpc;

static void discard_line(io::Stream &conn, error &err) {
    for (;;) {
        byte b = conn.read_byte(err);
        if (err || b == '\n') {
            return;
        }
    }
}

Server::Server(Service &service) : service(service) {}


void Server::serve(io::Stream &conn, error &err) {
    for (;;) {
        handle_rpc(conn, err);
        if (err) {
            return;
        }
    }
}

void Server::handle_rpc(io::Stream &conn, error &err) {
    uint8 n = conn.read_byte(err);
    if (err) {
        return;
    }

     if (is_printable(n)) {
        conn.write("invalid text input; ignoring until end of line\n", err);
        if (err) {
            return;
        }

        discard_line(conn, err);
        return;
    }

    if ((n & 0xF0) != 0xF0) {
        conn.write((byte) FatalError, error::ignore());
        print "serialrpc: invalid start byte 0x%x" % n;
        err(ErrCorruption);
        return;
    }

    ClientMessageType type = ClientMessageType(n & 0x0F);
    switch (type) {
        case Request:
            //print "serialrpc: request";
            handle_request(conn, err);
            return;

        case ClientGoodbye:
            //print "serialrpc: got client goodbye";
            handle_goodbye(conn, err);
            return;
    }

    print "serialrpc: unrecognized message type: %d", (int) type;
    conn.write((byte) BadMessage, err);
    if (err) {
        return;
    }
}

void Server::handle_request(io::Stream &conn, error &err) {
    uint32 method_id = varint::read_uint32(conn, err);
    if (err) {
        return;
    }

    size body_size = varint::read_uint32(conn, err);
    if (err) {
        return;
    }

    // handle too big

    buf body_data = readbuf[0, body_size];
    io::read_full(conn, body_data, err);
    if (err) {
        return;
    }


    CallCtx ctx {
        server: *this,
        conn: conn,
    };

    service.handle(ctx, method_id, body_data, err);
    if (err) {
        ctx.send_error(err.ref->msg);
        return;
    }

    if (!ctx.handled) {
        conn.write((byte) UnknownMethod, err);
        if (err) {
            return;
        }
    }

    if (ctx.err) {
        err = ctx.err;
    }
}

void Server::handle_goodbye(io::Stream &conn, error &err) {
    //print "about to write goodbye";
    conn.write((byte) ServerGoodbye, err);
    conn.flush(err);
    //print "done with goodbye write";
}

void CallCtx::reply(str data) {
    if (handled) {
        panic("already handled");
    }
    handled = true;
    conn.write((byte) ServerMessageType::Reply, err);

    varint::write_uint32(conn, len(data), err);
    conn.write(data, err);
    //print "about to flush";
    conn.flush(err);
    //print "flush done";
}


void CallCtx::send_code(ServerMessageType code) {
    if (handled) {
        panic("already handled");
    }
    handled = true;
    conn.write((byte) code, err);
    //print "about to flush 2";
    conn.flush(error::panic);
    //print "flush done 2";
}


void CallCtx::send_error(str data) {
    if (handled) {
        panic("already handled");
    }
    handled = true;
    conn.write((byte) ServerMessageType::ErrorReply, err);

    varint::write_uint32(conn, len(data), err);
    conn.write(data, err);
    conn.flush(err);
}

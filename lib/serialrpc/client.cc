#ifndef ARDUINO
#include "client.h"
#include "rpc.h"
#include "internal.h"

#include "lib/varint.h"
#include "lib/print.h"
#include "lib/sync.h"
#include "lib/io.h"

using namespace lib;
using namespace serialrpc;

static void print_line(byte b, io::Stream &conn, error &err) {
    io::out.write("serialrpc log: ", error::ignore());
    
    for (;;) {
        io::out.write(b, error::ignore());
        if (b == '\n') {
            return;
        }
    
        b = conn.read_byte(err);
        if (err) {
            return;
        }
    }
}

Client::Client(std::shared_ptr<io::Stream> const& stream)
        : iostream(stream) {
}

void Client::start(error &err) {
    started = true;
    receiver = sync::go(&Client::input, this, std::ref(err));
}

void Client::input(error &err) {
    //print "serialrpc Client::input";
    for (;;) {
        //print "input trying to read byte";
        byte b = iostream->read_byte(err);
        ServerMessageType status = ServerMessageType(b);
        //print "input done reading byte";
        if (err) {
            return;
        }
        
        //print "serialrpc client received status", (int) status;
        
        switch (status) {
        case ServerMessageType::Reply:
            handle_reply(true, err);
            continue;
        
        case ErrorReply:
            handle_reply(false, err);
            continue;
        
        case ServerMessageType::UnknownMethod:
            handle_error(ErrUnknownMethod);
            continue;
            
        case ServerMessageType::TooBig:
            handle_error(ErrRequestTooBig);
            continue;
            
        case ServerMessageType::BadMessage:
            handle_error(ErrBadMessage);
            continue;
            
        case ServerMessageType::FatalError:
            err(ErrFatal);
            return;
            
        case ServerMessageType::ServerGoodbye:
            // print "received server goodbye";
            return;
        }
        
        if (is_printable(b)) {
            print_line(b, *iostream, err);
        } else {
            print "serialrpc: client got unexpected byte %q" % (char) status;
        }
    }    
}

void Client::handle_reply(bool is_ok, error &err) {
    str body = read_body(err);
    if (err) {
        return;
    }
    
    sync::Lock call_lock(cond_mutex);
    // if (!awaiting_reply) {
    //     eprint "serialrpc: received unexpected reply; is_ok", is_ok;
    //     return;
    // }
    
    has_reply = true;
    pending_reply  = body;
    pending_error  = is_ok ? nil : &ErrReply;
    call_cond.signal();
}


void Client::handle_error(deferror &errp) {
    sync::Lock call_lock(cond_mutex);
    // if (!awaiting_reply) {
    //     eprint "serialrpc: received unexpected error reply", errp.msg;
    //     return;
    // }
    
    has_reply = true;
    pending_reply  = {};
    pending_error  = &errp;
    call_cond.signal();
}

str Client::read_body(error &err) {
    size body_size = varint::read_uint32(*iostream, err);
    if (err) {
        return {};
    }
    
    if (body_size > len(readbuf)) {
        err(ErrResponseTooBig);
        return {};
    }
    
    buf body_data = readbuf[0, body_size];
    io::read_full(*iostream, body_data, err);
    if (err) {
        return {};
    }
    
    return body_data;
}


str Client::call(uint method_id, str data, error &err) {
    sync::Lock lock(call_mutex);
    
    if (!started) {
        start(error::panic);
    }

    byte start_byte = byte(0xF0) | byte(Request);
    iostream->write(start_byte, err);
    if (err) {
        return {};
    }
    
    varint::write_uint32(*iostream, method_id, err);
    if (err) {
        return {};
    }
    
    varint::write_uint32(*iostream, uint32(len(data)), err);
    if (err) {
        return {};
    }
    
    iostream->write(data, err);
    if (err) {
        return {};
    }
    
    iostream->flush(err);
    
    sync::Lock cond_lock(cond_mutex);
    while (!has_reply) {
        call_cond.wait(cond_mutex);
    }
    has_reply = false;
    
    if (pending_error) {
        if (pending_error == &ErrReply) {
            print "serialrpc received error reply:", pending_reply;
        }
        err(*pending_error);
    }
    
    return pending_reply;
}

void Client::close(error &err) {
    //sync::Lock lock(call_mutex);
    // print "Client::close() called";
    
    if (!started) {
        return;
    }
    
    byte cmd_byte = byte(0xF0) | byte (ClientGoodbye);
    iostream->write(cmd_byte, err);
    iostream->flush(err);
    if (err) {
        return;
    }
    
    // print "waiting for receiver";
    receiver.get();
    
    // print "closing stream";
    iostream->close(err);
    started = false;
}

Client::~Client() {
    close(error::ignore());
}
#endif
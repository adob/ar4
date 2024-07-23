#include "rpc.h"

namespace serialrpc {
    deferror ErrReply("serialrpc error reply");
    deferror ErrUnknownMethod("serialrpc unknown method");
    deferror ErrRequestTooBig("serialrpc request too big");
    deferror ErrResponseTooBig("serialrpc request too big");
    deferror ErrBadMessage("serialrpc bad message");
    deferror ErrFatal("serialrpc fatal error");
    
    deferror ErrCorruption("serialrpc I/O corruption");
}

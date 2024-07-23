#pragma once
#include "lib/base.h"

namespace serialrpc {
    using namespace lib;
    
    enum ServerMessageType : byte {
        Reply,
        ErrorReply,   
        UnknownMethod,
        TooBig,
        BadMessage,
        FatalError,
        
        ServerGoodbye,
    };
    
    enum ClientMessageType : byte {
        Request = 0,
        ClientGoodbye = 1,
    };

    
    extern deferror ErrReply;
    extern deferror ErrUnknownMethod;
    extern deferror ErrRequestTooBig;
    extern deferror ErrResponseTooBig;
    extern deferror ErrBadMessage;
    extern deferror ErrFatal;
    
    extern deferror ErrCorruption;
}

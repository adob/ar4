#include "internal.h"

bool serialrpc::is_printable(uint8 c) {
    if (c >= ' '&& c <= '~') {
        return true;
    }
    
    return c == '\n' || c == '\r' || c == '\t' || c == '\v';
}

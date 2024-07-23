#include "example_service.h"
#include "encoding.h"

using namespace lib;
using namespace serialrpc;


ExampleService::SumResponse ExampleService::sum(SumRequest const& req, error&) {
    return {
        answer: req.left + req.right,
    };
}

void ExampleService::handle(serialrpc::CallCtx &ctx, int method_id, str msg, error &err) {
    MethodID id = MethodID(method_id);
    
    switch (id) {
        case Sum:
            handle_method(ctx, *this, &ExampleService::sum, msg, err);
            break;
    }
    
}

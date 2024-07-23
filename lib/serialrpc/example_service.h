#pragma once

#include "server.h"

namespace serialrpc {
    
    struct ExampleService : Service {
        
        enum MethodID: int {
            Sum = 1,
        } ;
        
        struct SumRequest {
            int32 left;
            int32 right;
            
            template <typename Archive>
            void serialize(Archive &ar) {
                ar(left, right);
            }
        };

        struct SumResponse {
            int32 answer;
            
            template <typename Archive>
            void serialize(Archive &ar) {
                ar(answer);
            }
        };
        
         SumResponse sum(SumRequest const&, error &err);
        
        void handle(serialrpc::CallCtx &ctx, int method_id, str msg, error &err) override;
    };
}

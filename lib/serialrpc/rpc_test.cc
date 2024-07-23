#include <future>


#include "client.h"
#include "server.h"
#include "encoding.h"
#include "example_service.h"
#include "gtest/gtest.h"

#include "hostlib/debug.h"
#include "sharedlib/os.h"
#include "sharedlib/print.h"
// #include "example.pwpb.h"

using namespace lib;
using namespace serialrpc;


TEST(serialrpc, e2e) {
    debug::init();
    
    os::FilePair host2device = os::pipe(error::panic);
    os::FilePair device2host = os::pipe(error::panic);
    
    io::Forwarder host_io(device2host.reader, host2device.writer);
    io::Forwarder device_io(host2device.reader, device2host.writer);
    
    auto device = std::async(std::launch::async, [&]() {
        // device end
        ExampleService service;
        
        serialrpc::Server serv(service);
        serv.serve(device_io, error::panic);
        
        print "device done";
    });
    
    ExampleService::SumRequest msg {
        left:  20,
        right: 14,
    };
    
    ExampleService::SumResponse resp;
    
    serialrpc::Client client(host_io);
    
    print "calling";
    client.call(ExampleService::Sum, msg, resp, error::panic);
    EXPECT_EQ(resp.answer, 34);
    
    
    print "got response", resp.answer;
    
    print "closing client...";
    client.close(error::panic);
    
    print "closing host io...";
    host_io.close(error::panic);
    
    print "waiting for device";
    device.get();
    print "async done";
}

TEST(Foo, Bar) {
    //ASSERT_EQ(1, 2);
}

TEST(Foo, DISABLED_Bar2) {
//     print "Hello";
//     SumRequest msg { left: 10, right: 20 };
//     io::MemStream stream;
//     
//     //serialrpc::serialize(stream, msg, error::panic);
    
}

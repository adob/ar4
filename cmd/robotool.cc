#include <QCommandLineParser>
#include <qcoreapplication.h>

#include "lib/debug/debug.h"
#include "lib/print.h"

using namespace lib;

static void home_cmd(QStringList args) {
    
}

int main(int argc, char *argv[]) {
    debug::init();

    QCommandLineParser parser;
    parser.addOption({
        "device", 
        "Path to use for robot comms [default = /dev/serial/by-id/usb-Teensyduino_USB_Serial_13756360-if00]", 
        "device", 
        "/dev/serial/by-id/usb-Teensyduino_USB_Serial_13756360-if00"});
    parser.addPositionalArgument("command", "The command to execute");
    parser.setApplicationDescription(
        "Valid commands:\n"
        "  home [j1] [j2] [j3] [j4] [j5] [j6]");

    QCoreApplication app(argc, argv);
    parser.process(app);

    
    if (parser.positionalArguments().size() == 0) {
        eprint parser.helpText();
        return 1;
    }

    String ar4_device = parser.value("device").toStdString();
    print "using device path", ar4_device;
    
    String cmd = parser.positionalArguments()[0].toStdString();
    String args = parser.positionalArguments().sliced(1);

    if (cmd == "home") {
        home_cmd(args);
    }
}
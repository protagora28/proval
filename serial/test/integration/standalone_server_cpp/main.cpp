#include <string.h>

#include <csignal>

#include <stub_server.hpp>

using namespace Serial;

std::atomic_bool running = true;

void
signal_handler(int signum)
{
    // Use strsignal() to get a string representation of the signal.
    const char* signal_str = strsignal(signum);
    PLOG_INFO.printf("Caught signal %d (%s).", signum, signal_str);
    running = false;
}

int
main(int argc, const char* argv[])
{
    Args args{};
    if (!ParseArgs(argc, argv, args))
    {
        std::cerr << "Error parsing arguments." << std::endl;
        return 1;
    }

    std::signal(SIGINT, signal_handler);
    args.log_stdout ? ms::PlogHelper::initPlogConsole(args.log_level.c_str())
                    : ms::PlogHelper::initPlog(args.log_file_path, args.log_level.c_str());
    args.print();
    StubServer server(args.serial_fd, set_serial_options, args.baud_rate, args.timeout, args.parity, running);
    server.run();
    PLOG_INFO.printf("Gracefully exited.");

    return 0;
}

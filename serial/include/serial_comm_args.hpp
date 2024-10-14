#pragma once

#include <plog/Log.h>

#include <argparse.hpp>
#include <serial_comm_setup.hpp>

inline speed_t
int_to_baud(const unsigned long int baud);
inline std::string
baud_to_string(const speed_t baud);
inline Serial::Parity
string_to_parity(const std::string& parity);

struct Args
{
    std::string serial_fd;
    speed_t baud_rate = B115200;
    unsigned int timeout = 1000;
    Serial::Parity parity = Serial::Parity::None;
    bool log_stdout = true;
    std::string log_file_path = "./ssx-serial-comm.log";
    std::string log_level = "warning";
    bool sas_cli = false;
    int ep_id = 0;
    int trm16_id = 0;
    int cmd_id = 0;
    int data = 0;
    int retries = 0;
    bool use_gpios = false;
    int gpio_rx = -1;
    int gpio_tx = -1;

    inline void
    print() const
    {
        PLOG_INFO.printf("Serial FD: %s", serial_fd.c_str());
        PLOG_INFO.printf("Baud rate: %s", baud_to_string(baud_rate).c_str());
        PLOG_INFO.printf("Timeout: %u", timeout);
        PLOG_INFO.printf(
            "Parity: %s",
            parity == Serial::Parity::None       ? "none"
                : parity == Serial::Parity::Even ? "even"
                                                 : "odd"
        );
        PLOG_INFO.printf("Log to stdout: %s", log_stdout ? "true" : "false");
        PLOG_INFO_IF(!log_stdout).printf("Log file path: %s", log_file_path.c_str());
        PLOG_INFO.printf("Log level: %s", log_level.c_str());
        PLOG_INFO.printf("SAS CLI: %s", sas_cli ? "true" : "false");
        if (sas_cli)
        {
            PLOG_INFO.printf("EP ID: %d", ep_id);
            PLOG_INFO.printf("TRM16 ID: %d", trm16_id);
            PLOG_INFO.printf("CMD ID: %d", cmd_id);
            PLOG_INFO.printf("Data: %d", data);
        }
        PLOG_INFO.printf("Retries: %d", retries);
        PLOG_INFO.printf("Use GPIOs: %s", use_gpios ? "true" : "false");
        PLOG_INFO_IF(use_gpios).printf("RX GPIO: %d", gpio_rx);
        PLOG_INFO_IF(use_gpios).printf("TX GPIO: %d", gpio_tx);
    }
};

inline bool
ParseArgs(int argc, const char* argv[], Args& args)
{
    argparse::ArgumentParser program("SSX serial comm library.", "1.0.0");
    program.add_argument("-s", "--serial-fd")
        .required()
        .help("serial interface file descriptor")
        .nargs(1)
        .metavar("FD");
    program.add_argument("-b", "--baud-rate")
        .required()
        .action([](const std::string& value) { return int_to_baud(std::stoul(value)); })
        .help("baud rate")
        .nargs(1)
        .metavar("BAUD");
    program.add_argument("-t", "--timeout")
        .default_value<unsigned int>(100)
        .scan<'u', unsigned int>()
        .help("timeout in milliseconds (minimum 100)")
        .nargs(1)
        .metavar("TIMEOUT");
    program.add_argument("-p", "--parity")
        .default_value<std::string>("none")
        // .choices("none", "even", "odd")
        // TODO choices are buggy here.
        .help("force enable or disable parity checking")
        .nargs(1)
        .metavar("PARITY");
    auto& log_group = program.add_mutually_exclusive_group();
    log_group.add_argument("-o", "--log-stdout").help("log to stdout").default_value<bool>(true).implicit_value(true);
    log_group.add_argument("-f", "--log-file-path")
        .default_value<std::string>(std::string{"./ssx-serial-comm.log"})
        .help("log file path")
        .nargs(1)
        .metavar("PATH");
    program.add_argument("-l", "--log-level")
        .default_value<std::string>(std::string{"warning"})
        .help("log level")
        .nargs(1)
        .metavar("LEVEL");
    program.add_argument("-c", "--sas-cli").help("enable SAS CLI mode").default_value<bool>(false).implicit_value(true);
    program.add_argument("-e", "--ep-id").default_value<int>(0).scan<'d', int>().help("EP ID").nargs(1).metavar("ID");
    program.add_argument("-m", "--trm16-id")
        .default_value<int>(0)
        .scan<'d', int>()
        .help("TRM16 ID")
        .nargs(1)
        .metavar("ID");
    program.add_argument("-d", "--cmd-id")
        .default_value<int>(0)
        .scan<'d', int>()
        .help("command ID")
        .nargs(1)
        .metavar("ID");
    program.add_argument("-a", "--data").default_value<int>(0).scan<'d', int>().help("data").nargs(1).metavar("DATA");
    program.add_argument("-y", "--retries")
        .default_value<int>(0)
        .scan<'d', int>()
        .help("number of retries")
        .nargs(1)
        .metavar("RETRIES");
    program.add_argument("-g", "--use-gpios")
        .help("use GPIOs for serial comm")
        .default_value<bool>(false)
        .implicit_value(true);
    program.add_argument("-r", "--gpio-rx")
        .default_value<int>(-1)
        .scan<'d', int>()
        .help("RX GPIO pin")
        .nargs(1)
        .metavar("PIN");
    program.add_argument("-x", "--gpio-tx")
        .default_value<int>(-1)
        .scan<'d', int>()
        .help("TX GPIO pin")
        .nargs(1)
        .metavar("PIN");

    try
    {
        program.parse_args(argc, argv);
    }
    catch (const std::runtime_error& err)
    {
        std::cerr << "Error parsing arguments: " << err.what() << std::endl;
        std::cerr << "Exiting." << std::endl;
        return false;
    }

    args.serial_fd = program.get<std::string>("--serial-fd");
    args.baud_rate = program.get<speed_t>("--baud-rate");
    args.timeout = program.get<unsigned int>("--timeout");
    args.parity = string_to_parity(program.get<std::string>("--parity"));
    args.log_stdout = program.get<bool>("--log-stdout");
    args.log_file_path = program.get<std::string>("--log-file-path");
    args.log_level = program.get<std::string>("--log-level");
    args.sas_cli = program.get<bool>("--sas-cli");
    args.ep_id = program.get<int>("--ep-id");
    args.trm16_id = program.get<int>("--trm16-id");
    args.cmd_id = program.get<int>("--cmd-id");
    args.data = program.get<int>("--data");
    args.retries = program.get<int>("--retries");
    args.use_gpios = program.get<bool>("--use-gpios");
    args.gpio_rx = program.get<int>("--gpio-rx");
    args.gpio_tx = program.get<int>("--gpio-tx");

    if (args.timeout < 100)
    {
        std::cerr << "Timeout must be at least 100 milliseconds." << std::endl;
        std::cerr << "Exiting." << std::endl;
        std::cerr << program.help().str().c_str() << std::endl;
        return false;
    }

    return true;
}

speed_t
int_to_baud(const unsigned long int baud)
{
    switch (baud)
    {
        case 9600ul:
            return B9600;
        case 19200ul:
            return B19200;
        case 38400ul:
            return B38400;
        case 57600ul:
            return B57600;
        case 115200ul:
            return B115200;
        case 230400ul:
            return B230400;
        case 460800ul:
            return B460800;
        case 500000ul:
            return B500000;
        case 576000ul:
            return B576000;
        case 921600ul:
            return B921600;
        case 1000000ul:
            return B1000000;
        case 1152000ul:
            return B1152000;
        case 1500000ul:
            return B1500000;
        case 2000000ul:
            return B2000000;
        case 2500000ul:
            return B2500000;
        case 3000000ul:
            return B3000000;
        case 3500000ul:
            return B3500000;
        case 4000000ul:
            return B4000000;
        default:
            PLOG_WARNING.printf("Invalid baud rate: %lu", baud);
            return -1;
    }
}

std::string
baud_to_string(const speed_t baud)
{
    switch (baud)
    {
        case B9600:
            return "9600";
        case B19200:
            return "19200";
        case B38400:
            return "38400";
        case B57600:
            return "57600";
        case B115200:
            return "115200";
        case B230400:
            return "230400";
        case B460800:
            return "460800";
        case B500000:
            return "500000";
        case B576000:
            return "576000";
        case B921600:
            return "921600";
        case B1000000:
            return "1000000";
        case B1152000:
            return "1152000";
        case B1500000:
            return "1500000";
        case B2000000:
            return "2000000";
        case B2500000:
            return "2500000";
        case B3000000:
            return "3000000";
        case B3500000:
            return "3500000";
        case B4000000:
            return "4000000";
        default:
            return "unknown";
    }
}

Serial::Parity
string_to_parity(const std::string& parity)
{
    if (parity == "none")
    {
        return Serial::Parity::None;
    }
    else if (parity == "even")
    {
        return Serial::Parity::Even;
    }
    else if (parity == "odd")
    {
        return Serial::Parity::Odd;
    }
    else
    {
        PLOG_WARNING.printf("Invalid parity: %s", parity.c_str());
        return Serial::Parity::None;
    }
}

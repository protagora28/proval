#ifndef SERIAL_COMM_ARGS_H
#define SERIAL_COMM_ARGS_H

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>

#define DEFAULT_SERIAL_FD "/tmp/ttyDC"
#define DEFAULT_BAUD_RATE B115200
#define DEFAULT_TIMEOUT 1

inline speed_t
int_to_baud(const unsigned long int baud);
inline void
baud_to_string(const speed_t baud, char* str);

struct Args
{
    const char* serial_fd;
    speed_t baud_rate;
    uint8_t timeout;
};

void
print_help(const char* program_name)
{
    char baud_str[9];
    baud_to_string(DEFAULT_BAUD_RATE, baud_str);
    printf(
        "Usage: %s [--serial-fd <serial_fd>] [--baud-rate <baud_rate>] [--timeout <timeout>] [--help]\n",
        program_name
    );
    printf("Options:\n");
    printf("  --serial-fd   Specify the serial port (default: %s)\n", DEFAULT_SERIAL_FD);
    printf("  --baud-rate   Specify the baud rate (default: %s)\n", baud_str);
    printf("  --timeout     Specify the timeout in seconds (default: %d)\n", DEFAULT_TIMEOUT);
    printf("  --help        Display this help message\n");
}

void
parse_args(int argc, char* argv[], struct Args* args)
{
    static struct option long_options[] = {
        {"serial-fd", required_argument, 0, 's'},
        {"baud-rate", required_argument, 0, 'b'},
        {"timeout", required_argument, 0, 't'},
        {"help", no_argument, 0, 'h'},
        {0, 0, 0, 0}};

    int opt;
    while ((opt = getopt_long(argc, argv, "s:b:t:h", long_options, NULL)) != -1)
    {
        switch (opt)
        {
            case 's':
                args->serial_fd = optarg;
                break;
            case 'b':
                args->baud_rate = int_to_baud(atoi(optarg));
                break;
            case 't':
                args->timeout = (uint8_t)atoi(optarg);
                break;
            case 'h':
                print_help(argv[0]);
                exit(EXIT_SUCCESS);
            default:
                fprintf(stderr, "Unknown option\n");
                print_help(argv[0]);
                exit(EXIT_FAILURE);
        }
    }
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
            return -1;
    }
}

void
baud_to_string(const speed_t baud, char* str)
{
    switch (baud)
    {
        case B9600:
            strcpy(str, "B0009600");
            break;
        case B19200:
            strcpy(str, "B0019200");
            break;
        case B38400:
            strcpy(str, "B0038400");
            break;
        case B57600:
            strcpy(str, "B0057600");
            break;
        case B115200:
            strcpy(str, "B0115200");
            break;
        case B230400:
            strcpy(str, "B0230400");
            break;
        case B460800:
            strcpy(str, "B0460800");
            break;
        case B500000:
            strcpy(str, "B0500000");
            break;
        case B576000:
            strcpy(str, "B0576000");
            break;
        case B921600:
            strcpy(str, "B0921600");
            break;
        case B1000000:
            strcpy(str, "B1000000");
            break;
        case B1152000:
            strcpy(str, "B1152000");
            break;
        case B1500000:
            strcpy(str, "B1500000");
            break;
        case B2000000:
            strcpy(str, "B2000000");
            break;
        case B2500000:
            strcpy(str, "B2500000");
            break;
        case B3000000:
            strcpy(str, "B3000000");
            break;
        case B3500000:
            strcpy(str, "B3500000");
            break;
        case B4000000:
            strcpy(str, "B4000000");
            break;
        default:
            strcpy(str, "unknown");
            break;
    }
}

#endif  // SERIAL_COMM_ARGS_H

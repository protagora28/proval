#include <fcntl.h>
#include <getopt.h>
#include <serial_comm_args.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include <cerrno>

inline uint16_t
merge_two_7_bit_symbols(const uint8_t value_lower, const uint8_t value_upper)
{
    return (uint16_t)(((uint16_t)value_upper << 7) | (uint16_t)value_lower);
}

inline uint8_t
upper_7_bits_0_msb(const uint16_t value)
{
    return (uint8_t)(((uint16_t)(value >> 7)) & 0x7F);
}

inline uint8_t
lower_7_bits_0_msb(const uint16_t value)
{
    return (uint8_t)(value & 0x7F);
}

uint16_t
calculate_crc14(uint8_t* data, uint16_t length)
{
    const uint16_t polynomial = 0x202D;  // CRC-14/ITU polynomial.
    uint16_t crc = 0;  // Initial CRC value

    for (size_t i = 0; i < length; ++i)
    {
        crc = static_cast<uint16_t>(crc ^ static_cast<uint16_t>(data[i] << 6u));  // XOR in the next 6 bits.

        for (int j = 0; j < 8; ++j)
        {
            if (crc & 0x4000)
            {  // Check if the MSB is set.
                crc = (uint16_t)(crc << 1) ^ polynomial;
            }
            else
            {
                crc = (uint16_t)(crc << 1);
            }
        }
    }

    return crc & 0x3FFF;  // Mask to keep only the lower 14 bits.
}

struct StubServer
{
    int serial_fd;
    struct timeval timeout;
};

void
initialize_serial(struct StubServer* server, const char* serial_fd, speed_t baud_rate, uint8_t timeout)
{
    server->serial_fd = open(serial_fd, O_RDWR | O_NOCTTY | O_SYNC);
    if (server->serial_fd < 0)
    {
        perror("Error opening serial port");
        exit(EXIT_FAILURE);
    }

    server->timeout.tv_sec = timeout;
    server->timeout.tv_usec = 0;

    struct termios options;
    memset(&options, 0, sizeof(options));
    if (tcgetattr(server->serial_fd, &options) != 0)
    {
        perror("Error from tcgetattr");
        exit(EXIT_FAILURE);
    }

    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag |= (PARENB | PARODD);
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CRTSCTS;
    options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    options.c_oflag &= ~OPOST;

    options.c_cc[VTIME] = (cc_t)(10 * timeout);
    options.c_cc[VMIN] = 0;

    cfsetispeed(&options, baud_rate);
    cfsetospeed(&options, baud_rate);

    if (tcsetattr(server->serial_fd, TCSANOW, &options) != 0)
    {
        perror("Error from tcsetattr");
        exit(EXIT_FAILURE);
    }
    tcflush(server->serial_fd, TCIOFLUSH);

    printf("Server connected to device %s\n", serial_fd);
}

void
run(struct StubServer* server)
{
    while (1)
    {
        uint8_t command[1024];
        ssize_t command_length{0};

        {
            uint8_t partial_command[1];
            const size_t timeout = server->timeout.tv_sec;
            size_t elapsed{0};
            bool is_read_complete{false};
            enum class ReadState
            {
                Start,
                End
            } read_state{ReadState::Start};
            clock_t start = clock();

            while (!is_read_complete)
            {
                const ssize_t n_read = read(server->serial_fd, partial_command, 1);

                if (n_read <= 0)
                {
                    // perror("Error reading from serial port");
                    command_length = n_read;
                    break;
                }

                printf("Read %ld bytes\n", n_read);

                switch (read_state)
                {
                    case ReadState::Start:
                    {
                        if (partial_command[0] == 0xA4)
                        {
                            read_state = ReadState::End;
                            if (partial_command[0] == 0x80)
                            {
                                is_read_complete = true;
                            }
                        }
                        else
                        {
                            printf("Invalid start symbol: %d\n", partial_command[0]);
                            break;
                        }
                        break;
                    }
                    case ReadState::End:
                    {
                        if (partial_command[0] == 0x80)
                        {
                            is_read_complete = true;
                        }
                        break;
                    }
                }

                if (n_read + command_length > 1024)
                {
                    printf("Command too long.\n");
                    break;
                }

                memcpy(command + command_length, partial_command, n_read);
                command_length += n_read;
                elapsed = (clock() - start) * 1000 / CLOCKS_PER_SEC;

                if (elapsed >= timeout)
                {
                    printf("Read timeout expired.\n");
                    break;
                }
            }
        }

        if (command_length < 0)
        {
            printf("Error %i reading from serial port: %s\n", errno, strerror(errno));
            continue;
        }
        else if (command_length == 0)
        {
            printf("Read timeout expired.\n");
            continue;
        }
        else if (command_length < 12)
        {
            printf("Command too short.\n");
            continue;
        }
        else if (command_length > 62)
        {
            printf("Command too long.\n");
            continue;
        }

        printf("Received %ld bytes\n", command_length);

        uint8_t command_payload[1024];
        if (command_length > 12)
        {
            memcpy(command_payload, command + 6, command_length - 12);
        }

        const uint8_t start_symbol{0xA4};
        const uint8_t end_symbol{0x80};
        const uint8_t ep_id = command[1];
        const uint8_t trm16_id = command[2];
        const uint8_t cmd_id = command[3];
        const uint8_t cmd_crc_lsb = command[command_length - 5];
        const uint8_t cmd_crc_msb = command[command_length - 4];
        uint8_t payload_len_lsb{0};
        uint8_t payload_len_msb{0};
        uint8_t outcome{0x0};

        uint8_t response[1024];
        uint16_t response_len{0};
        uint8_t header[6] = {start_symbol, ep_id, trm16_id, cmd_id, payload_len_lsb, payload_len_msb};
        uint8_t* payload[1024];
        uint8_t footer[6] = {0, 0, 0, 0, 0, end_symbol};

        // TODO Update IDs.
        switch (cmd_id)
        {
            case 1:  // r_id
            {
                payload_len_lsb = 3;
                payload_len_msb = 0;
                outcome = 0x0;
                uint8_t payload_temp[3] = {1, 2, 3};
                memcpy(payload, payload_temp, sizeof(payload_temp));
                break;
            }
            case 2:  // r_serial_number
            {
                payload_len_lsb = 3;
                payload_len_msb = 0;
                outcome = 0x0;
                uint8_t payload_temp[3] = {1, 2, 3};
                memcpy(payload, payload_temp, sizeof(payload_temp));
                break;
            }
            case 3:  // c_error
            {
                payload_len_lsb = 0;
                payload_len_msb = 0;
                outcome = 0x0;
                break;
            }
            case 4:  // r_error
            {
                payload_len_lsb = 1;
                payload_len_msb = 0;
                outcome = 0x0;
                uint8_t payload_temp[1] = {0b1111};
                memcpy(payload, payload_temp, sizeof(payload_temp));
                break;
            }
            // Skip 5.
            case 6:  // w_operation_mode
            {
                payload_len_lsb = 0;
                payload_len_msb = 0;
                outcome = 0x0;
                break;
            }
            case 7:  // r_operation_mode
            {
                payload_len_lsb = 1;
                payload_len_msb = 0;
                outcome = 0x0;
                uint8_t payload_temp[1] = {0b1};
                memcpy(payload, payload_temp, sizeof(payload_temp));
                break;
            }
            case 8:  // w_configuration_mode
            {
                payload_len_lsb = 0;
                payload_len_msb = 0;
                outcome = 0x0;
                break;
            }
            case 9:  // r_configuration_mode
            {
                payload_len_lsb = 1;
                payload_len_msb = 0;
                outcome = 0x0;
                uint8_t payload_temp[1] = {0b1};
                memcpy(payload, payload_temp, sizeof(payload_temp));
                break;
            }
            case 10:  // r_telemetry
            {
                payload_len_lsb = 17;
                payload_len_msb = 0;
                outcome = 0x0;
                uint8_t payload_temp[17] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17};
                memcpy(payload, payload_temp, sizeof(payload_temp));
                break;
            }
            case 11:  // c_pgt
            {
                payload_len_lsb = 0;
                payload_len_msb = 0;
                outcome = 0x0;
                break;
            }
            // Skip 12, 13.
            case 14:  // w_pgt_row
            {
                payload_len_lsb = 0;
                payload_len_msb = 0;
                outcome = 0x0;
                break;
            }
            case 15:  // r_pgt_row
            {
                payload_len_lsb = 50;
                payload_len_msb = 0;
                outcome = 0x0;

                uint8_t payload_temp[50];
                payload_temp[0] = command_payload[0];
                payload_temp[1] = command_payload[1];
                memset(&payload_temp[2], 1, sizeof(payload_temp) - 2);
                memcpy(payload, payload_temp, sizeof(payload_temp));

                break;
            }
            case 16:  // w_pgt_entry
            {
                payload_len_lsb = 0;
                payload_len_msb = 0;
                outcome = 0x0;
                break;
            }
            case 17:  // r_pgt_entry
            {
                payload_len_lsb = 6;
                payload_len_msb = 0;
                outcome = 0x0;

                uint8_t payload_temp[6];
                payload_temp[0] = command_payload[0];  // row_id_lsb
                payload_temp[1] = command_payload[1];  // row_id_msb
                payload_temp[2] = command_payload[2];  // trm_id
                payload_temp[3] = 1;  // entry[0]
                payload_temp[4] = 2;  // entry[1]
                payload_temp[5] = 3;  // entry[2]
                memcpy(payload, payload_temp, sizeof(payload_temp));

                break;
            }
            case 18:  // w_pgti
            {
                payload_len_lsb = 0;
                payload_len_msb = 0;
                outcome = 0x0;
                break;
            }
            case 19:  // r_pgti
            {
                payload_len_lsb = 2;
                payload_len_msb = 0;
                outcome = 0x0;

                uint8_t payload_temp[2] = {10, 0};
                memcpy(payload, payload_temp, sizeof(payload_temp));
                break;
            }
            case 20:  // w_pgti_bounds
            {
                payload_len_lsb = 0;
                payload_len_msb = 0;
                outcome = 0x0;
                break;
            }
            case 21:  // r_pgti_bounds
            {
                payload_len_lsb = 4;
                payload_len_msb = 0;
                outcome = 0x0;

                uint8_t payload_temp[4] = {10, 0, 11, 0};
                memcpy(payload, payload_temp, sizeof(payload_temp));
                break;
            }
            case 22:  // w_index_strobe_mode
            {
                payload_len_lsb = 0;
                payload_len_msb = 0;
                outcome = 0x0;
                break;
            }
            case 23:  // r_index_strobe_mode
            {
                payload_len_lsb = 1;
                payload_len_msb = 0;
                outcome = 0x0;
                uint8_t payload_temp[1] = {1};
                memcpy(payload, payload_temp, sizeof(payload_temp));
                break;
            }
            case 24:  // w_temperature_characterization
            {
                payload_len_lsb = 0;
                payload_len_msb = 0;
                outcome = 0x0;
                break;
            }
            case 25:  // r_temperature_characterization
            {
                payload_len_lsb = 1;
                payload_len_msb = 0;
                outcome = 0x0;
                uint8_t payload_temp[1] = {1};
                memcpy(payload, payload_temp, sizeof(payload_temp));
                break;
            }
            case 26:  // w_tt
            {
                payload_len_lsb = 0;
                payload_len_msb = 0;
                outcome = 0x0;
                break;
            }
            case 27:  // r_tt
            {
                payload_len_lsb = 16 * 2;
                payload_len_msb = 0;
                outcome = 0x0;

                uint8_t payload_temp[16 * 2];
                for (uint8_t i = 0; i < 16 * 2; ++i)
                {
                    payload_temp[i] = i;
                }
                memcpy(payload, payload_temp, sizeof(payload_temp));

                break;
            }
            case 28:  // w_tti
            {
                payload_len_lsb = 0;
                payload_len_msb = 0;
                outcome = 0x0;
                break;
            }
            case 29:  // r_tti
            {
                payload_len_lsb = 1;
                payload_len_msb = 0;
                outcome = 0x0;

                uint8_t payload_temp[1] = {1};
                memcpy(payload, payload_temp, sizeof(payload_temp));
                break;
            }
            default:
            {
                printf("Unknown command ID: %d\n", cmd_id);
                break;
            }
        }

        if (sizeof(header) == 6 && sizeof(footer) == 6)
        {
            header[4] = payload_len_lsb;
            header[5] = payload_len_msb;
            const uint16_t payload_len = merge_two_7_bit_symbols(payload_len_lsb, payload_len_msb);
            response_len = (uint16_t)(sizeof(header) + payload_len + sizeof(footer));
            memcpy(response, header, sizeof(header));
            memcpy(&response[sizeof(header)], payload, payload_len);
            memcpy(&response[sizeof(header) + payload_len], footer, sizeof(footer));
            const uint16_t crc = calculate_crc14(response, response_len);
            const uint8_t crc_upper = upper_7_bits_0_msb(crc);
            const uint8_t crc_lower = lower_7_bits_0_msb(crc);
            uint8_t temp_footer[6] = {outcome, cmd_crc_lsb, cmd_crc_msb, crc_lower, crc_upper, end_symbol};
            memcpy(&response[sizeof(header) + payload_len], temp_footer, sizeof(temp_footer));
        }
        else
        {
            printf("Response is empty or too short. Sending invalid response to fail test.\n");
            response[0] = 0;
        }

        long int bytes_written = write(server->serial_fd, response, response_len);

        if (bytes_written < 0)
        {
            printf("Error %i writing to serial port: %s\n", errno, strerror(errno));
        }
        else
        {
            printf("Responded with %ld bytes\n", bytes_written);
        }
    }
}

int
main(int argc, char* argv[])
{
    struct Args args;
    args.serial_fd = DEFAULT_SERIAL_FD;
    args.baud_rate = DEFAULT_BAUD_RATE;
    args.timeout = DEFAULT_TIMEOUT;

    parse_args(argc, argv, &args);

    char baud_str[9];
    baud_to_string(args.baud_rate, baud_str);

    printf("Serial FD: %s\n", args.serial_fd);
    printf("Baud rate: %s\n", baud_str);
    printf("Timeout: %d s\n", args.timeout);

    struct StubServer server;
    initialize_serial(&server, args.serial_fd, args.baud_rate, args.timeout);
    run(&server);

    return 0;
}

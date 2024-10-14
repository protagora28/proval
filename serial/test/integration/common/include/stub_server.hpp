#pragma once

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <exception>
#include <thread>
#include <unordered_set>

#include <PlogHelper.hpp>
#include <antenna.hpp>
#include <client.hpp>
#include <message.hpp>
#include <safe_io.hpp>
#include <serial_comm_args.hpp>
#include <serial_comm_setup.hpp>
#include <test_utils.hpp>

struct MockData
{
    std::array<uint8_t, 3> id;
    std::array<uint8_t, 3> serial_number;
    SAS::TRM::OperationMode operation_mode;
    bool configuration_mode;
    __uint128_t uuid;
    std::array<uint8_t, Serial::c_telemetry_size> telemetry;
    std::vector<SAS::Beam> pgt_rows;
    uint16_t pgti;
    uint16_t pgti_min;
    uint16_t pgti_max;
    bool index_strobe_mode;
    bool temperature_characterization;
    uint8_t power_mask_main;
    uint8_t power_mask_redundant;
    bool override_temperature_mode;
    uint8_t override_temperature_code;
};

namespace Serial
{

class StubServer
{
public:

    StubServer(
        const std::string& serial_fd,
        const std::function<void(termios&)>& set_options,
        const speed_t baud_rate,
        const unsigned int timeout,
        const Parity parity,
        std::atomic_bool& running
    )
        : m_running{running}
        , m_serial_fd(-1)
        , m_timeout{timeout}
    {
        Status status = Status::NoError;
        constexpr int retries = 0;
        const bool res = m_client.connect(status, serial_fd, set_options, baud_rate, timeout, parity, retries);

        if (!res)
        {
            PLOG_FATAL
                .printf("Error connecting to %s: %s", serial_fd.c_str(), status_message_to_string(status).c_str());
            throw std::runtime_error("Error connecting to serial port.");
        }

        m_serial_fd = m_client.get_fd();

        PLOG_INFO.printf("Server connected to device %s", serial_fd.c_str());
    }

    ~StubServer() { m_running = false; }

    virtual void
    run()
    {
        MockData mock_data{};

        while (m_running)
        {
            Status status{Status::NoError};
            Message command{0};

            if (!safe_read(m_serial_fd, m_timeout, command, status, TimeoutSeverity::Info, true))
            {
                continue;
            }

            // Sleep below needed just for the integration with the temporary jig.
            // std::this_thread::sleep_for(std::chrono::milliseconds(100));

            const long int command_length = command.size();
            PLOG_DEBUG.printf("Received %ld bytes", command_length);
            PLOG_DEBUG.printf("Encoded command bytes: %s", to_string(command).c_str());
            const auto command_payload = command.get_payload();
            const auto serialized_command = command.encode();
            const uint8_t start_symbol = Message::c_start_symbol;
            const uint8_t end_symbol = Message::c_end_symbol;
            const uint8_t ep_id = serialized_command.at(1);
            const uint8_t trm16_id = serialized_command.at(2);
            const uint8_t cmd_id = serialized_command.at(3);
            const uint8_t cmd_crc_lsb = serialized_command.at(command_length - 5);
            const uint8_t cmd_crc_msb = serialized_command.at(command_length - 4);
            uint8_t payload_len_lsb;
            uint8_t payload_len_msb;
            uint8_t outcome;
            std::vector<uint8_t> response;
            std::vector<uint8_t> header;
            std::vector<uint8_t> payload;
            std::vector<uint8_t> footer = {0, 0, 0, 0, 0, end_symbol};

            // Send a response depending on command ID.
            switch (cmd_id)
            {
                case 1:  // r_id
                {
                    payload_len_lsb = 3;
                    payload_len_msb = 0;
                    outcome = 0x0;
                    header = {start_symbol, ep_id, trm16_id, cmd_id, payload_len_lsb, payload_len_msb};
                    footer = {outcome, cmd_crc_lsb, cmd_crc_msb, 0, 0, end_symbol};
                    payload.resize(payload_len_lsb);
                    payload = {
                        trm16_id == 0 ? ep_id : trm16_id,
                        0,
                        trm16_id == 0 ? (uint8_t)1 : (uint8_t)10 /* version number */};
                    break;
                }
                case 2:  // r_serial_number
                {
                    payload_len_lsb = 3;
                    payload_len_msb = 0;
                    outcome = 0x0;
                    header = {start_symbol, ep_id, trm16_id, cmd_id, payload_len_lsb, payload_len_msb};
                    footer = {outcome, cmd_crc_lsb, cmd_crc_msb, 0, 0, end_symbol};
                    payload.resize(payload_len_lsb);
                    payload = {1, 2, 3};
                    break;
                }
                case 3:  // c_error
                {
                    payload_len_lsb = 0;
                    payload_len_msb = 0;
                    outcome = 0x0;
                    header = {start_symbol, ep_id, trm16_id, cmd_id, payload_len_lsb, payload_len_msb};
                    footer = {outcome, cmd_crc_lsb, cmd_crc_msb, 0, 0, end_symbol};
                    break;
                }
                case 4:  // r_error
                {
                    payload_len_lsb = 1;
                    payload_len_msb = 0;
                    outcome = 0x0;
                    header = {start_symbol, ep_id, trm16_id, cmd_id, payload_len_lsb, payload_len_msb};
                    footer = {outcome, cmd_crc_lsb, cmd_crc_msb, 0, 0, end_symbol};
                    payload.resize(payload_len_lsb);
                    payload = {0b1111};
                    break;
                }
                // Skip 5.
                case 6:  // w_operation_mode
                {
                    payload_len_lsb = 0;
                    payload_len_msb = 0;
                    outcome = 0x0;
                    header = {start_symbol, ep_id, trm16_id, cmd_id, payload_len_lsb, payload_len_msb};
                    footer = {outcome, cmd_crc_lsb, cmd_crc_msb, 0, 0, end_symbol};

                    // Write operation mode.
                    mock_data.operation_mode =
                        command_payload.at(0) == 0 ? SAS::TRM::OperationMode::Idle : SAS::TRM::OperationMode::Operative;

                    break;
                }
                case 7:  // r_operation_mode
                {
                    payload_len_lsb = 1;
                    payload_len_msb = 0;
                    outcome = 0x0;
                    header = {start_symbol, ep_id, trm16_id, cmd_id, payload_len_lsb, payload_len_msb};
                    footer = {outcome, cmd_crc_lsb, cmd_crc_msb, 0, 0, end_symbol};
                    payload.resize(payload_len_lsb);
                    payload = {static_cast<uint8_t>(mock_data.operation_mode)};
                    break;
                }
                case 8:  // w_configuration_mode
                {
                    payload_len_lsb = 0;
                    payload_len_msb = 0;
                    outcome = 0x0;
                    header = {start_symbol, ep_id, trm16_id, cmd_id, payload_len_lsb, payload_len_msb};
                    footer = {outcome, cmd_crc_lsb, cmd_crc_msb, 0, 0, end_symbol};
                    break;
                }
                case 9:  // r_configuration_mode
                {
                    payload_len_lsb = 1;
                    payload_len_msb = 0;
                    outcome = 0x0;
                    header = {start_symbol, ep_id, trm16_id, cmd_id, payload_len_lsb, payload_len_msb};
                    footer = {outcome, cmd_crc_lsb, cmd_crc_msb, 0, 0, end_symbol};
                    payload.resize(payload_len_lsb);
                    payload = {1};
                    break;
                }
                case 10:  // w_uuid
                {
                    payload_len_lsb = 0;
                    payload_len_msb = 0;
                    outcome = 0x0;
                    header = {start_symbol, ep_id, trm16_id, cmd_id, payload_len_lsb, payload_len_msb};
                    footer = {outcome, cmd_crc_lsb, cmd_crc_msb, 0, 0, end_symbol};

                    // Extract the uuid.
                    std::array<uint8_t, 19> uuid_array{};
                    std::copy(command_payload.cbegin(), command_payload.cend(), uuid_array.begin());
                    mock_data.uuid = Serial::merge_19_symbols(uuid_array);

                    break;
                }
                case 11:  // r_uuid
                {
                    payload_len_lsb = 19;
                    payload_len_msb = 0;
                    outcome = 0x0;
                    header = {start_symbol, ep_id, trm16_id, cmd_id, payload_len_lsb, payload_len_msb};
                    footer = {outcome, cmd_crc_lsb, cmd_crc_msb, 0, 0, end_symbol};

                    payload.resize(payload_len_lsb);

                    // Write the uuid to the payload.
                    const auto split = Serial::split_into_19_symbols(mock_data.uuid);
                    std::copy(split.cbegin(), split.cend(), payload.begin());

                    break;
                }
                case 12:  // r_telemetry
                {
                    payload_len_lsb = 13;
                    payload_len_msb = 0;
                    outcome = 0x0;
                    header = {start_symbol, ep_id, trm16_id, cmd_id, payload_len_lsb, payload_len_msb};
                    footer = {outcome, cmd_crc_lsb, cmd_crc_msb, 0, 0, end_symbol};
                    payload.resize(payload_len_lsb);
                    payload = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
                    break;
                }
                case 13:  // c_pgt
                {
                    payload_len_lsb = 0;
                    payload_len_msb = 0;
                    outcome = 0x0;
                    header = {start_symbol, ep_id, trm16_id, cmd_id, payload_len_lsb, payload_len_msb};
                    footer = {outcome, cmd_crc_lsb, cmd_crc_msb, 0, 0, end_symbol};
                    break;
                }
                case 14:  // w_pgt_rows
                {
                    payload_len_lsb = 0;
                    payload_len_msb = 0;
                    outcome = 0x0;
                    header = {start_symbol, ep_id, trm16_id, cmd_id, payload_len_lsb, payload_len_msb};
                    footer = {outcome, cmd_crc_lsb, cmd_crc_msb, 0, 0, end_symbol};
                    break;
                }
                case 15:  // r_pgt_rows
                {
                    payload_len_lsb = 100;
                    payload_len_msb = 0;
                    payload.resize(100);
                    std::fill(payload.begin(), payload.begin() + 50, 1);
                    payload[1] = 0;
                    std::fill(payload.begin() + 50, payload.end(), 2);
                    payload[51] = 0;
                    header = {start_symbol, ep_id, trm16_id, cmd_id, payload_len_lsb, payload_len_msb};
                    footer = {outcome, cmd_crc_lsb, cmd_crc_msb, 0, 0, end_symbol};
                    break;
                }
                case 16:  // w_pgt_row
                {
                    payload_len_lsb = 0;
                    payload_len_msb = 0;
                    outcome = 0x0;
                    header = {start_symbol, ep_id, trm16_id, cmd_id, payload_len_lsb, payload_len_msb};
                    footer = {outcome, cmd_crc_lsb, cmd_crc_msb, 0, 0, end_symbol};

                    SAS::Beam beam{};
                    beam.row_id = merge_two_symbols(command_payload[0], command_payload[1]);
                    std::memcpy(
                        beam.coefficients.data(),
                        command_payload.data() + 2,
                        sizeof(uint8_t) * SAS::c_entries_per_beam
                    );
                    if (mock_data.pgt_rows.size() <= beam.row_id)
                    {
                        mock_data.pgt_rows.resize(static_cast<size_t>(beam.row_id) + 1);
                    }
                    mock_data.pgt_rows[beam.row_id] = beam;

                    break;
                }
                case 17:  // r_pgt_row
                {
                    payload_len_lsb = 50;
                    payload_len_msb = 0;
                    outcome = 0x0;
                    header = {start_symbol, ep_id, trm16_id, cmd_id, payload_len_lsb, payload_len_msb};
                    footer = {outcome, cmd_crc_lsb, cmd_crc_msb, 0, 0, end_symbol};

                    payload.resize(payload_len_lsb);

                    payload[0] = command_payload[0];
                    payload[1] = command_payload[1];
                    const uint16_t row_id = merge_two_symbols(command_payload[0], command_payload[1]);
                    std::memcpy(
                        payload.data() + 2,
                        mock_data.pgt_rows[row_id].coefficients.data(),
                        sizeof(uint8_t) * SAS::c_entries_per_beam
                    );

                    break;
                }
                case 18:  // w_pgt_entry
                {
                    payload_len_lsb = 0;
                    payload_len_msb = 0;
                    outcome = 0x0;
                    header = {start_symbol, ep_id, trm16_id, cmd_id, payload_len_lsb, payload_len_msb};
                    footer = {outcome, cmd_crc_lsb, cmd_crc_msb, 0, 0, end_symbol};
                    break;
                }
                case 19:  // r_pgt_entry
                {
                    payload_len_lsb = 6;
                    payload_len_msb = 0;
                    outcome = 0x0;
                    header = {start_symbol, ep_id, trm16_id, cmd_id, payload_len_lsb, payload_len_msb};
                    footer = {outcome, cmd_crc_lsb, cmd_crc_msb, 0, 0, end_symbol};

                    payload.resize(payload_len_lsb);
                    payload[0] = command_payload[0];  // row_id_lsb
                    payload[1] = command_payload[1];  // row_id_msb
                    payload[2] = command_payload[2];  // trm_id
                    payload[3] = 1;  // entry[0]
                    payload[4] = 2;  // entry[1]
                    payload[5] = 3;  // entry[2]

                    break;
                }
                case 20:  // w_pgti
                {
                    payload_len_lsb = 0;
                    payload_len_msb = 0;
                    outcome = 0x0;
                    header = {start_symbol, ep_id, trm16_id, cmd_id, payload_len_lsb, payload_len_msb};
                    footer = {outcome, cmd_crc_lsb, cmd_crc_msb, 0, 0, end_symbol};

                    // Record the pgti.
                    mock_data.pgti = merge_two_symbols(command_payload.at(0), command_payload.at(1));

                    break;
                }
                case 21:  // r_pgti
                {
                    payload_len_lsb = 2;
                    payload_len_msb = 0;
                    outcome = 0x0;
                    header = {start_symbol, ep_id, trm16_id, cmd_id, payload_len_lsb, payload_len_msb};
                    footer = {outcome, cmd_crc_lsb, cmd_crc_msb, 0, 0, end_symbol};
                    payload.resize(payload_len_lsb);

                    const auto [pgti_lsb, pgti_msb] = split_into_two_symbols(mock_data.pgti);
                    payload = {pgti_lsb, pgti_msb};

                    break;
                }
                case 22:  // w_pgti_bounds
                {
                    payload_len_lsb = 0;
                    payload_len_msb = 0;
                    outcome = 0x0;
                    header = {start_symbol, ep_id, trm16_id, cmd_id, payload_len_lsb, payload_len_msb};
                    footer = {outcome, cmd_crc_lsb, cmd_crc_msb, 0, 0, end_symbol};

                    mock_data.pgti_min = merge_two_symbols(command_payload.at(0), command_payload.at(1));
                    mock_data.pgti_max = merge_two_symbols(command_payload.at(2), command_payload.at(3));

                    break;
                }
                case 23:  // r_pgti_bounds
                {
                    payload_len_lsb = 4;
                    payload_len_msb = 0;
                    outcome = 0x0;
                    header = {start_symbol, ep_id, trm16_id, cmd_id, payload_len_lsb, payload_len_msb};
                    footer = {outcome, cmd_crc_lsb, cmd_crc_msb, 0, 0, end_symbol};
                    payload.resize(payload_len_lsb);

                    const auto [pgti_min_lsb, pgti_min_msb] = split_into_two_symbols(mock_data.pgti_min);
                    const auto [pgti_max_lsb, pgti_max_msb] = split_into_two_symbols(mock_data.pgti_max);

                    payload = {pgti_min_lsb, pgti_min_msb, pgti_max_lsb, pgti_max_msb};

                    break;
                }
                case 24:  // w_index_strobe_mode
                {
                    payload_len_lsb = 0;
                    payload_len_msb = 0;
                    outcome = 0x0;
                    header = {start_symbol, ep_id, trm16_id, cmd_id, payload_len_lsb, payload_len_msb};
                    footer = {outcome, cmd_crc_lsb, cmd_crc_msb, 0, 0, end_symbol};
                    break;
                }
                case 25:  // r_index_strobe_mode
                {
                    payload_len_lsb = 1;
                    payload_len_msb = 0;
                    outcome = 0x0;
                    header = {start_symbol, ep_id, trm16_id, cmd_id, payload_len_lsb, payload_len_msb};
                    footer = {outcome, cmd_crc_lsb, cmd_crc_msb, 0, 0, end_symbol};
                    payload.resize(payload_len_lsb);
                    payload = {1};
                    break;
                }
                case 26:  // w_temperature_characterization
                {
                    payload_len_lsb = 0;
                    payload_len_msb = 0;
                    outcome = 0x0;
                    header = {start_symbol, ep_id, trm16_id, cmd_id, payload_len_lsb, payload_len_msb};
                    footer = {outcome, cmd_crc_lsb, cmd_crc_msb, 0, 0, end_symbol};
                    break;
                }
                case 27:  // r_temperature_characterization
                {
                    payload_len_lsb = 1;
                    payload_len_msb = 0;
                    outcome = 0x0;
                    header = {start_symbol, ep_id, trm16_id, cmd_id, payload_len_lsb, payload_len_msb};
                    footer = {outcome, cmd_crc_lsb, cmd_crc_msb, 0, 0, end_symbol};
                    payload.resize(payload_len_lsb);
                    payload = {1};
                    break;
                }
                case 28:  // w_tt
                {
                    payload_len_lsb = 0;
                    payload_len_msb = 0;
                    outcome = 0x0;
                    header = {start_symbol, ep_id, trm16_id, cmd_id, payload_len_lsb, payload_len_msb};
                    footer = {outcome, cmd_crc_lsb, cmd_crc_msb, 0, 0, end_symbol};
                    break;
                }
                case 29:  // r_tt
                {
                    payload_len_lsb = 16 * 2;
                    payload_len_msb = 0;
                    outcome = 0x0;
                    header = {start_symbol, ep_id, trm16_id, cmd_id, payload_len_lsb, payload_len_msb};
                    footer = {outcome, cmd_crc_lsb, cmd_crc_msb, 0, 0, end_symbol};

                    payload.resize(payload_len_lsb);
                    for (uint8_t i = 0; i < payload.size(); ++i)
                    {
                        payload.at(i) = i;
                    }

                    break;
                }
                case 30:  // w_tti
                {
                    payload_len_lsb = 0;
                    payload_len_msb = 0;
                    outcome = 0x0;
                    header = {start_symbol, ep_id, trm16_id, cmd_id, payload_len_lsb, payload_len_msb};
                    footer = {outcome, cmd_crc_lsb, cmd_crc_msb, 0, 0, end_symbol};
                    break;
                }
                case 31:  // r_tti
                {
                    payload_len_lsb = 1;
                    payload_len_msb = 0;
                    outcome = 0x0;
                    header = {start_symbol, ep_id, trm16_id, cmd_id, payload_len_lsb, payload_len_msb};
                    footer = {outcome, cmd_crc_lsb, cmd_crc_msb, 0, 0, end_symbol};
                    payload.resize(payload_len_lsb);
                    payload = {1};
                    break;
                }
                case 32:  // w_power
                {
                    payload_len_lsb = 0;
                    payload_len_msb = 0;
                    outcome = 0x0;
                    header = {start_symbol, ep_id, trm16_id, cmd_id, payload_len_lsb, payload_len_msb};
                    footer = {outcome, cmd_crc_lsb, cmd_crc_msb, 0, 0, end_symbol};

                    // Record power bitmask.
                    mock_data.power_mask_main =
                        static_cast<uint8_t>(merge_two_symbols(command_payload.at(0), command_payload.at(1)));
                    mock_data.power_mask_redundant =
                        static_cast<uint8_t>(merge_two_symbols(command_payload.at(2), command_payload.at(3)));
                    break;
                }
                case 33:  // r_power
                {
                    payload_len_lsb = 4;
                    payload_len_msb = 0;
                    outcome = 0x0;
                    header = {start_symbol, ep_id, trm16_id, cmd_id, payload_len_lsb, payload_len_msb};
                    footer = {outcome, cmd_crc_lsb, cmd_crc_msb, 0, 0, end_symbol};
                    payload.resize(payload_len_lsb);
                    const auto [power_main_lsb, power_main_msb] = split_into_two_symbols(mock_data.power_mask_main);
                    const auto [power_redundant_lsb, power_redundant_msb] =
                        split_into_two_symbols(mock_data.power_mask_redundant);
                    payload = {power_main_lsb, power_main_msb, power_redundant_lsb, power_redundant_msb};
                    break;
                }
                case 80:  // r_cc_regs
                {
                    payload_len_lsb = 18 * 5 * 2;
                    payload_len_msb = 0;
                    outcome = 0x0;
                    header = {start_symbol, ep_id, trm16_id, cmd_id, payload_len_lsb, payload_len_msb};
                    footer = {outcome, cmd_crc_lsb, cmd_crc_msb, 0, 0, end_symbol};
                    for (uint16_t i = 0; i < 18 * 5; ++i)
                    {
                        const auto [cc_reg_lsb, cc_reg_msb] = split_into_two_symbols(i);
                        payload.push_back(cc_reg_lsb);
                        payload.push_back(cc_reg_msb);
                    }
                    break;
                }
                case 81:  // w_override_temp_mode
                {
                    payload_len_lsb = 0;
                    payload_len_msb = 0;
                    outcome = 0x0;
                    header = {start_symbol, ep_id, trm16_id, cmd_id, payload_len_lsb, payload_len_msb};
                    footer = {outcome, cmd_crc_lsb, cmd_crc_msb, 0, 0, end_symbol};

                    // Record override temperature mode.
                    mock_data.override_temperature_mode = command_payload.at(0) == 1;
                    break;
                }
                case 82:  // r_override_temp_mode
                {
                    payload_len_lsb = 1;
                    payload_len_msb = 0;
                    outcome = 0x0;
                    header = {start_symbol, ep_id, trm16_id, cmd_id, payload_len_lsb, payload_len_msb};
                    footer = {outcome, cmd_crc_lsb, cmd_crc_msb, 0, 0, end_symbol};

                    payload.resize(payload_len_lsb);
                    payload = {mock_data.override_temperature_mode ? static_cast<uint8_t>(1) : static_cast<uint8_t>(0)};
                    break;
                }
                case 83:  // w_override_temp
                {
                    payload_len_lsb = 0;
                    payload_len_msb = 0;
                    outcome = 0x0;
                    header = {start_symbol, ep_id, trm16_id, cmd_id, payload_len_lsb, payload_len_msb};
                    footer = {outcome, cmd_crc_lsb, cmd_crc_msb, 0, 0, end_symbol};

                    // Record override temperature code.
                    mock_data.override_temperature_code = command_payload.at(0);
                    break;
                }
                case 84:  // r_override_temp
                {
                    payload_len_lsb = 2;
                    payload_len_msb = 0;
                    outcome = 0x0;
                    header = {start_symbol, ep_id, trm16_id, cmd_id, payload_len_lsb, payload_len_msb};
                    footer = {outcome, cmd_crc_lsb, cmd_crc_msb, 0, 0, end_symbol};

                    payload.resize(payload_len_lsb);
                    payload[0] = mock_data.override_temperature_code;
                    payload[1] = 0;
                    break;
                }
                default:
                    PLOG_ERROR.printf("Unknown command ID: %d", cmd_id);
                    break;
            }

            if (header.size() == 6 && footer.size() == 6)
            {
                response.resize(header.size() + payload.size() + footer.size());
                std::copy(header.cbegin(), header.cend(), response.begin());
                std::copy(payload.cbegin(), payload.cend(), response.begin() + header.size());
                std::copy(footer.cbegin(), footer.cend(), response.begin() + header.size() + payload.size());
                const auto [crc_lower, crc_upper] = split_into_two_symbols(calculate_crc14(response));
                footer = {outcome, cmd_crc_lsb, cmd_crc_msb, crc_lower, crc_upper, end_symbol};
                std::copy(footer.cbegin(), footer.cend(), response.begin() + header.size() + payload.size());
            }
            else
            {
                PLOG_ERROR.printf("Response is empty or too short. Sending invalid response to fail test.");
                response.resize(1);
                response = {0};
            }

            PLOG_DEBUG.printf("Encoded response bytes: %s", ::to_string(response.cbegin(), response.cend()).c_str());
            const long int bytes_written = write(m_serial_fd, response.data(), response.size());

            if (bytes_written < 0)
            {
                PLOG_ERROR.printf("Error writing to serial port: %s", strerror(errno));
            }
            else
            {
                PLOG_INFO.printf("Responded with %ld bytes", bytes_written);
            }
        }
    }

protected:

    Client m_client;
    std::atomic<bool>& m_running;
    int m_serial_fd;
    unsigned int m_timeout;
};

}  // namespace Serial

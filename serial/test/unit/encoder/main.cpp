#include <iostream>

#include <client.hpp>
#include <serial_comm_setup.hpp>
#include <test_utils.hpp>

using namespace Serial;

void
test_r_id(const Client& client);
void
test_r_serial_number(const Client& client);
void
test_c_error(const Client& client);
void
test_r_error(const Client& client);
void
test_reboot(const Client& client);
void
test_w_operation_mode(const Client& client);
void
test_r_operation_mode(const Client& client);
void
test_w_configuration_mode(const Client& client);
void
test_r_configuration_mode(const Client& client);
void
test_r_telemetry(const Client& client);
void
test_c_pgt(const Client& client);
void
test_w_pgt_rows(const Client& client);
void
test_r_pgt_rows(const Client& client);
void
test_w_pgt_row(const Client& client);
void
test_r_pgt_row(const Client& client);
void
test_w_pgt_entry(const Client& client);
void
test_r_pgt_entry(const Client& client);
void
test_w_pgti(const Client& client);
void
test_r_pgti(const Client& client);
void
test_w_pgti_bounds(const Client& client);
void
test_r_pgti_bounds(const Client& client);
void
test_w_index_strobe_mode(const Client& client);
void
test_r_index_strobe_mode(const Client& client);
void
test_w_temperature_characterization(const Client& client);
void
test_r_temperature_characterization(const Client& client);
void
test_w_tt(const Client& client);
void
test_r_tt(const Client& client);
void
test_w_tti(const Client& client);
void
test_r_tti(const Client& client);
void
test_w_power(const Client& client);
void
test_r_power(const Client& client);
void
test_r_cc_regs(const Client& client);
void
test_w_override_temp_mode(const Client& client);
void
test_r_override_temp_mode(const Client& client);
void
test_w_override_temp(const Client& client);
void
test_r_override_temp(const Client& client);

int
main()
{
    auto client = Client();
    // Skipping connection checks because we're testing the encoder.
    test_r_id(client);
    test_r_serial_number(client);
    test_c_error(client);
    test_r_error(client);
    test_reboot(client);
    test_w_operation_mode(client);
    test_r_operation_mode(client);
    test_w_configuration_mode(client);
    test_r_configuration_mode(client);
    test_r_telemetry(client);
    test_c_pgt(client);
    test_w_pgt_rows(client);
    test_r_pgt_rows(client);
    test_w_pgt_row(client);
    test_r_pgt_row(client);
    test_w_pgt_entry(client);
    test_r_pgt_entry(client);
    test_w_pgti(client);
    test_r_pgti(client);
    test_w_pgti_bounds(client);
    test_r_pgti_bounds(client);
    test_w_index_strobe_mode(client);
    test_r_index_strobe_mode(client);
    test_w_temperature_characterization(client);
    test_r_temperature_characterization(client);
    test_w_tt(client);
    test_r_tt(client);
    test_w_tti(client);
    test_r_tti(client);
    test_w_power(client);
    test_r_power(client);
    test_r_cc_regs(client);
    test_w_override_temp_mode(client);
    test_r_override_temp_mode(client);
    test_w_override_temp(client);
    test_r_override_temp(client);
    return 0;
}

void
test_r_id(const Client& client)
{
    for (uint8_t ep_id = 0; ep_id < 8; ++ep_id)
    {
        for (uint8_t trm16_id = 0; trm16_id < 64; ++trm16_id)
        {
            const uint8_t cmd_id = 1;
            const auto test_message = client.enc_r_id(ep_id, trm16_id);
            Message expected_message{ep_id, trm16_id, cmd_id};

            check(test_message.encode() == expected_message.encode());
        }
    }
}

void
test_r_serial_number(const Client& client)
{
    for (uint8_t ep_id = 0; ep_id < 8; ++ep_id)
    {
        for (uint8_t trm16_id = 0; trm16_id < 64; ++trm16_id)
        {
            const uint8_t cmd_id = 2;
            const auto test_message = client.enc_r_serial_number(ep_id, trm16_id);
            Message expected_message{ep_id, trm16_id, cmd_id};

            check(test_message.encode() == expected_message.encode());
        }
    }
}

void
test_c_error(const Client& client)
{
    for (uint8_t ep_id = 0; ep_id < 8; ++ep_id)
    {
        for (uint8_t trm16_id = 0; trm16_id < 64; ++trm16_id)
        {
            const uint8_t cmd_id = 3;
            const auto test_message = client.enc_c_error(ep_id, trm16_id);
            Message expected_message{ep_id, trm16_id, cmd_id};

            check(test_message.encode() == expected_message.encode());
        }
    }
}

void
test_r_error(const Client& client)
{
    for (uint8_t ep_id = 0; ep_id < 8; ++ep_id)
    {
        for (uint8_t trm16_id = 0; trm16_id < 64; ++trm16_id)
        {
            const uint8_t cmd_id = 4;
            const auto test_message = client.enc_r_error(ep_id, trm16_id);
            Message expected_message{ep_id, trm16_id, cmd_id};

            check(test_message.encode() == expected_message.encode());
        }
    }
}

void
test_reboot(const Client& client)
{
    for (uint8_t ep_id = 0; ep_id < 8; ++ep_id)
    {
        for (uint8_t trm16_id = 0; trm16_id < 64; ++trm16_id)
        {
            const uint8_t cmd_id = 5;
            const auto test_message = client.enc_reboot(ep_id, trm16_id);
            Message expected_message{ep_id, trm16_id, cmd_id};

            check(test_message.encode() == expected_message.encode());
        }
    }
}

void
test_w_operation_mode(const Client& client)
{
    for (uint8_t ep_id = 0; ep_id < 8; ++ep_id)
    {
        for (uint8_t trm16_id = 0; trm16_id < 64; ++trm16_id)
        {
            const uint8_t cmd_id = 6;
            const SAS::TRM::OperationMode operation_mode = SAS::TRM::OperationMode::Idle;
            const auto test_message = client.enc_w_operation_mode(ep_id, trm16_id, operation_mode);
            const std::vector<uint8_t> payload = {static_cast<uint8_t>(operation_mode)};
            Message expected_message{ep_id, trm16_id, cmd_id, payload};

            check(test_message.encode() == expected_message.encode());
        }
    }
}

void
test_r_operation_mode(const Client& client)
{
    for (uint8_t ep_id = 0; ep_id < 8; ++ep_id)
    {
        for (uint8_t trm16_id = 0; trm16_id < 64; ++trm16_id)
        {
            const uint8_t cmd_id = 7;
            const auto test_message = client.enc_r_operation_mode(ep_id, trm16_id);
            Message expected_message{ep_id, trm16_id, cmd_id};

            check(test_message.encode() == expected_message.encode());
        }
    }
}

void
test_w_configuration_mode(const Client& client)
{
    for (uint8_t ep_id = 0; ep_id < 8; ++ep_id)
    {
        for (uint8_t trm16_id = 0; trm16_id < 64; ++trm16_id)
        {
            const uint8_t cmd_id = 8;
            const bool configuration_mode = true;
            const auto test_message = client.enc_w_configuration_mode(ep_id, trm16_id, configuration_mode);
            const std::vector<uint8_t> payload = {static_cast<uint8_t>(configuration_mode)};
            Message expected_message{ep_id, trm16_id, cmd_id, payload};

            check(test_message.encode() == expected_message.encode());
        }
    }
}

void
test_r_configuration_mode(const Client& client)
{
    for (uint8_t ep_id = 0; ep_id < 8; ++ep_id)
    {
        for (uint8_t trm16_id = 0; trm16_id < 64; ++trm16_id)
        {
            const uint8_t cmd_id = 9;
            const auto test_message = client.enc_r_configuration_mode(ep_id, trm16_id);
            Message expected_message{ep_id, trm16_id, cmd_id};

            check(test_message.encode() == expected_message.encode());
        }
    }
}

void
test_w_uuid(const Client& client)
{
    for (uint8_t ep_id = 0; ep_id < 8; ++ep_id)
    {
        for (uint8_t trm16_id = 0; trm16_id < 64; ++trm16_id)
        {
            const uint8_t cmd_id = 10;
            const __uint128_t uuid = ((__uint128_t)0x1122334455667788 << 64) | 0x99AABBCCDDEEFF00;
            const auto test_message = client.enc_w_uuid(ep_id, trm16_id, uuid);
            std::array<uint8_t, 19> uuid_array = split_into_19_symbols(uuid);
            const std::vector<uint8_t> payload(uuid_array.cbegin(), uuid_array.cend());
            Message expected_message{ep_id, trm16_id, cmd_id, payload};

            check(test_message.encode() == expected_message.encode());
        }
    }
}

void
test_r_telemetry(const Client& client)
{
    for (uint8_t ep_id = 0; ep_id < 8; ++ep_id)
    {
        for (uint8_t trm16_id = 0; trm16_id < 64; ++trm16_id)
        {
            const uint8_t cmd_id = 12;
            const auto test_message = client.enc_r_telemetry(ep_id, trm16_id);
            Message expected_message{ep_id, trm16_id, cmd_id};

            check(test_message.encode() == expected_message.encode());
        }
    }
}

void
test_c_pgt(const Client& client)
{
    for (uint8_t ep_id = 0; ep_id < 8; ++ep_id)
    {
        for (uint8_t trm16_id = 0; trm16_id < 64; ++trm16_id)
        {
            const uint8_t cmd_id = 13;
            const auto test_message = client.enc_c_pgt(ep_id, trm16_id);
            Message expected_message{ep_id, trm16_id, cmd_id};

            check(test_message.encode() == expected_message.encode());
        }
    }
}

void
test_w_pgt_rows(const Client& client)
{
    (void)client;
    for (uint8_t ep_id = 0; ep_id < 8; ++ep_id)
    {
        for (uint8_t trm16_id = 0; trm16_id < 64; ++trm16_id)
        {
            /*
            const uint8_t cmd_id = 14;
            const uint16_t start_row = 0x01;
            const uint16_t n_beams = 200;
            std::vector<SAS::Beam> beams(n_beams);
            std::array<uint8_t, SAS::c_coefficients_size> coeff;
            std::fill(coeff.begin(), coeff.end(), 2);

            for (auto& beam: beams)
            {
                beam.row_id = 1;
                beam.coefficients = coeff;
            }

            const auto test_message = client.enc_w_pgt_rows(ep_id, trm16_id, beams);
            const auto [start_row_lsb, start_row_msb] = split_into_two_symbols(start_row);
            std::vector<uint8_t> payload(beams.size() + 4);
            payload[0] = start_row_lsb;
            payload[1] = start_row_msb;
            const auto [n_beams_lsb, n_beams_msb] = split_into_two_symbols(n_beams);
            payload[2] = n_beams_lsb;
            payload[3] = n_beams_msb;
            std::copy(
                beams.cbegin(),
                beams.cend(),
                payload.begin() + 4
            );
            Message expected_message{
                ep_id,
                trm16_id,
                cmd_id,
                payload
            };

            check(test_message.encode() == expected_message.encode());
            */
        }
    }
}

void
test_r_pgt_rows(const Client& client)
{
    for (uint8_t ep_id = 0; ep_id < 8; ++ep_id)
    {
        for (uint8_t trm16_id = 0; trm16_id < 64; ++trm16_id)
        {
            const uint8_t cmd_id = 15;
            const uint16_t row_id_1 = 1;
            const uint16_t row_id_2 = 2;
            std::array<uint8_t, SAS::c_coefficients_size> coeff_1 = {2, 3};
            std::array<uint8_t, SAS::c_coefficients_size> coeff_2 = {4, 5};
            std::fill(coeff_1.begin(), coeff_1.end(), 2);
            std::fill(coeff_2.begin(), coeff_2.end(), 3);
            const SAS::Beam beam_1{row_id_1, coeff_1};
            const SAS::Beam beam_2{row_id_2, coeff_2};
            const std::vector<SAS::Beam> beams{beam_1, beam_2};
            const auto test_message = client.enc_r_pgt_rows(ep_id, trm16_id, beams);
            const auto [row_id_1_lsb, row_id_1_msb] = split_into_two_symbols(row_id_1);
            const auto [row_id_2_lsb, row_id_2_msb] = split_into_two_symbols(row_id_2);
            const std::vector<uint8_t> payload = {row_id_1_lsb, row_id_1_msb, row_id_2_lsb, row_id_2_msb};
            Message expected_message{ep_id, trm16_id, cmd_id, payload};

            check(test_message.encode() == expected_message.encode());
        }
    }
}

void
test_w_pgt_row(const Client& client)
{
    for (uint8_t ep_id = 0; ep_id < 8; ++ep_id)
    {
        for (uint8_t trm16_id = 0; trm16_id < 64; ++trm16_id)
        {
            const uint8_t cmd_id = 16;
            const uint16_t row_id = 3;
            std::array<uint8_t, SAS::c_coefficients_size> coeff;
            std::fill(coeff.begin(), coeff.end(), 2);
            const SAS::Beam beam{row_id, coeff};
            const auto test_message = client.enc_w_pgt_row(ep_id, trm16_id, beam);
            std::vector<uint8_t> payload(SAS::c_beam_size);
            const auto [row_id_lsb, row_id_msb] = split_into_two_symbols(row_id);
            payload[0] = row_id_lsb;
            payload[1] = row_id_msb;
            std::copy(beam.coefficients.cbegin(), beam.coefficients.cend(), payload.begin() + 2);
            Message expected_message{ep_id, trm16_id, cmd_id, payload};

            check(test_message.encode() == expected_message.encode());
        }
    }
}

void
test_r_pgt_row(const Client& client)
{
    for (uint8_t ep_id = 0; ep_id < 8; ++ep_id)
    {
        for (uint8_t trm16_id = 0; trm16_id < 64; ++trm16_id)
        {
            const uint8_t cmd_id = 17;
            const uint16_t row_id = 1;
            std::array<uint8_t, SAS::c_coefficients_size> coeff;
            std::fill(coeff.begin(), coeff.end(), 2);
            const SAS::Beam beam{row_id, coeff};
            const auto test_message = client.enc_r_pgt_row(ep_id, trm16_id, beam);
            const auto [row_id_lsb, row_id_msb] = split_into_two_symbols(row_id);
            const std::vector<uint8_t> payload = {row_id_lsb, row_id_msb};
            Message expected_message{ep_id, trm16_id, cmd_id, payload};

            check(test_message.encode() == expected_message.encode());
        }
    }
}

void
test_w_pgt_entry(const Client& client)
{
    for (uint8_t ep_id = 0; ep_id < 8; ++ep_id)
    {
        for (uint8_t trm16_id = 0; trm16_id < 64; ++trm16_id)
        {
            const uint8_t cmd_id = 18;
            const std::array<uint8_t, 3> entry = {0x01, 0x02, 0x03};
            const uint16_t row_id = 1;
            const uint8_t trm_id = 1;
            const auto test_message = client.enc_w_pgt_entry(ep_id, trm16_id, row_id, trm_id, entry);
            const auto [row_id_lsb, row_id_msb] = split_into_two_symbols(row_id);
            std::vector<uint8_t> payload(entry.size() + 3);
            payload[0] = row_id_lsb;
            payload[1] = row_id_msb;
            payload[2] = trm_id;
            std::copy(entry.cbegin(), entry.cend(), payload.begin() + 3);
            Message expected_message{ep_id, trm16_id, cmd_id, payload};

            check(test_message.encode() == expected_message.encode());
        }
    }
}

void
test_r_pgt_entry(const Client& client)
{
    for (uint8_t ep_id = 0; ep_id < 8; ++ep_id)
    {
        for (uint8_t trm16_id = 0; trm16_id < 64; ++trm16_id)
        {
            const uint8_t cmd_id = 19;
            const uint16_t row_id = 1;
            const uint8_t trm_id = 1;
            const auto test_message = client.enc_r_pgt_entry(ep_id, trm16_id, row_id, trm_id);
            const auto [row_id_lsb, row_id_msb] = split_into_two_symbols(row_id);
            const std::vector<uint8_t> payload = {row_id_lsb, row_id_msb, trm_id};
            Message expected_message{ep_id, trm16_id, cmd_id, payload};

            check(test_message.encode() == expected_message.encode());
        }
    }
}

void
test_w_pgti(const Client& client)
{
    for (uint8_t ep_id = 0; ep_id < 8; ++ep_id)
    {
        for (uint8_t trm16_id = 0; trm16_id < 64; ++trm16_id)
        {
            const uint8_t cmd_id = 20;
            const uint16_t pgti = 1;
            const auto test_message = client.enc_w_pgti(ep_id, trm16_id, pgti);
            const auto [pgti_lsb, pgti_msb] = split_into_two_symbols(pgti);
            const std::vector<uint8_t> payload = {pgti_lsb, pgti_msb};
            Message expected_message{ep_id, trm16_id, cmd_id, payload};

            check(test_message.encode() == expected_message.encode());
        }
    }
}

void
test_r_pgti(const Client& client)
{
    for (uint8_t ep_id = 0; ep_id < 8; ++ep_id)
    {
        for (uint8_t trm16_id = 0; trm16_id < 64; ++trm16_id)
        {
            const uint8_t cmd_id = 21;
            const auto test_message = client.enc_r_pgti(ep_id, trm16_id);
            Message expected_message{ep_id, trm16_id, cmd_id};

            check(test_message.encode() == expected_message.encode());
        }
    }
}

void
test_w_pgti_bounds(const Client& client)
{
    for (uint8_t ep_id = 0; ep_id < 8; ++ep_id)
    {
        for (uint8_t trm16_id = 0; trm16_id < 64; ++trm16_id)
        {
            const uint8_t cmd_id = 22;
            const uint16_t lower_bound = 1;
            const uint16_t upper_bound = 2;
            const auto test_message = client.enc_w_pgti_bounds(ep_id, trm16_id, lower_bound, upper_bound);
            const auto [lower_bound_lsb, lower_bound_msb] = split_into_two_symbols(lower_bound);
            const auto [upper_bound_lsb, upper_bound_msb] = split_into_two_symbols(upper_bound);
            const std::vector<uint8_t> payload = {lower_bound_lsb, lower_bound_msb, upper_bound_lsb, upper_bound_msb};
            Message expected_message{ep_id, trm16_id, cmd_id, payload};

            check(test_message.encode() == expected_message.encode());
        }
    }
}

void
test_r_pgti_bounds(const Client& client)
{
    for (uint8_t ep_id = 0; ep_id < 8; ++ep_id)
    {
        for (uint8_t trm16_id = 0; trm16_id < 64; ++trm16_id)
        {
            const uint8_t cmd_id = 23;
            const auto test_message = client.enc_r_pgti_bounds(ep_id, trm16_id);
            Message expected_message{ep_id, trm16_id, cmd_id};

            check(test_message.encode() == expected_message.encode());
        }
    }
}

void
test_w_index_strobe_mode(const Client& client)
{
    for (uint8_t ep_id = 0; ep_id < 8; ++ep_id)
    {
        for (uint8_t trm16_id = 0; trm16_id < 64; ++trm16_id)
        {
            const uint8_t cmd_id = 24;
            const bool index_strobe_mode = true;
            const auto test_message = client.enc_w_index_strobe_mode(ep_id, trm16_id, index_strobe_mode);
            const std::vector<uint8_t> payload = {static_cast<uint8_t>(index_strobe_mode)};
            Message expected_message{ep_id, trm16_id, cmd_id, payload};

            check(test_message.encode() == expected_message.encode());
        }
    }
}

void
test_r_index_strobe_mode(const Client& client)
{
    for (uint8_t ep_id = 0; ep_id < 8; ++ep_id)
    {
        for (uint8_t trm16_id = 0; trm16_id < 64; ++trm16_id)
        {
            const uint8_t cmd_id = 25;
            const auto test_message = client.enc_r_index_strobe_mode(ep_id, trm16_id);
            Message expected_message{ep_id, trm16_id, cmd_id};

            check(test_message.encode() == expected_message.encode());
        }
    }
}

void
test_w_temperature_characterization(const Client& client)
{
    for (uint8_t ep_id = 0; ep_id < 8; ++ep_id)
    {
        for (uint8_t trm16_id = 0; trm16_id < 64; ++trm16_id)
        {
            const uint8_t cmd_id = 26;
            const bool temperature_characterization = true;
            const auto test_message =
                client.enc_w_temperature_characterization(ep_id, trm16_id, temperature_characterization);
            const std::vector<uint8_t> payload = {static_cast<uint8_t>(temperature_characterization)};
            Message expected_message{ep_id, trm16_id, cmd_id, payload};

            check(test_message.encode() == expected_message.encode());
        }
    }
}

void
test_r_temperature_characterization(const Client& client)
{
    for (uint8_t ep_id = 0; ep_id < 8; ++ep_id)
    {
        for (uint8_t trm16_id = 0; trm16_id < 64; ++trm16_id)
        {
            const uint8_t cmd_id = 27;
            const auto test_message = client.enc_r_temperature_characterization(ep_id, trm16_id);
            Message expected_message{ep_id, trm16_id, cmd_id};

            check(test_message.encode() == expected_message.encode());
        }
    }
}

void
test_w_tt(const Client& client)
{
    for (uint8_t ep_id = 0; ep_id < 8; ++ep_id)
    {
        for (uint8_t trm16_id = 0; trm16_id < 64; ++trm16_id)
        {
            const uint8_t cmd_id = 28;
            const std::array<uint8_t, 16> tt{};
            const auto test_message = client.enc_w_tt(ep_id, trm16_id, tt);
            std::vector<uint8_t> payload(tt.size());
            std::copy(tt.cbegin(), tt.cend(), payload.begin());
            Message expected_message{ep_id, trm16_id, cmd_id, payload};

            check(test_message.encode() == expected_message.encode());
        }
    }
}

void
test_r_tt(const Client& client)
{
    for (uint8_t ep_id = 0; ep_id < 8; ++ep_id)
    {
        for (uint8_t trm16_id = 0; trm16_id < 64; ++trm16_id)
        {
            const uint8_t cmd_id = 29;
            const auto test_message = client.enc_r_tt(ep_id, trm16_id);
            Message expected_message{ep_id, trm16_id, cmd_id};

            check(test_message.encode() == expected_message.encode());
        }
    }
}

void
test_w_tti(const Client& client)
{
    for (uint8_t ep_id = 0; ep_id < 8; ++ep_id)
    {
        for (uint8_t trm16_id = 0; trm16_id < 64; ++trm16_id)
        {
            const uint8_t cmd_id = 30;
            const uint8_t tti = 1;
            const auto test_message = client.enc_w_tti(ep_id, trm16_id, tti);
            const std::vector<uint8_t> payload = {tti};
            Message expected_message{ep_id, trm16_id, cmd_id, payload};

            check(test_message.encode() == expected_message.encode());
        }
    }
}

void
test_r_tti(const Client& client)
{
    for (uint8_t ep_id = 0; ep_id < 8; ++ep_id)
    {
        for (uint8_t trm16_id = 0; trm16_id < 64; ++trm16_id)
        {
            const uint8_t cmd_id = 31;
            const auto test_message = client.enc_r_tti(ep_id, trm16_id);
            Message expected_message{ep_id, trm16_id, cmd_id};

            check(test_message.encode() == expected_message.encode());
        }
    }
}

void
test_w_power(const Client& client)
{
    for (uint8_t ep_id = 0; ep_id < 8; ++ep_id)
    {
        const uint8_t cmd_id = 32;

        for (uint8_t power_mask_main = 1; power_mask_main < 0b11111111;
             power_mask_main |= static_cast<uint8_t>(power_mask_main << 1))
        {
            for (uint8_t power_mask_redundant = 1; power_mask_redundant < 0b11111111;
                 power_mask_redundant |= static_cast<uint8_t>(power_mask_redundant << 1))
            {
                const auto test_message = client.enc_w_power(ep_id, power_mask_main, power_mask_redundant);
                const std::vector<uint8_t> payload = {
                    static_cast<uint8_t>(power_mask_main & 0x7F),
                    static_cast<uint8_t>((power_mask_main >> 7) & 0x7F),
                    static_cast<uint8_t>(power_mask_redundant & 0x7F),
                    static_cast<uint8_t>((power_mask_redundant >> 7) & 0x7F)};
                Message expected_message{ep_id, 0, cmd_id, payload};
                check(test_message.encode() == expected_message.encode());
            }
        }
    }
}

void
test_r_power(const Client& client)
{
    for (uint8_t ep_id = 0; ep_id < 8; ++ep_id)
    {
        const uint8_t cmd_id = 33;
        const auto test_message = client.enc_r_power(ep_id);
        Message expected_message{ep_id, 0, cmd_id};

        check(test_message.encode() == expected_message.encode());
    }
}

void
test_r_cc_regs(const Client& client)
{
    for (uint8_t ep_id = 0; ep_id < 8; ++ep_id)
    {
        for (uint8_t trm16_id = 0; trm16_id < 64; ++trm16_id)
        {
            const uint8_t cmd_id = 80;
            const auto test_message = client.enc_r_cc_regs(ep_id, trm16_id);
            Message expected_message{ep_id, trm16_id, cmd_id};

            check(test_message.encode() == expected_message.encode());
        }
    }
}

void
test_w_override_temp_mode(const Client& client)
{
    for (uint8_t ep_id = 0; ep_id < 8; ++ep_id)
    {
        for (uint8_t trm16_id = 0; trm16_id < 64; ++trm16_id)
        {
            const uint8_t cmd_id = 81;

            for (bool override_temp_mode : {false, true})
            {
                const auto test_message = client.enc_w_override_temp_mode(ep_id, trm16_id, override_temp_mode);
                const std::vector<uint8_t> payload = {override_temp_mode};
                Message expected_message{ep_id, trm16_id, cmd_id, payload};
                check(test_message.encode() == expected_message.encode());
            }
        }
    }
}

void
test_r_override_temp_mode(const Client& client)
{
    for (uint8_t ep_id = 0; ep_id < 8; ++ep_id)
    {
        for (uint8_t trm16_id = 0; trm16_id < 64; ++trm16_id)
        {
            const uint8_t cmd_id = 82;
            const auto test_message = client.enc_r_override_temp_mode(ep_id, trm16_id);
            Message expected_message{ep_id, trm16_id, cmd_id};

            check(test_message.encode() == expected_message.encode());
        }
    }
}

void
test_w_override_temp(const Client& client)
{
    for (uint8_t ep_id = 0; ep_id < 8; ++ep_id)
    {
        for (uint8_t trm16_id = 0; trm16_id < 64; ++trm16_id)
        {
            const uint8_t cmd_id = 83;

            for (int override_temp = 0; override_temp < 256; ++override_temp)
            {
                const uint8_t override_temp_u8 = static_cast<uint8_t>(override_temp);
                const auto test_message = client.enc_w_override_temp(ep_id, trm16_id, override_temp_u8);
                std::vector<uint8_t> payload{override_temp_u8, 0};
                Message expected_message{ep_id, trm16_id, cmd_id, payload};
                check(test_message.encode() == expected_message.encode());
            }
        }
    }
}

void
test_r_override_temp(const Client& client)
{
    for (uint8_t ep_id = 0; ep_id < 8; ++ep_id)
    {
        for (uint8_t trm16_id = 0; trm16_id < 64; ++trm16_id)
        {
            const uint8_t cmd_id = 84;
            const auto test_message = client.enc_r_override_temp(ep_id, trm16_id);
            Message expected_message{ep_id, trm16_id, cmd_id};

            check(test_message.encode() == expected_message.encode());
        }
    }
}

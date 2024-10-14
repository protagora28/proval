#include <PlogHelper.hpp>
#include <client.hpp>
#include <serial_comm_args.hpp>
#include <serial_comm_setup.hpp>
#include <test_utils.hpp>

using namespace Serial;

int
main(int argc, const char* argv[])
{
    Args args{};
    if (!ParseArgs(argc, argv, args))
    {
        std::cerr << "Error parsing arguments." << std::endl;
        return 1;
    }

    args.log_stdout ? ms::PlogHelper::initPlogConsole(args.log_level.c_str())
                    : ms::PlogHelper::initPlog(args.log_file_path, args.log_level.c_str());

    PLOG_INFO.printf("Start Serial::Client.");
    args.print();

    // Create and start the Client
    Status status;
    auto client = Client();
    check(client.connect(status, args.serial_fd, set_serial_options, args.baud_rate, args.timeout, args.parity));
    check(status == Status::NoError);

    const uint8_t ep_id = 1;
    const uint8_t trm16_id = 1;

    PLOG_INFO.printf("Testing r_id.");
    std::array<uint8_t, 3> id;
    check(client.r_id(status, ep_id, trm16_id, id));
    // check(id == ?); // Skipping this check for now.

    PLOG_INFO.printf("Testing r_serial_number.");
    std::array<uint8_t, 3> serial_number;
    check(client.r_serial_number(status, ep_id, trm16_id, serial_number));
    // check(serial_number == ?); // Skipping this check for now.

    PLOG_INFO.printf("Testing c_error.");
    check(client.c_error(status, ep_id, trm16_id));

    PLOG_INFO.printf("Testing r_error.");
    uint8_t error;
    check(client.r_error(status, ep_id, trm16_id, error));
    check((error <= 0b1111) && (error & 0b1111));

    // check(client.status, reboot(ep_id, trm16_id, status)); // Skip because expectresponse.

    PLOG_INFO.printf("Testing w_operation_mode.");
    const SAS::TRM::OperationMode operation_mode{SAS::TRM::OperationMode::Operative};
    check(client.w_operation_mode(status, ep_id, trm16_id, operation_mode));

    PLOG_INFO.printf("Testing r_operation_mode.");
    SAS::TRM::OperationMode response;
    check(client.r_operation_mode(status, ep_id, trm16_id, response));
    check(response == operation_mode);

    PLOG_INFO.printf("Testing w_configuration_mode.");
    const bool configuration_mode_w{true};
    check(client.w_configuration_mode(status, ep_id, trm16_id, configuration_mode_w));

    PLOG_INFO.printf("Testing r_configuration_mode.");
    bool configuration_mode_r;
    check(client.r_configuration_mode(status, ep_id, trm16_id, configuration_mode_r));
    check(configuration_mode_r == configuration_mode_w);

    PLOG_INFO.printf("Testing r_telemetry.");
    std::array<uint8_t, Serial::c_telemetry_size> telemetry;
    check(client.r_telemetry(status, ep_id, trm16_id, telemetry));
    check(telemetry == std::array<uint8_t, Serial::c_telemetry_size>{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13});

    PLOG_INFO.printf("Testing c_pgt.");
    check(client.c_pgt(status, ep_id, trm16_id));

    {
        PLOG_INFO.printf("Testing w_pgt_rows.");
        SAS::Beam beam_w_1;
        beam_w_1.row_id = 1;
        std::fill(beam_w_1.coefficients.begin(), beam_w_1.coefficients.end(), 1);
        SAS::Beam beam_w_2;
        beam_w_2.row_id = 2;
        std::fill(beam_w_2.coefficients.begin(), beam_w_2.coefficients.end(), 2);
        std::vector<SAS::Beam> beams_w{beam_w_1, beam_w_2};
        check(client.w_pgt_rows(status, ep_id, trm16_id, beams_w));

        PLOG_INFO.printf("Testing r_pgt_rows.");
        SAS::Beam beam_r_1;
        beam_r_1.row_id = 1;
        SAS::Beam beam_r_2;
        beam_r_2.row_id = 2;
        std::vector<SAS::Beam> beams_r{beam_r_1, beam_r_2};
        check(client.r_pgt_rows(status, ep_id, trm16_id, beams_r));
        check(beams_w == beams_r);
    }

    {
        PLOG_INFO.printf("Testing w_pgt_row.");
        SAS::Beam beam_w;
        beam_w.row_id = 3;
        std::fill(beam_w.coefficients.begin(), beam_w.coefficients.end(), 1);
        check(client.w_pgt_row(status, ep_id, trm16_id, beam_w));

        PLOG_INFO.printf("Testing r_pgt_row.");
        SAS::Beam beam_r;
        beam_r.row_id = 3;
        check(client.r_pgt_row(status, ep_id, trm16_id, beam_r));
        check(beam_r == beam_w);
    }

    PLOG_INFO.printf("Testing w_pgt_entry.");
    const uint16_t row_id_e_w = 4;
    const uint8_t trm_id_e_w = 5;
    const std::array<uint8_t, 3> entry_w{1, 2, 3};
    check(client.w_pgt_entry(status, ep_id, trm16_id, row_id_e_w, trm_id_e_w, entry_w));

    PLOG_INFO.printf("Testing r_pgt_entry.");
    uint16_t row_id_e_r = 4;
    uint8_t trm_id_e_r = 5;
    std::array<uint8_t, 3> entry_r;
    check(client.r_pgt_entry(status, ep_id, trm16_id, row_id_e_r, trm_id_e_r, entry_r));
    check(row_id_e_r == row_id_e_w);
    check(trm_id_e_r == trm_id_e_w);
    check(entry_r == entry_w);

    PLOG_INFO.printf("Testing w_pgti.");
    const uint8_t pgti_w{10};
    check(client.w_pgti(status, ep_id, trm16_id, pgti_w));

    PLOG_INFO.printf("Testing r_pgti.");
    uint16_t pgti_r;
    check(client.r_pgti(status, ep_id, trm16_id, pgti_r));
    check(pgti_r == pgti_w);

    PLOG_INFO.printf("Testing w_pgti_bounds.");
    const uint8_t pgti_lower_bound_w{10};
    const uint8_t pgti_upper_bound_w{11};
    check(client.w_pgti_bounds(status, ep_id, trm16_id, pgti_lower_bound_w, pgti_upper_bound_w));

    PLOG_INFO.printf("Testing r_pgti_bounds.");
    uint16_t lower_bound_r;
    uint16_t upper_bound_r;
    check(client.r_pgti_bounds(status, ep_id, trm16_id, lower_bound_r, upper_bound_r));
    check(lower_bound_r == pgti_lower_bound_w);
    check(upper_bound_r == pgti_upper_bound_w);

    PLOG_INFO.printf("Testing w_index_strobe_mode.");
    const bool index_strobe_mode_w{true};
    check(client.w_index_strobe_mode(status, ep_id, trm16_id, index_strobe_mode_w));

    PLOG_INFO.printf("Testing r_index_strobe_mode.");
    bool index_strobe_mode_r;
    check(client.r_index_strobe_mode(status, ep_id, trm16_id, index_strobe_mode_r));
    check(index_strobe_mode_r == index_strobe_mode_w);

    // Skipping temperature characterization because it is not implemented yet.

    // const bool tc_w{true};
    // check(client.w_temperature_characterization(ep_id, trm16_id, tc_w, status));

    // uint8_t tc_r;
    // check(client.r_temperature_characterization(ep_id, trm16_id, tc_r, status));
    // check(tc_r == tc_w);

    // const std::array<uint8_t, 8 * 2> tt_w{};
    // check(client.w_tt(ep_id, trm16_id, tt_w, status));

    // std::array<uint8_t, 8 * 2> tt_r;
    // check(client.r_tt(ep_id, trm16_id, tt_r, status));
    // check(tt_r == tt_w);

    // const uint8_t tti_w{0};
    // check(client.w_tti(ep_id, trm16_id, tti_w, status));

    // uint8_t tti_r;
    // check(client.r_tti(ep_id, trm16_id, tti_r, status));
    // check(tti_r == tti_w);

    PLOG_INFO.printf("Testing w_power.");
    const uint8_t power_w_main{0b10101010};
    const uint8_t power_w_redundant{0b11111111};
    check(client.w_power(status, 0, power_w_main, power_w_redundant));

    PLOG_INFO.printf("Testing r_power.");
    uint8_t power_r_main = 0;
    uint8_t power_r_redundant = 0;
    check(client.r_power(status, 0, power_r_main, power_r_redundant));
    check(power_w_main == power_r_main);
    check(power_w_redundant == power_r_redundant);

    PLOG_INFO.printf("Success.");

    return 0;
}

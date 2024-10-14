#pragma once

#include <termios.h>

#include <array>
#include <cstring>
#include <functional>
#include <string>
#include <type_traits>
#include <vector>

#include <antenna.hpp>
#include <message.hpp>
#include <serial_comm_setup.hpp>

namespace Serial
{

constexpr size_t c_telemetry_size = 13;
constexpr int c_min_timeout = 100;  // Minimum timeout in milliseconds.
constexpr int c_max_timeout = 255 * 100;  // Maximum timeout in milliseconds.

class Client
{
private:

    typedef __uint128_t uuid_t;

    int m_serial_port{-1};

    std::function<void(termios&)> m_set_options{};
    speed_t m_baud_rate{0};
    int m_timeoutMillis{0};  // Timeout in milliseconds.
    Parity m_parity{Parity::None};

    struct termios m_tty
    {};

    int m_retries{0};
    int m_byte_time_estimate_ns{0};

    bool m_use_gpios{false};
    int m_fd_gpio_rx{-1};
    int m_fd_gpio_tx{-1};

    /**
     * @brief Send a message through the serial port and wait for a response.
     * @details Used to send both read and write commands. If successful, in response there will be stored the
     * answer. A select call is used on the m_serial_port both for reading and writing. Expects response to be sized
     * as the expected response size.
     */
    bool
    round_trip_comm(Status& status, const Message& message, Message& response, const bool silent_timeout = false) const;

    /**
     * @brief Send a message through the serial port.
     * @details Send a message through the serial port and wait for the acknowledge. Used to send write commands, that
     * is, commands which overwrite SAS properties. Fails if the read or write fail, if a CRC mismatch is detected, if a
     * negative acknowledge is received or if timeout is reached.
     * @param message Message to send.
     * @param status Communication status
     * @return True if the message was sent successfully and ack was received, false otherwise.
     */
    bool
    command(Status& status, const Message& message) const;

    /**
     * @brief Send a message through the serial port and receive a response.
     * @details Send a message through the serial port and receive a response. Used to send read commands, that
     * is, commands which read SAS properties. Fails if the read or write fail, if a CRC mismatch is detected, if a
     * negative acknowledge is received or if timeout is reached. If successful, in response there will be stored the
     * bytes read from the serial port, stripped of header and footer.
     * @param message Message to send.
     * @param response Response to receive.
     * @param status Communication status
     * @param silent_timeout If true, the timeout will not be logged as an error.
     * @return True if the message was sent successfully and response was received, false otherwise.
     */
    bool
    query(Status& status, const Message& message, Message& response, const bool silent_timeout = false) const;

    bool
    open_gpios(const int gpio_rx, const int gpio_tx);

    void
    close_gpios();

    bool
    set_idle_mode() const;

    bool
    set_tx_mode() const;

    bool
    set_rx_mode() const;

    bool
    set_gpios(const int value_rx, const int value_tx) const;

    bool
    apply_settings(Status& status);

public:

    /**
     * @brief Open and configure device for serial communication. Returns true if the configuration was successful,
     * false otherwise.
     * @details Calls open on the file descriptor and sets the termios serial port parameters.
     * @param serial_port Serial device name.
     * @param baud_rate Baud rate.
     * @param timeout Timeout in milliseconds, minimum 100.
     * @param parity Parity.
     * @param retries Number of retries.
     * @param use_gpios If true, the GPIOs will be used to control the direction of the serial port.
     * @param gpio_rx GPIO to control the RX direction.
     * @param gpio_tx GPIO to control the TX direction.
     * @return True if the configuration was successful, false otherwise.
     */
    bool
    connect(
        Status& status,
        const std::string& serial_port,
        const std::function<void(termios&)>& set_options,
        const speed_t baud_rate,
        const unsigned int timeout = 100,
        const Parity parity = Parity::None,
        const int retries = 0,
        const bool use_gpios = false,
        const int gpio_rx = -1,
        const int gpio_tx = -1
    );

    void
    disconnect();

    bool
    is_connected();

    void
    set_retries(const int retries);

    bool
    set_timeout(const unsigned int timeout);

    int
    get_fd();

    /**
     * @brief Destructor.
     * @details Closes the serial port.
     */
    ~Client();

    // Commands.

    bool
    r_id(
        Status& status,
        const uint8_t ep_id,
        const uint8_t trm16_id,
        std::array<uint8_t, 3>& id,
        const bool silent_timeout = false
    ) const;

    bool
    r_serial_number(Status& status, const uint8_t ep_id, const uint8_t trm16_id, std::array<uint8_t, 3>& serial_number)
        const;

    bool
    c_error(Status& status, const uint8_t ep_id, const uint8_t trm16_id) const;

    bool
    r_error(Status& status, const uint8_t ep_id, const uint8_t trm16_id, uint8_t& error) const;

    bool
    reboot(Status& status, const uint8_t ep_id, const uint8_t trm16_id) const;

    bool
    w_operation_mode(
        Status& status,
        const uint8_t ep_id,
        const uint8_t trm16_id,
        const SAS::TRM::OperationMode& operation_mode
    ) const;

    bool
    r_operation_mode(Status& status, const uint8_t ep_id, const uint8_t trm16_id, SAS::TRM::OperationMode& response)
        const;

    bool
    w_configuration_mode(Status& status, const uint8_t ep_id, const uint8_t trm16_id, const bool configuration_mode)
        const;

    bool
    r_configuration_mode(Status& status, const uint8_t ep_id, const uint8_t trm16_id, bool& configuration_mode) const;

    bool
    w_uuid(Status& status, const uint8_t ep_id, const uint8_t trm16_id, const uuid_t uuid) const;

    bool
    r_uuid(Status& status, const uint8_t ep_id, const uint8_t trm16_id, uuid_t& uuid) const;

    bool
    r_telemetry(
        Status& status,
        const uint8_t ep_id,
        const uint8_t trm16_id,
        std::array<uint8_t, Serial::c_telemetry_size>& telemetry
    ) const;

    bool
    c_pgt(Status& status, const uint8_t ep_id, const uint8_t trm16_id) const;

    bool
    w_pgt_rows(Status& status, const uint8_t ep_id, const uint8_t trm16_id, const std::vector<SAS::Beam>& rows) const;

    bool
    r_pgt_rows(Status& status, const uint8_t ep_id, const uint8_t trm16_id, std::vector<SAS::Beam>& rows) const;

    bool
    w_pgt_row(Status& status, const uint8_t ep_id, const uint8_t trm16_id, const SAS::Beam& beam) const;

    bool
    r_pgt_row(Status& status, const uint8_t ep_id, const uint8_t trm16_id, SAS::Beam& beam) const;

    bool
    w_pgt_entry(
        Status& status,
        const uint8_t ep_id,
        const uint8_t trm16_id,
        const uint16_t row_id,
        const uint8_t trm_id,
        const std::array<uint8_t, 3>& entry
    ) const;

    bool
    r_pgt_entry(
        Status& status,
        const uint8_t ep_id,
        const uint8_t trm16_id,
        const uint16_t row_id,
        const uint8_t trm_id,
        std::array<uint8_t, 3>& entry
    ) const;

    bool
    w_pgti(Status& status, const uint8_t ep_id, const uint8_t trm16_id, const uint16_t pgti) const;

    bool
    r_pgti(Status& status, const uint8_t ep_id, const uint8_t trm16_id, uint16_t& pgti) const;

    bool
    w_pgti_bounds(
        Status& status,
        const uint8_t ep_id,
        const uint8_t trm16_id,
        const uint16_t pgti_lower_bound,
        const uint16_t pgti_upper_bound
    ) const;

    bool
    r_pgti_bounds(
        Status& status,
        const uint8_t ep_id,
        const uint8_t trm16_id,
        uint16_t& pgti_lower_bound,
        uint16_t& pgti_upper_bound
    ) const;

    bool
    w_index_strobe_mode(Status& status, const uint8_t ep_id, const uint8_t trm16_id, const bool index_strobe_mode)
        const;

    bool
    r_index_strobe_mode(Status& status, const uint8_t ep_id, const uint8_t trm16_id, bool& index_strobe_mode) const;

    bool
    w_temperature_characterization(Status& status, const uint8_t ep_id, const uint8_t trm16_id, const bool tc) const;

    bool
    r_temperature_characterization(
        Status& status,
        const uint8_t ep_id,
        const uint8_t trm16_id,
        bool& temperature_characterization
    ) const;

    bool
    w_tt(Status& status, const uint8_t ep_id, const uint8_t trm16_id, const std::array<uint8_t, 8 * 2>& tt) const;

    bool
    r_tt(Status& status, const uint8_t ep_id, const uint8_t trm16_id, std::array<uint8_t, 8 * 2>& tt) const;

    bool
    w_tti(Status& status, const uint8_t ep_id, const uint8_t trm16_id, const uint8_t tti) const;

    bool
    r_tti(Status& status, const uint8_t ep_id, const uint8_t trm16_id, uint8_t& tti) const;

    bool
    w_power(Status& status, const uint8_t ep_id, const uint8_t power_mask_main, const uint8_t power_mask_redundant)
        const;

    bool
    r_power(Status& status, const uint8_t ep_id, uint8_t& power_mask_main, uint8_t& power_mask_redundant) const;

    bool
    r_cc_regs(
        Status& status,
        const uint8_t ep_id,
        const uint8_t trm16_id,
        std::array<uint16_t, SAS::c_cc_regs_per_trm16>& cc_regs
    ) const;

    bool
    w_override_temp_mode(Status& status, const uint8_t ep_id, const uint8_t trm16_id, const bool override_temp_mode)
        const;

    bool
    r_override_temp_mode(Status& status, const uint8_t ep_id, const uint8_t trm16_id, bool& override_temp_mode) const;

    bool
    w_override_temp(Status& status, const uint8_t ep_id, const uint8_t trm16_id, const uint8_t override_temp) const;

    bool
    r_override_temp(Status& status, const uint8_t ep_id, const uint8_t trm16_id, uint8_t& override_temp) const;

    // Encoding methods.

    Message
    enc_r_id(const uint8_t ep_id, const uint8_t trm16_id) const;

    Message
    enc_r_serial_number(const uint8_t ep_id, const uint8_t trm16_id) const;

    Message
    enc_c_error(const uint8_t ep_id, const uint8_t trm16_id) const;

    Message
    enc_r_error(const uint8_t ep_id, const uint8_t trm16_id) const;

    Message
    enc_reboot(const uint8_t ep_id, const uint8_t trm16_id) const;

    Message
    enc_w_operation_mode(const uint8_t ep_id, const uint8_t trm16_id, const SAS::TRM::OperationMode operation_mode)
        const;

    Message
    enc_r_operation_mode(const uint8_t ep_id, const uint8_t trm16_id) const;

    Message
    enc_w_configuration_mode(const uint8_t ep_id, const uint8_t trm16_id, const bool configuration_mode) const;

    Message
    enc_r_configuration_mode(const uint8_t ep_id, const uint8_t trm16_id) const;

    Message
    enc_w_uuid(const uint8_t ep_id, const uint8_t trm16_id, const uuid_t uuid) const;

    Message
    enc_r_uuid(const uint8_t ep_id, const uint8_t trm16_id) const;

    Message
    enc_r_telemetry(const uint8_t ep_id, const uint8_t trm16_id) const;

    Message
    enc_c_pgt(const uint8_t ep_id, const uint8_t trm16_id) const;

    Message
    enc_w_pgt_rows(const uint8_t ep_id, const uint8_t trm16_id, const std::vector<SAS::Beam>& beams) const;

    Message
    enc_r_pgt_rows(const uint8_t ep_id, const uint8_t trm16_id, const std::vector<SAS::Beam>& beams) const;

    Message
    enc_w_pgt_row(const uint8_t ep_id, const uint8_t trm16_id, const SAS::Beam& beam) const;

    Message
    enc_r_pgt_row(const uint8_t ep_id, const uint8_t trm16_id, const SAS::Beam& beam) const;

    Message
    enc_w_pgt_entry(
        const uint8_t ep_id,
        const uint8_t trm16_id,
        const uint16_t row_id,
        const uint8_t trm_id,
        const std::array<uint8_t, 3>& entry
    ) const;

    Message
    enc_r_pgt_entry(const uint8_t ep_id, const uint8_t trm16_id, const uint16_t row_id, const uint8_t trm_id) const;

    Message
    enc_w_pgti(const uint8_t ep_id, const uint8_t trm16_id, const uint16_t pgti) const;

    Message
    enc_r_pgti(const uint8_t ep_id, const uint8_t trm16_id) const;

    Message
    enc_w_pgti_bounds(
        const uint8_t ep_id,
        const uint8_t trm16_id,
        const uint16_t lower_bound,
        const uint16_t upper_bound
    ) const;

    Message
    enc_r_pgti_bounds(const uint8_t ep_id, const uint8_t trm16_id) const;

    Message
    enc_w_index_strobe_mode(const uint8_t ep_id, const uint8_t trm16_id, const bool index_strobe_mode) const;

    Message
    enc_r_index_strobe_mode(const uint8_t ep_id, const uint8_t trm16_id) const;

    Message
    enc_w_temperature_characterization(const uint8_t ep_id, const uint8_t trm16_id, const bool tc) const;

    Message
    enc_r_temperature_characterization(const uint8_t ep_id, const uint8_t trm16_id) const;

    Message
    enc_w_tt(const uint8_t ep_id, const uint8_t trm16_id, const std::array<uint8_t, 8 * 2>& tt) const;

    Message
    enc_r_tt(const uint8_t ep_id, const uint8_t trm16_id) const;

    Message
    enc_w_tti(const uint8_t ep_id, const uint8_t trm16_id, const uint8_t tti) const;

    Message
    enc_r_tti(const uint8_t ep_id, const uint8_t trm16_id) const;

    Message
    enc_w_power(const uint8_t ep_id, const uint8_t power_mask_main, const uint8_t power_mask_redundant) const;

    Message
    enc_r_power(const uint8_t ep_id) const;

    Message
    enc_r_cc_regs(const uint8_t ep_id, const uint8_t trm16_id) const;

    Message
    enc_w_override_temp_mode(const uint8_t ep_id, const uint8_t trm16_id, const bool override_temp_mode) const;

    Message
    enc_r_override_temp_mode(const uint8_t ep_id, const uint8_t trm16_id) const;

    Message
    enc_w_override_temp(const uint8_t ep_id, const uint8_t trm16_id, const uint8_t override_temp) const;

    Message
    enc_r_override_temp(const uint8_t ep_id, const uint8_t trm16_id) const;
};

std::array<uint8_t, 19>
split_into_19_symbols(__uint128_t value);

__uint128_t
merge_19_symbols(const std::array<uint8_t, 19>& symbols);

int
baud_rate_to_int(const speed_t speed);

template<typename T, std::size_t n, std::size_t n_bytes = sizeof(T) * 8 / n + 1>
std::array<uint8_t, n_bytes>
split_to_n_bit_symbols(const T value)
{
    static_assert(std::is_unsigned<T>::value, "Input type must be an unsigned integer type.");
    constexpr std::size_t mask = (1 << n) - 1;
    std::array<uint8_t, n> result;
    std::generate(
        result.begin(),
        result.end(),
        [&value, i_symbol = 0]() mutable { return (value >> (i_symbol++ * n)) & mask; }
    );

    return result;
}

template<typename T, std::size_t N = sizeof(T) * 8 / 7 + 1>
T
merge_7bit_symbols(const std::array<uint8_t, N>& symbols)
{
    static_assert(std::is_unsigned<T>::value, "Output type must be an unsigned integer type.");
    T result = 0;
    for (std::size_t i_symbol = 0; i_symbol < N; ++i_symbol)
    {
        result |= (symbols[i_symbol] & 0x7F) << (i_symbol * 7);
    }

    return result;
}

}  // namespace Serial

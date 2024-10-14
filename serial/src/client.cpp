#include <fcntl.h>
#include <plog/Log.h>
#include <unistd.h>

#include <chrono>
#include <thread>

#include <client.hpp>
#include <safe_io.hpp>
#include <serial_comm_setup.hpp>

namespace Serial
{

Client::~Client()
{
    disconnect();
}

bool
Client::connect(
    Status& status,
    const std::string& serial_port,
    const std::function<void(termios&)>& set_options,
    const speed_t baud_rate,
    const unsigned int timeout,
    const Parity parity,
    const int retries,
    const bool use_gpios,
    const int gpio_rx,
    const int gpio_tx
)
{
    status = Status::NoError;

    if (is_connected())
    {
        PLOG_ERROR.printf("Already connected.");
        status = Status::AlreadyConnectedError;
        return false;
    }

    m_serial_port = open(serial_port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);

    if (m_serial_port < 0)
    {
        PLOG_ERROR.printf("Opening %s failed: %s", serial_port.c_str(), strerror(errno));
        status = Status::OpenError;
        return false;
    }

    m_set_options = set_options;
    m_baud_rate = baud_rate;
    m_timeoutMillis = timeout;
    m_parity = parity;

    apply_settings(status);

    const Parity effective_parity =
        (m_tty.c_cflag & PARENB) ? ((m_tty.c_cflag & PARODD) ? Parity::Odd : Parity::Even) : Parity::None;
    const int stop_bits = (m_tty.c_cflag & CSTOPB) ? 2 : 1;
    const int baud_rate_int = baud_rate_to_int(baud_rate);
    const int bits_per_byte = 8 + stop_bits + (effective_parity != Parity::None ? 1 : 0);
    m_byte_time_estimate_ns = static_cast<int>(
        (static_cast<int64_t>(1e9) * static_cast<int64_t>(bits_per_byte)) / static_cast<int64_t>(baud_rate_int)
    );

    PLOG_DEBUG.printf(
        "Effective parity setting is %s.",
        effective_parity == Parity::None       ? "none"
            : effective_parity == Parity::Even ? "even"
                                               : "odd"
    );

    PLOG_DEBUG.printf("Stop bits setting is %d.", stop_bits);
    PLOG_DEBUG.printf("Baud rate setting is %d.", baud_rate_int);
    PLOG_DEBUG.printf("Estimated byte time is %d ns.", m_byte_time_estimate_ns);

    set_retries(retries);

    m_use_gpios = use_gpios;
    if (!open_gpios(gpio_rx, gpio_tx))
    {
        PLOG_ERROR.printf("Failed to open GPIOs.");
        status = Status::GPIOError;
        return false;
    }

    return true;
}

void
Client::disconnect()
{
    if (is_connected())
    {
        if (close(m_serial_port) != 0)
        {
            PLOG_ERROR.printf("Failed to close serial port: %s", strerror(errno));
        }
        m_serial_port = -1;

        close_gpios();
    }
}

bool
Client::is_connected()
{
    return m_serial_port >= 0;
}

void
Client::set_retries(const int retries)
{
    PLOG_WARNING_IF(retries < 0).printf("Retries set to a negative value: %d. Setting to 0.", retries);
    PLOG_INFO.printf("Setting retries to %d.", retries);
    m_retries = std::max<int>(0, retries);
}

bool
Client::set_timeout(const unsigned int timeout)
{
    PLOG_INFO.printf("Setting timeout to %d ms.", timeout);
    m_timeoutMillis = timeout;
    Status status{};
    const bool result = apply_settings(status);
    PLOG_ERROR_IF(!result).printf("Failed to set timeout: %s.", status_message_to_string(status).c_str());
    return result;
}

int
Client::get_fd()
{
    return m_serial_port;
}

bool
Client::round_trip_comm(Status& status, const Message& command, Message& response, const bool silent_timeout) const
{
    // TODO Log the command and response.
    const auto start = std::chrono::high_resolution_clock::now();
    status = Status::NoError;

    if (response.size() < Message::c_min_length)
    {
        PLOG_ERROR.printf("Response size too small: %lu", response.size());
        return false;
    }

    if (!set_idle_mode())
    {
        PLOG_ERROR.printf("Failed to set idle mode.");
        status = Status::GPIOError;
        return false;
    }

    const int attempts = m_retries + 1;
    int sleepTimeMillis = m_timeoutMillis;  // Initial sleep time.
    bool success = false;
    for (int attempt = 0; attempt < attempts; ++attempt)
    {
        if (attempt > 0)
        {
            PLOG_INFO
                .printf("Attempt %d/%d, sleeping for %d ms before retrying.", attempt + 1, attempts, sleepTimeMillis);
            // Exponential backoff.
            std::this_thread::sleep_for(std::chrono::milliseconds(sleepTimeMillis));
            sleepTimeMillis *= 2;  // TODO Add jitter.
        }

        if (!set_tx_mode())
        {
            PLOG_ERROR.printf("Failed to set TX mode.");
            status = Status::GPIOError;
            continue;
        }
        constexpr bool drain = true;
        if (safe_write(m_serial_port, command, status, drain, m_byte_time_estimate_ns))
        {
            if (!set_rx_mode())
            {
                PLOG_ERROR.printf("Failed to set RX mode.");
                status = Status::GPIOError;
                continue;
            }
            // Successfull write, try to read.
            if (safe_read(
                    m_serial_port,
                    m_timeoutMillis,
                    response,
                    status,
                    silent_timeout ? TimeoutSeverity::Info : TimeoutSeverity::Warning
                ))
            {
                // Successfull read, validate the response.

                if (!response.is_valid(status))
                {
                    PLOG_ERROR.printf(
                        "Invalid response at attempt %d/%d with error: %s.",
                        attempt + 1,
                        attempts,
                        status_message_to_string(status).c_str()
                    );
                    continue;
                }

                if (!response.check_integrity(command, status))
                {
                    PLOG_ERROR.printf(
                        "Response integrity check failed at attempt %d/%d with error: %s.",
                        attempt + 1,
                        attempts,
                        status_message_to_string(status).c_str()
                    );
                    continue;
                }

                // Valid response.
                success = true;
                break;
            }
            else
            {
                if (silent_timeout && status == Status::Timeout)
                {
                    // Expected timeout, try again to see if the timeout is consistent.
                    PLOG_INFO.printf("Expected timeout at attempt %d/%d.", attempt + 1, attempts);
                }
                else
                {
                    // Read failed.
                    PLOG_WARNING.printf(
                        "Read failed at attempt %d/%d with error: %s.",
                        attempt + 1,
                        attempts,
                        status_message_to_string(status).c_str()
                    );
                }
            }
        }
        else
        {
            // Write failed.
            PLOG_WARNING.printf(
                "Write failed at attempt %d/%d with error: %s.",
                attempt + 1,
                attempts,
                status_message_to_string(status).c_str()
            );
        }
    }

    if (!set_idle_mode())
    {
        PLOG_ERROR.printf("Failed to set idle mode.");
        status = Status::GPIOError;
        return false;
    }

    const auto end = std::chrono::high_resolution_clock::now();
    const auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);

    PLOG_INFO.printf(
        "Elapsed time: %f ms, speed: %.4f Mb/s",
        static_cast<double>(elapsed.count()) / 1e6,
        static_cast<double>(command.size() + response.size()) / (static_cast<double>(elapsed.count()) / 1e9) * 8 / 1e6
    );

    if (!success)
    {
        if (silent_timeout && status == Status::Timeout)
        {
            PLOG_INFO.printf("Expected timeout.");
        }
        else
        {
            PLOG_ERROR.printf(
                "Round trip communication failed after %d attempts, last error is: %s.",
                attempts,
                status_message_to_string(status).c_str()
            );
        }
        return false;
    }

    return true;
}

bool
Client::command(Status& status, const Message& command_message) const
{
    Message response{0};

    return query(status, command_message, response);
}

bool
Client::query(Status& status, const Message& query_message, Message& response, const bool silent_timeout) const
{
    if (!round_trip_comm(status, query_message, response, silent_timeout))
    {
        if (silent_timeout && status == Status::Timeout)
        {
            PLOG_INFO.printf("Expected timeout.");
        }
        else
        {
            PLOG_ERROR.printf("Query failed.");
        }
        return false;
    }

    return true;
}

bool
Client::open_gpios(const int gpio_rx, const int gpio_tx)
{
    if (!m_use_gpios)
    {
        return true;
    }

    const std::string rx_gpio_a_str = "/sys/class/gpio/gpio" + std::to_string(gpio_rx) + "/value";
    const std::string tx_gpio_a_str = "/sys/class/gpio/gpio" + std::to_string(gpio_tx) + "/value";

    PLOG_DEBUG.printf("Opening RX GPIO %d value file: %s", gpio_rx, rx_gpio_a_str.c_str());
    PLOG_DEBUG.printf("Opening TX GPIO %d value file: %s", gpio_tx, tx_gpio_a_str.c_str());

    // Open the GPIO value files.
    m_fd_gpio_rx = open(rx_gpio_a_str.c_str(), O_WRONLY);
    if (m_fd_gpio_rx == -1)
    {
        PLOG_ERROR.printf("Failed to open RX GPIO %d value file: %s", gpio_rx, strerror(errno));
        return false;
    }
    else
    {
        PLOG_DEBUG.printf("Opened RX GPIO %d value file.", gpio_rx);
    }
    m_fd_gpio_tx = open(tx_gpio_a_str.c_str(), O_WRONLY);
    if (m_fd_gpio_tx == -1)
    {
        PLOG_ERROR.printf("Failed to open TX GPIO %d value file: %s", gpio_tx, strerror(errno));
        return false;
    }
    else
    {
        PLOG_DEBUG.printf("Opened TX GPIO %d value file.", gpio_tx);
    }
    return true;
}

void
Client::close_gpios()
{
    // Close the GPIO value files.
    if (m_fd_gpio_rx != -1)
    {
        if (close(m_fd_gpio_rx) == -1)
        {
            PLOG_ERROR.printf("Failed to close RX GPIO value file: %s", strerror(errno));
        }
        else
        {
            PLOG_DEBUG.printf("Closed RX GPIO value file.");
        }
        m_fd_gpio_rx = -1;
    }
    if (m_fd_gpio_tx != -1)
    {
        if (close(m_fd_gpio_tx) == -1)
        {
            PLOG_ERROR.printf("Failed to close TX GPIO value file: %s", strerror(errno));
        }
        else
        {
            PLOG_DEBUG.printf("Closed TX GPIO value file.");
        }
        m_fd_gpio_tx = -1;
    }
}

bool
Client::set_idle_mode() const
{
    if (!m_use_gpios)
    {
        return true;
    }
    return set_gpios(1, 0);
}

bool
Client::set_tx_mode() const
{
    if (!m_use_gpios)
    {
        return true;
    }
    return set_idle_mode() && set_gpios(1, 1);
}

bool
Client::set_rx_mode() const
{
    if (!m_use_gpios)
    {
        return true;
    }
    return set_idle_mode() && set_gpios(0, 0);
}

bool
Client::set_gpios(const int value_rx, const int value_tx) const
{
    const char value_char_rx = value_rx ? '1' : '0';
    const char value_char_tx = value_tx ? '1' : '0';
    if (write(m_fd_gpio_rx, &value_char_rx, 1) != 1)
    {
        PLOG_ERROR.printf("Failed to write RX GPIO value %d to file: %s", value_rx, strerror(errno));
        return false;
    }
    else
    {
        PLOG_DEBUG.printf("Wrote RX GPIO value %d to file.", value_rx);
    }
    if (write(m_fd_gpio_tx, &value_char_tx, 1) != 1)
    {
        PLOG_ERROR.printf("Failed to write TX GPIO value %s to file: %s", value_tx, strerror(errno));
        return false;
    }
    else
    {
        PLOG_DEBUG.printf("Wrote TX GPIO value %d to file.", value_tx);
    }
    return true;
}

bool
Client::apply_settings(Status& status)
{
    status = Status::NoError;

    if (!is_connected())
    {
        PLOG_ERROR.printf("Not connected.");
        status = Status::OpenError;  // TODO Change to NotConnectedError.
        return false;
    }

    if (m_timeoutMillis < c_min_timeout)
    {
        PLOG_ERROR.printf("Timeout must be at least %d ms.", c_min_timeout);
        status = Status::InvalidTimeout;
        return false;
    }

    if (m_timeoutMillis > c_max_timeout)
    {
        PLOG_ERROR.printf("Timeout must be at most %d ms.", c_max_timeout);
        status = Status::InvalidTimeout;
        return false;
    }

    memset(&m_tty, 0, sizeof(m_tty));

    if (tcgetattr(m_serial_port, &m_tty) != 0)
    {
        PLOG_ERROR.printf("Failed tcgetattr call: %s", strerror(errno));
        status = Status::TCGetAttrError;
        return false;
    }

    struct termios current_options;
    std::memcpy(&current_options, &m_tty, sizeof(m_tty));

    m_set_options(m_tty);

    // Override parity setting.
    PLOG_DEBUG.printf(
        "Overriding parity setting to %s.",
        m_parity == Parity::None       ? "none"
            : m_parity == Parity::Even ? "even"
                                       : "odd"
    );
    switch (m_parity)
    {
        case Parity::Odd:
            m_tty.c_cflag |= (PARENB | PARODD);
            break;
        case Parity::Even:
            m_tty.c_cflag |= PARENB;
            m_tty.c_cflag &= ~PARODD;
            break;
        case Parity::None:
            m_tty.c_cflag &= ~PARENB;
            break;
        default:
            break;
    }

    // Setting VTIME > 0 and VMIN = 0 means that read() will block until timeout.
    m_tty.c_cc[VTIME] =
        static_cast<cc_t>(static_cast<double>(m_timeoutMillis) / 1000.0 * 10.0);  // Conversion to deciseconds.
    m_tty.c_cc[VMIN] = 0;  // Enables read to return 0 when no data is available after timeout.

    if (cfsetispeed(&m_tty, m_baud_rate) != 0)
    {
        PLOG_ERROR.printf("Failed cfsetispeed call: %s", strerror(errno));
        status = Status::CFSetISpeedError;
        return false;
    }

    const auto cfgetispeed_result = cfgetispeed(&m_tty);
    if (cfgetispeed_result != m_baud_rate)
    {
        PLOG_ERROR.printf(
            "Input speed mismatch, expected %s got %s instead.",
            strerror(errno),
            std::to_string(m_baud_rate).c_str(),
            std::to_string(cfgetispeed_result).c_str()
        );
        status = Status::ISpeedMismatch;
        return false;
    }

    if (cfsetospeed(&m_tty, m_baud_rate) != 0)
    {
        PLOG_ERROR.printf("Failed cfsetospeed call: %s", strerror(errno));
        status = Status::CFSetOSpeedError;
        return false;
    }

    const auto cfgetospeed_result = cfgetospeed(&m_tty);
    if (cfgetospeed_result != m_baud_rate)
    {
        PLOG_ERROR.printf(
            "Output speed mismatch, expected %s got %s instead.",
            strerror(errno),
            std::to_string(m_baud_rate).c_str(),
            std::to_string(cfgetospeed_result).c_str()
        );
        status = Status::OSpeedMismatch;
        return false;
    }

    if (std::memcmp(&m_tty, &current_options, sizeof(m_tty)) != 0)
    {
        // Apply the new settings.
        if (tcsetattr(m_serial_port, TCSANOW, &m_tty) != 0)
        {
            PLOG_ERROR.printf("Failed tcsetattr call: %s", strerror(errno));
            status = Status::TCSetAttrError;
            return false;
        }

        // Verify the new settings.
        struct termios new_options;
        memset(&new_options, 0, sizeof(new_options));
        if (tcgetattr(m_serial_port, &new_options) != 0)
        {
            PLOG_ERROR.printf("Failed tcgetattr call: %s", strerror(errno));
            status = Status::TCGetAttrError;
            return false;
        }

        if (std::memcmp(&m_tty, &new_options, sizeof(m_tty)) != 0)
        {
            PLOG_ERROR.printf("Failed to set serial port options.");
            status = Status::OptionsMismatch;
            return false;
        }
    }

    if (tcflush(m_serial_port, TCIOFLUSH) != 0)
    {
        PLOG_ERROR.printf("Failed tcflush call: %s", strerror(errno));
        status = Status::TCFlushError;
        return false;
    }

    return true;
}

bool
Client::r_id(
    Status& status,
    const uint8_t ep_id,
    const uint8_t trm16_id,
    std::array<uint8_t, 3>& id,
    const bool silent_timeout
) const
{
    const Message message = enc_r_id(ep_id, trm16_id);

    Message response{id.size()};

    if (!query(status, message, response, silent_timeout))
    {
        if (silent_timeout && status == Status::Timeout)
        {
            PLOG_INFO.printf("Expected timeout.");
        }
        else
        {
            PLOG_ERROR.printf("Query failed.");
        }
        return false;
    }

    const auto response_payload = response.get_payload();
    std::copy(response_payload.cbegin(), response_payload.cend(), id.begin());

    return true;
}

bool
Client::r_serial_number(
    Status& status,
    const uint8_t ep_id,
    const uint8_t trm16_id,
    std::array<uint8_t, 3>& serial_number
) const
{
    const Message message = enc_r_serial_number(ep_id, trm16_id);

    Message response{serial_number.size()};

    if (!query(status, message, response))
    {
        PLOG_ERROR.printf("Query failed.");
        return false;
    }

    const auto response_payload = response.get_payload();
    std::copy(response_payload.cbegin(), response_payload.cend(), serial_number.begin());

    return true;
}

bool
Client::c_error(Status& status, const uint8_t ep_id, const uint8_t trm16_id) const
{
    const Message message = enc_c_error(ep_id, trm16_id);

    if (!command(status, message))
    {
        PLOG_ERROR.printf("Control failed.");
        return false;
    }

    return true;
}

bool
Client::r_error(Status& status, const uint8_t ep_id, const uint8_t trm16_id, uint8_t& error) const
{
    const Message message = enc_r_error(ep_id, trm16_id);

    Message response{sizeof(error)};

    if (!query(status, message, response))
    {
        PLOG_ERROR.printf("Query failed.");
        return false;
    }

    const auto response_payload = response.get_payload();
    error = response_payload[0];

    return true;
}

bool
Client::reboot(Status& status, const uint8_t ep_id, const uint8_t trm16_id) const
{
    const Message message = enc_reboot(ep_id, trm16_id);

    if (!command(status, message))
    {
        PLOG_ERROR.printf("Control failed.");
        return false;
    }

    return true;
}

bool
Client::w_operation_mode(
    Status& status,
    const uint8_t ep_id,
    const uint8_t trm16_id,
    const SAS::TRM::OperationMode& operation_mode
) const
{
    const Message message = enc_w_operation_mode(ep_id, trm16_id, operation_mode);

    if (!command(status, message))
    {
        PLOG_ERROR.printf("Control failed.");
        return false;
    }

    return true;
}

bool
Client::r_operation_mode(
    Status& status,
    const uint8_t ep_id,
    const uint8_t trm16_id,
    SAS::TRM::OperationMode& operation_mode
) const
{
    const Message message = enc_r_operation_mode(ep_id, trm16_id);

    Message response{sizeof(operation_mode)};

    if (!query(status, message, response))
    {
        PLOG_ERROR.printf("Query failed.");
        return false;
    }

    const auto response_payload = response.get_payload();
    operation_mode = static_cast<SAS::TRM::OperationMode>(response_payload[0]);

    return true;
}

bool
Client::w_configuration_mode(Status& status, const uint8_t ep_id, const uint8_t trm16_id, const bool configuration_mode)
    const
{
    const Message message = enc_w_configuration_mode(ep_id, trm16_id, configuration_mode);

    if (!command(status, message))
    {
        PLOG_ERROR.printf("Control failed.");
        return false;
    }

    return true;
}

bool
Client::r_configuration_mode(Status& status, const uint8_t ep_id, const uint8_t trm16_id, bool& configuration_mode)
    const
{
    const Message message = enc_r_configuration_mode(ep_id, trm16_id);

    Message response{sizeof(configuration_mode)};

    if (!query(status, message, response))
    {
        PLOG_ERROR.printf("Query failed.");
        return false;
    }

    const auto response_payload = response.get_payload();
    configuration_mode = static_cast<bool>(response_payload[0]);

    return true;
}

bool
Client::w_uuid(Status& status, const uint8_t ep_id, const uint8_t trm16_id, const uuid_t uuid) const
{
    const Message message = enc_w_uuid(ep_id, trm16_id, uuid);

    if (!command(status, message))
    {
        PLOG_ERROR.printf("Control failed.");
        return false;
    }

    return true;
}

bool
Client::r_uuid(Status& status, const uint8_t ep_id, const uint8_t trm16_id, uuid_t& uuid) const
{
    const Message message = enc_r_uuid(ep_id, trm16_id);

    Message response{19};

    if (!query(status, message, response))
    {
        PLOG_ERROR.printf("Query failed.");
        return false;
    }

    const auto& response_payload = response.get_payload();
    std::array<uint8_t, 19> uuid_array{};
    std::copy(response_payload.cbegin(), response_payload.cend(), uuid_array.begin());
    uuid = merge_19_symbols(uuid_array);

    return true;
}

bool
Client::r_telemetry(
    Status& status,
    const uint8_t ep_id,
    const uint8_t trm16_id,
    std::array<uint8_t, Serial::c_telemetry_size>& telemetry
) const
{
    const Message message = enc_r_telemetry(ep_id, trm16_id);

    Message response{telemetry.size()};

    if (!query(status, message, response))
    {
        PLOG_ERROR.printf("Query failed.");
        return false;
    }

    const auto response_payload = response.get_payload();
    std::copy(response_payload.cbegin(), response_payload.cend(), telemetry.begin());

    return true;
}

bool
Client::c_pgt(Status& status, const uint8_t ep_id, const uint8_t trm16_id) const
{
    const Message message = enc_c_pgt(ep_id, trm16_id);

    if (!command(status, message))
    {
        PLOG_ERROR.printf("Control failed.");
        return false;
    }

    return true;
}

bool
Client::w_pgt_rows(Status& status, const uint8_t ep_id, const uint8_t trm16_id, const std::vector<SAS::Beam>& rows)
    const
{
    for (const auto& beam : rows)
    {
        if (beam.row_id >= SAS::c_beams_per_trm16)
        {
            PLOG_ERROR.printf("Invalid beam row ID: %u", beam.row_id);
            status = Status::InvalidBeamRowID;
            return false;
        }
    }

    const Message message = enc_w_pgt_rows(ep_id, trm16_id, rows);

    if (!command(status, message))
    {
        PLOG_ERROR.printf("Control failed.");
        return false;
    }

    return true;
}

bool
Client::r_pgt_rows(Status& status, const uint8_t ep_id, const uint8_t trm16_id, std::vector<SAS::Beam>& rows) const
{
    for (const auto& beam : rows)
    {
        if (beam.row_id >= SAS::c_beams_per_trm16)
        {
            PLOG_ERROR.printf("Invalid beam row ID: %u", beam.row_id);
            status = Status::InvalidBeamRowID;
            return false;
        }
    }

    const Message message = enc_r_pgt_rows(ep_id, trm16_id, rows);

    Message response{rows.size() * SAS::c_beam_size};

    if (!query(status, message, response))
    {
        PLOG_ERROR.printf("Query failed.");
        return false;
    }

    const auto response_payload = response.get_payload();

    for (size_t i_beam = 0; i_beam < rows.size(); i_beam += SAS::c_beam_size)
    {
        const uint16_t read_row_id = merge_two_symbols(response_payload[0], response_payload[1]);

        if (read_row_id != rows[i_beam].row_id)
        {
            PLOG_ERROR.printf("Invalid row ID returned from r_pgt_rows.");
            status = Status::NAck;
            return false;
        }

        // Skip the row ID (two bytes).
        rows[i_beam].row_id = read_row_id;
        std::copy(response_payload.cbegin() + 2, response_payload.cend(), rows[i_beam].coefficients.begin());
    }

    return true;
}

bool
Client::w_pgt_row(Status& status, const uint8_t ep_id, const uint8_t trm16_id, const SAS::Beam& beam) const
{
    if (beam.row_id >= SAS::c_beams_per_trm16)
    {
        PLOG_ERROR.printf("Invalid beam row ID: %u", beam.row_id);
        status = Status::InvalidBeamRowID;
        return false;
    }

    // Check that maximum coefficient value is not exceeded.
    for (const auto& coefficient : beam.coefficients)
    {
        if (coefficient > SAS::c_max_beam_coefficient_value)
        {
            PLOG_ERROR.printf("Coefficient value exceeds maximum value of %u.", SAS::c_max_beam_coefficient_value);
            status = Status::InvalidBeamCoefficientValue;
            return false;
        }
    }

    const Message message = enc_w_pgt_row(ep_id, trm16_id, beam);

    if (!command(status, message))
    {
        PLOG_ERROR.printf("Control failed.");
        return false;
    }

    return true;
}

bool
Client::r_pgt_row(Status& status, const uint8_t ep_id, const uint8_t trm16_id, SAS::Beam& beam) const
{
    if (beam.row_id >= SAS::c_beams_per_trm16)
    {
        PLOG_ERROR.printf("Invalid beam row ID: %u", beam.row_id);
        status = Status::InvalidBeamRowID;
        return false;
    }

    const Message message = enc_r_pgt_row(ep_id, trm16_id, beam);

    Message response{sizeof(SAS::Beam)};

    if (!query(status, message, response))
    {
        PLOG_ERROR.printf("Query failed.");
        return false;
    }

    const auto response_payload = response.get_payload();
    // Check if the row ID is correct.
    const uint16_t read_row_id = merge_two_symbols(response_payload[0], response_payload[1]);

    if (read_row_id != beam.row_id)
    {
        PLOG_ERROR.printf("Invalid row ID returned from r_pgt_row.");
        return false;
    }

    // Skip the row ID (two bytes).
    beam.row_id = read_row_id;
    std::copy(response_payload.cbegin() + 2, response_payload.cend(), beam.coefficients.begin());

    return true;
}

bool
Client::w_pgt_entry(
    Status& status,
    const uint8_t ep_id,
    const uint8_t trm16_id,
    const uint16_t row_id,
    const uint8_t trm_id,
    const std::array<uint8_t, 3>& entry
) const
{
    if (row_id >= SAS::c_beams_per_trm16)
    {
        PLOG_ERROR.printf("Invalid beam row ID: %u", row_id);
        status = Status::InvalidBeamRowID;
        return false;
    }

    const Message message = enc_w_pgt_entry(ep_id, trm16_id, row_id, trm_id, entry);

    if (!command(status, message))
    {
        PLOG_ERROR.printf("Control failed.");
        return false;
    }

    return true;
}

bool
Client::r_pgt_entry(
    Status& status,
    const uint8_t ep_id,
    const uint8_t trm16_id,
    const uint16_t row_id,
    const uint8_t trm_id,
    std::array<uint8_t, 3>& entry
) const
{
    if (row_id >= SAS::c_beams_per_trm16)
    {
        PLOG_ERROR.printf("Invalid beam row ID: %u", row_id);
        status = Status::InvalidBeamRowID;
        return false;
    }

    const Message message = enc_r_pgt_entry(ep_id, trm16_id, row_id, trm_id);

    Message response{sizeof(row_id) + sizeof(trm_id) + entry.size()};

    if (!query(status, message, response))
    {
        PLOG_ERROR.printf("Query failed.");
        return false;
    }

    const auto response_payload = response.get_payload();
    // Check if the row and TRM16 IDs are correct.
    const uint16_t read_row_id = merge_two_symbols(response_payload[0], response_payload[1]);

    if (read_row_id != row_id)
    {
        PLOG_ERROR.printf("Invalid row ID returned from r_pgt_row.");
        return false;
    }

    const uint8_t read_trm16_id = response_payload[2];

    if (read_trm16_id != trm_id)
    {
        PLOG_ERROR.printf("Invalid TRM16 ID returned from r_pgt_row.");
        return false;
    }

    // Skip the row ID and TRM16 ID (three bytes).
    std::copy(response_payload.cbegin() + 3, response_payload.cend(), entry.begin());

    return true;
}

bool
Client::w_pgti(Status& status, const uint8_t ep_id, const uint8_t trm16_id, const uint16_t pgti) const
{
    if (pgti >= SAS::c_beams_per_trm16)
    {
        PLOG_ERROR.printf("Invalid beam row ID: %u", pgti);
        status = Status::InvalidBeamRowID;
        return false;
    }

    const Message message = enc_w_pgti(ep_id, trm16_id, pgti);

    if (!command(status, message))
    {
        PLOG_ERROR.printf("Control failed.");
        return false;
    }

    return true;
}

bool
Client::r_pgti(Status& status, const uint8_t ep_id, const uint8_t trm16_id, uint16_t& pgti) const
{
    const Message message = enc_r_pgti(ep_id, trm16_id);

    Message response{sizeof(pgti)};

    if (!query(status, message, response))
    {
        PLOG_ERROR.printf("Query failed.");
        return false;
    }

    const auto response_payload = response.get_payload();
    pgti = merge_two_symbols(response_payload[0], response_payload[1]);

    return true;
}

bool
Client::w_pgti_bounds(
    Status& status,
    const uint8_t ep_id,
    const uint8_t trm16_id,
    const uint16_t pgti_lower_bound,
    const uint16_t pgti_upper_bound
) const
{
    if (pgti_lower_bound >= SAS::c_beams_per_trm16)
    {
        PLOG_ERROR.printf("Invalid beam row ID: %u", pgti_lower_bound);
        status = Status::InvalidBeamRowID;
        return false;
    }

    if (pgti_upper_bound >= SAS::c_beams_per_trm16)
    {
        PLOG_ERROR.printf("Invalid beam row ID: %u", pgti_upper_bound);
        status = Status::InvalidBeamRowID;
        return false;
    }

    const Message message = enc_w_pgti_bounds(ep_id, trm16_id, pgti_lower_bound, pgti_upper_bound);

    if (!command(status, message))
    {
        PLOG_ERROR.printf("Control failed.");
        return false;
    }

    return true;
}

bool
Client::r_pgti_bounds(
    Status& status,
    const uint8_t ep_id,
    const uint8_t trm16_id,
    uint16_t& pgti_lower_bound,
    uint16_t& pgti_upper_bound
) const
{
    const Message message = enc_r_pgti_bounds(ep_id, trm16_id);

    Message response{sizeof(pgti_lower_bound) + sizeof(pgti_upper_bound)};

    if (!query(status, message, response))
    {
        PLOG_ERROR.printf("Query failed.");
        return false;
    }

    const auto response_payload = response.get_payload();
    const uint8_t pgti_lower_bound_lsb = response_payload[0];
    const uint8_t pgti_lower_bound_msb = response_payload[1];
    const uint8_t pgti_upper_bound_lsb = response_payload[2];
    const uint8_t pgti_upper_bound_msb = response_payload[3];

    pgti_lower_bound = merge_two_symbols(pgti_lower_bound_lsb, pgti_lower_bound_msb);

    pgti_upper_bound = merge_two_symbols(pgti_upper_bound_lsb, pgti_upper_bound_msb);

    return true;
}

bool
Client::w_index_strobe_mode(Status& status, const uint8_t ep_id, const uint8_t trm16_id, const bool index_strobe_mode)
    const
{
    const Message message = enc_w_index_strobe_mode(ep_id, trm16_id, index_strobe_mode);

    if (!command(status, message))
    {
        PLOG_ERROR.printf("Control failed.");
        return false;
    }

    return true;
}

bool
Client::r_index_strobe_mode(Status& status, const uint8_t ep_id, const uint8_t trm16_id, bool& index_strobe_mode) const
{
    const Message message = enc_r_index_strobe_mode(ep_id, trm16_id);

    Message response{sizeof(index_strobe_mode)};

    if (!query(status, message, response))
    {
        PLOG_ERROR.printf("Query failed.");
        return false;
    }

    const auto response_payload = response.get_payload();
    index_strobe_mode = response_payload[0];

    return true;
}

bool
Client::w_temperature_characterization(Status& status, const uint8_t ep_id, const uint8_t trm16_id, const bool tc) const
{
    const Message message = enc_w_temperature_characterization(ep_id, trm16_id, tc);

    if (!command(status, message))
    {
        PLOG_ERROR.printf("Control failed.");
        return false;
    }

    return true;
}

bool
Client::r_temperature_characterization(
    Status& status,
    const uint8_t ep_id,
    const uint8_t trm16_id,
    bool& temperature_characterization
) const
{
    const Message message = enc_r_temperature_characterization(ep_id, trm16_id);

    Message response{sizeof(temperature_characterization)};

    if (!query(status, message, response))
    {
        PLOG_ERROR.printf("Query failed.");
        return false;
    }

    const auto response_payload = response.get_payload();
    temperature_characterization = response_payload[0];

    return true;
}

bool
Client::w_tt(Status& status, const uint8_t ep_id, const uint8_t trm16_id, const std::array<uint8_t, 8 * 2>& tt) const
{
    const Message message = enc_w_tt(ep_id, trm16_id, tt);

    if (!command(status, message))
    {
        PLOG_ERROR.printf("Control failed.");
        return false;
    }

    return true;
}

bool
Client::r_tt(Status& status, const uint8_t ep_id, const uint8_t trm16_id, std::array<uint8_t, 8 * 2>& tt) const
{
    const Message message = enc_r_tt(ep_id, trm16_id);

    Message response{tt.size()};

    if (!query(status, message, response))
    {
        PLOG_ERROR.printf("Query failed.");
        return false;
    }

    const auto response_payload = response.get_payload();
    std::copy(response_payload.cbegin(), response_payload.cend(), tt.begin());

    return true;
}

bool
Client::w_tti(Status& status, const uint8_t ep_id, const uint8_t trm16_id, const uint8_t tti) const
{
    const Message message = enc_w_tti(ep_id, trm16_id, tti);

    if (!command(status, message))
    {
        PLOG_ERROR.printf("Control failed.");
        return false;
    }

    return true;
}

bool
Client::r_tti(Status& status, const uint8_t ep_id, const uint8_t trm16_id, uint8_t& tti) const
{
    const Message message = enc_r_tti(ep_id, trm16_id);

    Message response{sizeof(tti)};

    if (!query(status, message, response))
    {
        PLOG_ERROR.printf("Query failed.");
        return false;
    }

    const auto response_payload = response.get_payload();
    tti = response_payload[0];

    return true;
}

bool
Client::w_power(Status& status, const uint8_t ep_id, const uint8_t power_mask_main, const uint8_t power_mask_redundant)
    const
{
    const Message message = enc_w_power(ep_id, power_mask_main, power_mask_redundant);

    if (!command(status, message))
    {
        PLOG_ERROR.printf("Control failed.");
        return false;
    }

    return true;
}

bool
Client::r_power(Status& status, const uint8_t ep_id, uint8_t& power_mask_main, uint8_t& power_mask_redundant) const
{
    const Message message = enc_r_power(ep_id);

    Message response{4 * sizeof(uint8_t)};  // Two power masks.

    if (!query(status, message, response))
    {
        PLOG_ERROR.printf("Query failed.");
        return false;
    }

    const std::vector<uint8_t> response_payload = response.get_payload();
    power_mask_main = static_cast<uint8_t>(merge_two_symbols(response_payload.at(0), response_payload.at(1)));
    power_mask_redundant = static_cast<uint8_t>(merge_two_symbols(response_payload.at(2), response_payload.at(3)));

    return true;
}

bool
Client::w_override_temp_mode(Status& status, const uint8_t ep_id, const uint8_t trm16_id, const bool override_temp_mode)
    const
{
    const Message message = enc_w_override_temp_mode(ep_id, trm16_id, override_temp_mode);

    if (!command(status, message))
    {
        PLOG_ERROR.printf("Control failed.");
        return false;
    }

    return true;
}

bool
Client::r_override_temp_mode(Status& status, const uint8_t ep_id, const uint8_t trm16_id, bool& override_temp_mode)
    const
{
    const Message message = enc_r_override_temp_mode(ep_id, trm16_id);

    Message response{sizeof(override_temp_mode)};

    if (!query(status, message, response))
    {
        PLOG_ERROR.printf("Query failed.");
        return false;
    }

    const auto& response_payload = response.get_payload();
    override_temp_mode = response_payload[0];

    return true;
}

bool
Client::w_override_temp(Status& status, const uint8_t ep_id, const uint8_t trm16_id, const uint8_t override_temp) const
{
    const Message message = enc_w_override_temp(ep_id, trm16_id, override_temp);

    if (!command(status, message))
    {
        PLOG_ERROR.printf("Control failed.");
        return false;
    }

    return true;
}

bool
Client::r_override_temp(Status& status, const uint8_t ep_id, const uint8_t trm16_id, uint8_t& override_temp) const
{
    const Message message = enc_r_override_temp(ep_id, trm16_id);

    Message response{2 * sizeof(override_temp)};

    if (!query(status, message, response))
    {
        PLOG_ERROR.printf("Query failed.");
        return false;
    }

    const auto& response_payload = response.get_payload();
    override_temp = response_payload[0];

    return true;
}

bool
Client::r_cc_regs(
    Status& status,
    const uint8_t ep_id,
    const uint8_t trm16_id,
    std::array<uint16_t, SAS::c_cc_regs_per_trm16>& cc_regs
) const
{
    const Message message = enc_r_cc_regs(ep_id, trm16_id);

    Message response{SAS::c_cc_regs_per_trm16 * sizeof(uint16_t)};

    if (!query(status, message, response))
    {
        PLOG_ERROR.printf("Query failed.");
        return false;
    }

    const auto& response_payload = response.get_payload();
    for (size_t i = 0; i < cc_regs.size(); ++i)
    {
        cc_regs[i] = merge_two_symbols(response_payload[i * 2], response_payload[i * 2 + 1]);
    }

    return true;
}

Message
Client::enc_r_id(const uint8_t ep_id, const uint8_t trm16_id) const
{
    const uint8_t cmd_id = 1;
    Message message{ep_id, trm16_id, cmd_id};
    return message;
}

Message
Client::enc_r_serial_number(const uint8_t ep_id, const uint8_t trm16_id) const
{
    const uint8_t cmd_id = 2;
    Message message{ep_id, trm16_id, cmd_id};
    return message;
}

Message
Client::enc_c_error(const uint8_t ep_id, const uint8_t trm16_id) const
{
    const uint8_t cmd_id = 3;
    Message message{ep_id, trm16_id, cmd_id};
    return message;
}

Message
Client::enc_r_error(const uint8_t ep_id, const uint8_t trm16_id) const
{
    const uint8_t cmd_id = 4;
    Message message{ep_id, trm16_id, cmd_id};
    return message;
}

Message
Client::enc_reboot(const uint8_t ep_id, const uint8_t trm16_id) const
{
    const uint8_t cmd_id = 5;
    Message message{ep_id, trm16_id, cmd_id};
    return message;
}

Message
Client::enc_w_operation_mode(const uint8_t ep_id, const uint8_t trm16_id, const SAS::TRM::OperationMode operation_mode)
    const
{
    const uint8_t cmd_id = 6;
    const std::vector<uint8_t> payload{static_cast<uint8_t>(operation_mode)};
    Message message{ep_id, trm16_id, cmd_id, payload};
    return message;
}

Message
Client::enc_r_operation_mode(const uint8_t ep_id, const uint8_t trm16_id) const
{
    const uint8_t cmd_id = 7;
    Message message{ep_id, trm16_id, cmd_id};
    return message;
}

Message
Client::enc_w_configuration_mode(const uint8_t ep_id, const uint8_t trm16_id, const bool configuration_mode) const
{
    const uint8_t cmd_id = 8;
    const std::vector<uint8_t> payload{static_cast<uint8_t>(configuration_mode)};
    Message message{ep_id, trm16_id, cmd_id, payload};
    return message;
}

Message
Client::enc_r_configuration_mode(const uint8_t ep_id, const uint8_t trm16_id) const
{
    const uint8_t cmd_id = 9;
    Message message{ep_id, trm16_id, cmd_id};
    return message;
}

Message
Client::enc_w_uuid(const uint8_t ep_id, const uint8_t trm16_id, const uuid_t uuid) const
{
    const uint8_t cmd_id = 10;
    const auto split = split_into_19_symbols(uuid);
    const std::vector<uint8_t> payload(split.cbegin(), split.cend());
    Message message{ep_id, trm16_id, cmd_id, payload};
    return message;
}

Message
Client::enc_r_uuid(const uint8_t ep_id, const uint8_t trm16_id) const
{
    const uint8_t cmd_id = 11;
    Message message{ep_id, trm16_id, cmd_id};
    return message;
}

Message
Client::enc_r_telemetry(const uint8_t ep_id, const uint8_t trm16_id) const
{
    const uint8_t cmd_id = 12;
    Message message{ep_id, trm16_id, cmd_id};
    return message;
}

Message
Client::enc_c_pgt(const uint8_t ep_id, const uint8_t trm16_id) const
{
    const uint8_t cmd_id = 13;
    Message message{ep_id, trm16_id, cmd_id};
    return message;
}

Message
Client::enc_w_pgt_rows(const uint8_t ep_id, const uint8_t trm16_id, const std::vector<SAS::Beam>& beams) const
{
    const uint8_t cmd_id = 14;
    std::vector<uint8_t> payload;
    payload.reserve(beams.size() * SAS::c_beam_size);

    for (const auto& beam : beams)
    {
        const auto [row_id_lower, row_id_upper] = split_into_two_symbols(beam.row_id);
        payload.push_back(row_id_lower);
        payload.push_back(row_id_upper);

        std::copy(beam.coefficients.cbegin(), beam.coefficients.cend(), std::back_inserter(payload));
    }

    Message message{ep_id, trm16_id, cmd_id, payload};
    return message;
}

Message
Client::enc_r_pgt_rows(const uint8_t ep_id, const uint8_t trm16_id, const std::vector<SAS::Beam>& beams) const
{
    const uint8_t cmd_id = 15;
    std::vector<uint8_t> payload;
    payload.reserve(beams.size() * SAS::c_row_id_size);

    for (const auto& beam : beams)
    {
        const auto [row_id_lower, row_id_upper] = split_into_two_symbols(beam.row_id);
        payload.push_back(row_id_lower);
        payload.push_back(row_id_upper);
    }

    Message message{ep_id, trm16_id, cmd_id, payload};
    return message;
}

Message
Client::enc_w_pgt_row(const uint8_t ep_id, const uint8_t trm16_id, const SAS::Beam& beam) const
{
    const uint8_t cmd_id = 16;
    std::vector<uint8_t> payload;
    payload.reserve(SAS::c_beam_size);
    const auto [row_id_lower, row_id_upper] = split_into_two_symbols(beam.row_id);
    payload.push_back(row_id_lower);
    payload.push_back(row_id_upper);
    std::copy(beam.coefficients.cbegin(), beam.coefficients.cend(), std::back_inserter(payload));
    Message message{ep_id, trm16_id, cmd_id, payload};
    return message;
}

Message
Client::enc_r_pgt_row(const uint8_t ep_id, const uint8_t trm16_id, const SAS::Beam& beam) const
{
    const uint8_t cmd_id = 17;
    const auto [row_id_lower, row_id_upper] = split_into_two_symbols(beam.row_id);
    std::vector<uint8_t> payload{row_id_lower, row_id_upper};
    Message message{ep_id, trm16_id, cmd_id, payload};
    return message;
}

Message
Client::enc_w_pgt_entry(
    const uint8_t ep_id,
    const uint8_t trm16_id,
    const uint16_t row_id,
    const uint8_t trm_id,
    const std::array<uint8_t, 3>& entry
) const
{
    const uint8_t cmd_id = 18;
    std::vector<uint8_t> payload(entry.size() + 3);
    const auto [row_id_lower, row_id_upper] = split_into_two_symbols(row_id);
    payload[0] = row_id_lower;
    payload[1] = row_id_upper;
    payload[2] = trm_id;
    std::copy(entry.cbegin(), entry.cend(), payload.begin() + 3);
    Message message{ep_id, trm16_id, cmd_id, payload};
    return message;
}

Message
Client::enc_r_pgt_entry(const uint8_t ep_id, const uint8_t trm16_id, const uint16_t row_id, const uint8_t trm_id) const
{
    const uint8_t cmd_id = 19;
    const auto [row_id_lower, row_id_upper] = split_into_two_symbols(row_id);
    std::vector<uint8_t> payload{row_id_lower, row_id_upper, trm_id};
    Message message{ep_id, trm16_id, cmd_id, payload};
    return message;
}

Message
Client::enc_w_pgti(const uint8_t ep_id, const uint8_t trm16_id, const uint16_t pgti) const
{
    const uint8_t cmd_id = 20;
    const auto [pgti_lower, pgti_upper] = split_into_two_symbols(pgti);
    std::vector<uint8_t> payload{pgti_lower, pgti_upper};
    Message message{ep_id, trm16_id, cmd_id, payload};
    return message;
}

Message
Client::enc_r_pgti(const uint8_t ep_id, const uint8_t trm16_id) const
{
    const uint8_t cmd_id = 21;
    Message message{ep_id, trm16_id, cmd_id};
    return message;
}

Message
Client::enc_w_pgti_bounds(
    const uint8_t ep_id,
    const uint8_t trm16_id,
    const uint16_t lower_bound,
    const uint16_t upper_bound
) const
{
    const uint8_t cmd_id = 22;
    const auto [lower_bound_lower, lower_bound_upper] = split_into_two_symbols(lower_bound);
    const auto [upper_bound_lower, upper_bound_upper] = split_into_two_symbols(upper_bound);
    std::vector<uint8_t> payload{lower_bound_lower, lower_bound_upper, upper_bound_lower, upper_bound_upper};
    Message message{ep_id, trm16_id, cmd_id, payload};
    return message;
}

Message
Client::enc_r_pgti_bounds(const uint8_t ep_id, const uint8_t trm16_id) const
{
    const uint8_t cmd_id = 23;
    Message message{ep_id, trm16_id, cmd_id};
    return message;
}

Message
Client::enc_w_index_strobe_mode(const uint8_t ep_id, const uint8_t trm16_id, const bool index_strobe_mode) const
{
    const uint8_t cmd_id = 24;
    const std::vector<uint8_t> payload{static_cast<uint8_t>(index_strobe_mode)};
    Message message{ep_id, trm16_id, cmd_id, payload};
    return message;
}

Message
Client::enc_r_index_strobe_mode(const uint8_t ep_id, const uint8_t trm16_id) const
{
    const uint8_t cmd_id = 25;
    Message message{ep_id, trm16_id, cmd_id};
    return message;
}

Message
Client::enc_w_temperature_characterization(const uint8_t ep_id, const uint8_t trm16_id, const bool tc) const
{
    const uint8_t cmd_id = 26;
    const std::vector<uint8_t> payload{static_cast<uint8_t>(tc)};
    Message message{ep_id, trm16_id, cmd_id, payload};
    return message;
}

Message
Client::enc_r_temperature_characterization(const uint8_t ep_id, const uint8_t trm16_id) const
{
    const uint8_t cmd_id = 27;
    Message message{ep_id, trm16_id, cmd_id};
    return message;
}

Message
Client::enc_w_tt(const uint8_t ep_id, const uint8_t trm16_id, const std::array<uint8_t, 8 * 2>& tt) const
{
    const uint8_t cmd_id = 28;
    std::vector<uint8_t> payload(tt.cbegin(), tt.cend());
    Message message{ep_id, trm16_id, cmd_id, payload};
    return message;
}

Message
Client::enc_r_tt(const uint8_t ep_id, const uint8_t trm16_id) const
{
    const uint8_t cmd_id = 29;
    Message message{ep_id, trm16_id, cmd_id};
    return message;
}

Message
Client::enc_w_tti(const uint8_t ep_id, const uint8_t trm16_id, const uint8_t tti) const
{
    const uint8_t cmd_id = 30;
    std::vector<uint8_t> payload{tti};
    Message message{ep_id, trm16_id, cmd_id, payload};
    return message;
}

Message
Client::enc_r_tti(const uint8_t ep_id, const uint8_t trm16_id) const
{
    const uint8_t cmd_id = 31;
    Message message{ep_id, trm16_id, cmd_id};
    return message;
}

Message
Client::enc_w_power(const uint8_t ep_id, const uint8_t power_mask_main, const uint8_t power_mask_redundant) const
{
    const uint8_t cmd_id = 32;
    const auto power_mask_main_array = split_into_two_symbols(power_mask_main);
    const auto power_mask_redundant_array = split_into_two_symbols(power_mask_redundant);
    const std::vector<uint8_t> payload{
        power_mask_main_array.first,
        power_mask_main_array.second,
        power_mask_redundant_array.first,
        power_mask_redundant_array.second};
    Message message{ep_id, 0, cmd_id, payload};
    return message;
}

Message
Client::enc_r_power(const uint8_t ep_id) const
{
    const uint8_t cmd_id = 33;
    Message message{ep_id, 0, cmd_id};
    return message;
}

Message
Client::enc_r_cc_regs(const uint8_t ep_id, const uint8_t trm16_id) const
{
    const uint8_t cmd_id = 80;
    Message message{ep_id, trm16_id, cmd_id};
    return message;
}

Message
Client::enc_w_override_temp_mode(const uint8_t ep_id, const uint8_t trm16_id, const bool override_temp_mode) const
{
    const uint8_t cmd_id = 81;
    const std::vector<uint8_t> payload{static_cast<uint8_t>(override_temp_mode)};
    Message message{ep_id, trm16_id, cmd_id, payload};
    return message;
}

Message
Client::enc_r_override_temp_mode(const uint8_t ep_id, const uint8_t trm16_id) const
{
    const uint8_t cmd_id = 82;
    Message message{ep_id, trm16_id, cmd_id};
    return message;
}

Message
Client::enc_w_override_temp(const uint8_t ep_id, const uint8_t trm16_id, const uint8_t override_temp) const
{
    const uint8_t cmd_id = 83;
    std::vector<uint8_t> payload{override_temp, 0};
    Message message{ep_id, trm16_id, cmd_id, payload};
    return message;
}

Message
Client::enc_r_override_temp(const uint8_t ep_id, const uint8_t trm16_id) const
{
    const uint8_t cmd_id = 84;
    Message message{ep_id, trm16_id, cmd_id};
    return message;
}

std::array<uint8_t, 19>
split_into_19_symbols(__uint128_t value)
{
    return {
        static_cast<uint8_t>(value & 0x7F),
        static_cast<uint8_t>((value >> 7) & 0x7F),
        static_cast<uint8_t>((value >> 14) & 0x7F),
        static_cast<uint8_t>((value >> 21) & 0x7F),
        static_cast<uint8_t>((value >> 28) & 0x7F),
        static_cast<uint8_t>((value >> 35) & 0x7F),
        static_cast<uint8_t>((value >> 42) & 0x7F),
        static_cast<uint8_t>((value >> 49) & 0x7F),
        static_cast<uint8_t>((value >> 56) & 0x7F),
        static_cast<uint8_t>((value >> 63) & 0x7F),
        static_cast<uint8_t>((value >> 70) & 0x7F),
        static_cast<uint8_t>((value >> 77) & 0x7F),
        static_cast<uint8_t>((value >> 84) & 0x7F),
        static_cast<uint8_t>((value >> 91) & 0x7F),
        static_cast<uint8_t>((value >> 98) & 0x7F),
        static_cast<uint8_t>((value >> 105) & 0x7F),
        static_cast<uint8_t>((value >> 112) & 0x7F),
        static_cast<uint8_t>((value >> 119) & 0x7F),
        static_cast<uint8_t>((value >> 126) & 0x7F)};
}

__uint128_t
merge_19_symbols(const std::array<uint8_t, 19>& symbols)
{
    return static_cast<__uint128_t>(symbols[0]) | static_cast<__uint128_t>(symbols[1]) << 7 |
        static_cast<__uint128_t>(symbols[2]) << 14 | static_cast<__uint128_t>(symbols[3]) << 21 |
        static_cast<__uint128_t>(symbols[4]) << 28 | static_cast<__uint128_t>(symbols[5]) << 35 |
        static_cast<__uint128_t>(symbols[6]) << 42 | static_cast<__uint128_t>(symbols[7]) << 49 |
        static_cast<__uint128_t>(symbols[8]) << 56 | static_cast<__uint128_t>(symbols[9]) << 63 |
        static_cast<__uint128_t>(symbols[10]) << 70 | static_cast<__uint128_t>(symbols[11]) << 77 |
        static_cast<__uint128_t>(symbols[12]) << 84 | static_cast<__uint128_t>(symbols[13]) << 91 |
        static_cast<__uint128_t>(symbols[14]) << 98 | static_cast<__uint128_t>(symbols[15]) << 105 |
        static_cast<__uint128_t>(symbols[16]) << 112 | static_cast<__uint128_t>(symbols[17]) << 119 |
        static_cast<__uint128_t>(symbols[18]) << 126;
}

int
baud_rate_to_int(const speed_t speed)
{
    switch (speed)
    {
        case B0:
            return 0;
        case B50:
            return 50;
        case B75:
            return 75;
        case B110:
            return 110;
        case B134:
            return 134;
        case B150:
            return 150;
        case B200:
            return 200;
        case B300:
            return 300;
        case B600:
            return 600;
        case B1200:
            return 1200;
        case B1800:
            return 1800;
        case B2400:
            return 2400;
        case B4800:
            return 4800;
        case B9600:
            return 9600;
        case B19200:
            return 19200;
        case B38400:
            return 38400;
        case B57600:
            return 57600;
        case B115200:
            return 115200;
        case B230400:
            return 230400;
        case B460800:
            return 460800;
        case B500000:
            return 500000;
        case B576000:
            return 576000;
        case B921600:
            return 921600;
        case B1000000:
            return 1000000;
        case B1152000:
            return 1152000;
        case B1500000:
            return 1500000;
        case B2000000:
            return 2000000;
        case B2500000:
            return 2500000;
        case B3000000:
            return 3000000;
        case B3500000:
            return 3500000;
        case B4000000:
            return 4000000;
        default:
            return -1;  // Invalid or unknown baud rate
    }
}

}  // namespace Serial
